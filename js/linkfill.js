// Trial: filled links. Each printed part becomes one solid: an input connector, a straight pipe
// with a 90° bend to each output (like the Model view's rods), and an output connector (or the end
// sphere) on each output. Pipes are round or square tubes (hooks.tubeShape, toggled in the CAD tab);
// square ones have both runs squared to the plane of their bend, with a cube filling the elbow.
//  - Links attached straight to each other (no motor between) are rigid together, so they print
//    as one part: one input connector, the outputs of the last link, and pipes routed from the
//    first input to those outputs. The first link of the run draws and exports the whole part.
//  - Each pipe end is fused on with a tapered collar: a cone from the full connector face to the
//    pipe, sunk into the connector's solid wall, so it is physically joined rather than touching.
//  - Then the volume the motor before the part sweeps as it turns is cut away, as is the envelope of
//    each motor bolted to an output, and the space in front of every mating face is kept clear.
// Booleans use manifold-3d (WASM), served from vendor/ because the preview host blocks loading
// WebAssembly from a CDN. The CAD tab shows the parts and the link inspector exports their STL.
// To remove the trial, delete the `import './linkfill.js'` line in main.js.
import * as THREE from 'three';
import { state, node, isCapped, emit } from './robot.js';
import { localKinematics, forwardKinematics, rodDirection } from './kinematics.js';
import { hooks, cadShellGeometry, CAD_UP } from './viewer.js';
import { LINK } from './constants.js';

const MANIFOLD_URL = new URL('../vendor/manifold-3d-2.5.1/manifold.js', import.meta.url).href;
const PIPE = { radius: LINK.radius }; // same as the Model view's rods
const PLATE = { radius: 35, thickness: 3 }; // backing plate behind each output, matching the next part's face
const COLLAR = {
  length: 22, // mm from a connector face to where its cone meets the pipe
  sink: 1, // mm the collar sinks into each connector's solid wall (input top is ~3.6 mm, output base ~3 mm)
  fuseRadius: 30, // inside the output spigot's 31 mm solid base
};
const FACE_CLEARANCE = { radius: 38, depth: 30 }; // kept empty in front of every mating face
const SWEEP_CLEARANCE = 1.5; // mm around the motor before the part, as it turns
const SWEEP_BIN = 2; // mm slices of that swept volume
const SEGMENTS = 64;

let wasm = null; // the manifold-3d module once loaded
let loading = null;
const loadManifold = () => (loading ??= import(MANIFOLD_URL).then(async (m) => {
  wasm = await m.default();
  wasm.setup();
}));

const cache = new Map(); // signature → { geometry, triangles } once built, or 'pending' / 'failed'
let queue = Promise.resolve();
const EMPTY = new THREE.BufferGeometry(); // what a link drawn as part of another one shows

// ---- Helpers ----

/** Vertex positions of a loaded shell as [x, y, z] points, optionally moved by `m`. */
function points(geometry, m) {
  const p = geometry.attributes.position.array, out = [];
  const v = new THREE.Vector3();
  for (let i = 0; i < p.length; i += 3) {
    v.set(p[i], p[i + 1], p[i + 2]);
    if (m) v.applyMatrix4(m);
    out.push([v.x, v.y, v.z]);
  }
  return out;
}

/** A triangle-soup geometry as a Manifold (welding the duplicate vertices first). */
function toManifold(geometry) {
  const vertProperties = Float32Array.from(geometry.attributes.position.array);
  const triVerts = Uint32Array.from({ length: vertProperties.length / 3 }, (_, i) => i);
  const mesh = new wasm.Mesh({ numProp: 3, vertProperties, triVerts });
  mesh.merge();
  return new wasm.Manifold(mesh);
}

/** Points of a circle of radius r at height z in the frame m. */
const circle = (r, z, m) => Array.from({ length: SEGMENTS }, (_, k) => {
  const a = (k / SEGMENTS) * Math.PI * 2;
  return new THREE.Vector3(r * Math.cos(a), r * Math.sin(a), z).applyMatrix4(m).toArray();
});

const ball = (c, r) => wasm.Manifold.sphere(r, SEGMENTS).translate(c.toArray());

/** Straight pipe between two points with round ends: the hull of a ball at each. */
function capsule(a, b, ra, rb = ra) {
  const A = ball(a, ra), B = ball(b, rb);
  const c = wasm.Manifold.hull([A, B]);
  free(A, B);
  return c;
}

/**
 * Square-tube pieces. `basis` is three perpendicular unit vectors; every bar and cube of one tube
 * uses the same three, so the runs meet the elbow cube face to face.
 */
function squareCube(c, basis, r) {
  const pts = [];
  for (const a of [-r, r]) for (const b of [-r, r]) for (const d of [-r, r]) {
    pts.push(c.clone().addScaledVector(basis[0], a).addScaledVector(basis[1], b).addScaledVector(basis[2], d).toArray());
  }
  return wasm.Manifold.hull(pts);
}

/** Square bar from a to b (side 2r); its sides lie along the two basis vectors across it. */
function squareBar(a, b, basis, r) {
  const along = b.clone().sub(a).normalize();
  const [u, v] = basis.filter((e) => Math.abs(e.dot(along)) < 0.5);
  const pts = [];
  for (const p of [a, b]) for (const su of [-r, r]) for (const sv of [-r, r]) pts.push(p.clone().addScaledVector(u, su).addScaledVector(v, sv).toArray());
  return wasm.Manifold.hull(pts);
}

/** Three perpendicular unit vectors for a tube that runs along d1, then (optionally) d2 ⊥ d1. */
function tubeBasis(d1, d2) {
  let n = d2 ? new THREE.Vector3().crossVectors(d1, d2) : null;
  if (!n || n.lengthSq() < 1e-6) n = new THREE.Vector3().crossVectors(d1, Math.abs(d1.z) < 0.9 ? new THREE.Vector3(0, 0, 1) : new THREE.Vector3(1, 0, 0));
  n.normalize();
  return [d1.clone().normalize(), new THREE.Vector3().crossVectors(n, d1).normalize(), n];
}

/** Every point of an actuator's shells (motor, built-in gearbox, extra stages) in its input frame. */
function actuatorPoints(n) {
  const kin = localKinematics(n);
  const shells = cadShellGeometry();
  const pts = [...points(shells.motor, kin.mount), ...points(shells.firstGearbox, kin.mount)];
  for (const s of kin.segments.slice(2)) pts.push(...points(shells.gearbox, s.start));
  return { pts, kin };
}

const free = (...ms) => ms.forEach((m) => m?.delete());

// ---- Parts: runs of links with no motor between them ----

const isLink = (n) => n?.type === 'link' && n.parent; // the base is the world anchor, not a part

/** The first link of the run of links this one belongs to. */
function partRoot(n) {
  while (isLink(node(n.parent.nodeId))) n = node(n.parent.nodeId);
  return n;
}

/**
 * A part: its links, and its outputs in the first link's input frame. Each output knows its end
 * (sphere radius or 0), what is attached, and the direction its pipe leaves the input.
 */
function describePart(root) {
  const frames = forwardKinematics(state.robot);
  const toRoot = frames[root.id].input.clone().invert();
  const links = [], outputs = [];
  const visit = (x, rise) => {
    links.push(x);
    x.outputs.forEach((o, i) => {
      // The pipe leaves along the first link's rod for an XYZ link, straight up for a D-H one.
      const dir = rise ?? (x.spec === 'pose' ? rodDirection(o.rod).normalize() : new THREE.Vector3(0, 0, 1));
      const child = x.children[i] && node(x.children[i]);
      if (isLink(child)) return visit(child, dir);
      outputs.push({ m: toRoot.clone().multiply(frames[x.id].outputs[i]), sphere: isCapped(x, i) ? o.sphere : 0, child, dir });
    });
  };
  visit(root, null);
  return { root, links, outputs, shape: hooks.tubeShape };
}

/** What a part's solid depends on: its outputs, and the motors on either side of it. */
function signature(part) {
  const parent = node(part.root.parent.nodeId);
  const r3 = (v) => Math.round(v * 1000) / 1000;
  return JSON.stringify({
    shape: part.shape,
    outputs: part.outputs.map((o) => [o.m.elements.map(r3), o.sphere, o.dir.toArray().map(r3),
      o.child?.type === 'actuator' ? [o.child.motor, o.child.gearboxes.length, o.child.mount] : null]),
    parent: parent.type === 'actuator' ? [parent.motor, parent.gearboxes.length] : null,
  });
}

// ---- The solid ----

function build(part) {
  const { Manifold } = wasm;
  const shells = cadShellGeometry();
  const R = PIPE.radius;
  if (!shells.input.boundingBox) shells.input.computeBoundingBox();
  const top = shells.input.boundingBox.max.z;
  // Centre and size of the input's back face (its top).
  const back = points(shells.input).filter((p) => p[2] > top - 0.3);
  const backBox = new THREE.Box3().setFromArray(back.flat());
  const faceRadius = Math.min(backBox.max.x - backBox.min.x, backBox.max.y - backBox.min.y) / 2;
  const centre = backBox.getCenter(new THREE.Vector3());
  const I = new THREE.Matrix4();
  const atInput = new THREE.Matrix4().makeTranslation(centre.x, centre.y, top);

  const square = part.shape === 'square';
  const pieces = [];
  const start = new THREE.Vector3(centre.x, centre.y, top + COLLAR.length);
  for (const o of part.outputs) {
    const at = new THREE.Vector3().setFromMatrixPosition(o.m);
    const z = new THREE.Vector3().setFromMatrixColumn(o.m, 2).normalize();
    // The sphere itself is joined on with the connectors; otherwise stop behind the backing plate.
    const end = o.sphere ? at : at.clone().addScaledVector(z, -COLLAR.length);
    // Rise along the rod direction, bend at 90° (the point on that line nearest the end), run to the end.
    const bend = start.clone().addScaledVector(o.dir, Math.max(0, end.clone().sub(start).dot(o.dir)));
    const rises = bend.distanceTo(start) > 0.5;
    const runs = end.distanceTo(bend) > 0.5;
    const basis = rises
      ? tubeBasis(o.dir, runs ? end.clone().sub(bend).normalize() : null)
      : tubeBasis(end.clone().sub(start).normalize());
    // The end of the tube at a point: a ball (round) or a cube squared to the tube (square).
    const tip = (c) => (square ? squareCube(c, basis, R) : ball(c, R));
    const run = (a, b) => (square ? squareBar(a, b, basis, R) : capsule(a, b, R));

    // Input collar: a cone from the whole back face (sunk into the connector's solid top) to the tube.
    const startTip = tip(start);
    pieces.push(Manifold.hull([...circle(faceRadius, -COLLAR.sink, atInput), ...circle(faceRadius, PLATE.thickness, atInput), startTip]));
    free(startTip);
    if (!o.sphere) {
      // Output collar: the backing plate, a cone from it back to the tube, and a short plug into
      // the spigot's solid base so the plate is fused to it, not just touching.
      const endTip = tip(end);
      pieces.push(Manifold.hull([...circle(PLATE.radius, 0, o.m), ...circle(PLATE.radius, -PLATE.thickness, o.m), endTip]));
      pieces.push(Manifold.hull([...circle(COLLAR.fuseRadius, -1, o.m), ...circle(COLLAR.fuseRadius, COLLAR.sink, o.m)]));
      free(endTip);
    }
    // Constant size all the way, into the sphere too. Square tubes get a cube at each corner.
    if (rises) pieces.push(run(start, bend));
    if (runs) pieces.push(run(bend, end));
    if (square) {
      pieces.push(tip(start));
      if (rises && runs) pieces.push(tip(bend));
      // Into a sphere the bar alone is enough (its corners stay inside); a cube's would poke out.
      if (!o.sphere) pieces.push(tip(end));
    }
  }
  let body = Manifold.union(pieces);
  free(...pieces);

  // Keep-outs.
  const cuts = [];
  const parent = node(part.root.parent.nodeId);
  if (parent.type === 'actuator') {
    // The motor before turns about its output axis (this part's z), so cut its whole swept volume:
    // a stack of discs, each as wide as the motor reaches out from the axis at that height.
    const { pts, kin: pk } = actuatorPoints(parent);
    const toOutput = pk.outputs[0].clone().invert();
    const reach = new Map();
    const v = new THREE.Vector3();
    for (const p of pts) {
      v.set(...p).applyMatrix4(toOutput);
      const bin = Math.floor(v.z / SWEEP_BIN);
      reach.set(bin, Math.max(reach.get(bin) ?? 0, Math.hypot(v.x, v.y)));
    }
    for (const [bin, r] of reach) {
      cuts.push(Manifold.cylinder(SWEEP_BIN + 0.02, r + SWEEP_CLEARANCE, r + SWEEP_CLEARANCE, SEGMENTS).translate([0, 0, bin * SWEEP_BIN - 0.01]));
    }
  } else {
    cuts.push(Manifold.cylinder(FACE_CLEARANCE.depth, FACE_CLEARANCE.radius, FACE_CLEARANCE.radius, SEGMENTS).translate([0, 0, -FACE_CLEARANCE.depth]));
  }
  for (const o of part.outputs) {
    if (o.sphere) continue;
    cuts.push(Manifold.cylinder(FACE_CLEARANCE.depth, FACE_CLEARANCE.radius, FACE_CLEARANCE.radius, SEGMENTS).translate([0, 0, COLLAR.sink]).transform(o.m.elements));
    // A motor bolted on never moves relative to the part: cut its envelope where it sits.
    // (Lifted by the collar's sink: below that is the plug inside this part's own spigot.)
    if (o.child?.type === 'actuator') cuts.push(Manifold.hull(actuatorPoints(o.child).pts).translate([0, 0, COLLAR.sink]).transform(o.m.elements));
  }
  if (cuts.length) {
    const cut = Manifold.union(cuts);
    const trimmed = body.subtract(cut);
    free(body, cut, ...cuts);
    body = trimmed;
  }

  // Join the connectors on.
  const parts = [body, toManifold(shells.input).transform(I.elements)];
  for (const o of part.outputs) {
    parts.push(o.sphere ? Manifold.sphere(o.sphere, SEGMENTS).transform(o.m.elements) : toManifold(shells.output).transform(o.m.elements));
  }
  const solid = Manifold.union(parts);
  free(...parts);

  const mesh = solid.getMesh();
  free(solid);
  const tri = new Float32Array(mesh.triVerts.length * 3);
  mesh.triVerts.forEach((v, k) => { tri.set(mesh.vertProperties.subarray(v * mesh.numProp, v * mesh.numProp + 3), k * 3); });
  const geometry = new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(tri, 3));
  geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  return { geometry, triangles: tri };
}

// ---- Hooks for the viewer and inspector ----

/** Tube cross-section for every part: 'round' or 'square'. */
hooks.tubeShape = 'round';
hooks.setTubeShape = (shape) => {
  hooks.tubeShape = shape;
  hooks.redraw?.();
  emit('view');
};

/**
 * The solid for a link in the CAD tab (in its input frame), EMPTY for a link drawn as part of the
 * link before it, or null while the part is being built (the viewer redraws when it is ready).
 */
hooks.linkFill = (n) => {
  if (!isLink(n) || !cadShellGeometry()) return null;
  const root = partRoot(n);
  const part = describePart(root);
  const sig = signature(part);
  const hit = cache.get(sig);
  if (hit && typeof hit === 'object') return root === n ? hit.geometry : EMPTY;
  if (!hit) {
    cache.set(sig, 'pending');
    queue = queue.then(loadManifold).then(() => {
      cache.set(sig, build(part));
      hooks.redraw?.();
    }).catch((e) => {
      console.warn('Link fill failed:', e);
      cache.set(sig, 'failed');
      // Say why in the CAD legend, once.
      if (!hooks.linkFillError) { hooks.linkFillError = e?.message || String(e); emit('view'); }
    });
  }
  return null;
};

/** Names of the links printed together with this one (just itself when it is alone). */
hooks.linkPart = (n) => (isLink(n) ? describePart(partRoot(n)).links.map((x) => x.name) : []);

/** Is this link the first of its part (the one that draws and exports the whole part)? */
hooks.isPartRoot = (n) => isLink(n) && partRoot(n) === n;

/** Triangles (9 floats each, mm, Z-up) of the part a link belongs to, in that part's input frame. */
hooks.linkFillTriangles = async (n) => {
  if (!cadShellGeometry()) throw new Error('The connector files have not loaded.');
  await loadManifold();
  const part = describePart(partRoot(n));
  const sig = signature(part);
  let hit = cache.get(sig);
  if (!hit || typeof hit !== 'object') { hit = build(part); cache.set(sig, hit); }
  return hit.triangles;
};

/** Binary STL of the part a link belongs to (mm), turned back to the CAD files' up axis. */
hooks.linkFillSTL = async (n) => {
  const hit = { triangles: await hooks.linkFillTriangles(n) };
  const back = CAD_UP === 'y' ? new THREE.Matrix4().makeRotationX(-Math.PI / 2) : new THREE.Matrix4();
  const t = hit.triangles, count = t.length / 9;
  const buf = new ArrayBuffer(84 + count * 50), dv = new DataView(buf);
  dv.setUint32(80, count, true);
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3(), nrm = new THREE.Vector3();
  for (let k = 0; k < count; k++) {
    a.fromArray(t, k * 9).applyMatrix4(back);
    b.fromArray(t, k * 9 + 3).applyMatrix4(back);
    c.fromArray(t, k * 9 + 6).applyMatrix4(back);
    nrm.subVectors(b, a).cross(new THREE.Vector3().subVectors(c, a)).normalize();
    let o = 84 + k * 50;
    for (const v of [nrm, a, b, c]) { dv.setFloat32(o, v.x, true); dv.setFloat32(o + 4, v.y, true); dv.setFloat32(o + 8, v.z, true); o += 12; }
  }
  return new Blob([buf], { type: 'model/stl' });
};
