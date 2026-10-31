// Trial: filled links. Each printed part becomes one solid: an input connector, a straight pipe
// with a 90° bend to each output (like the Model view's rods), and an output connector (or the end
// sphere) on each output. Pipes are round or square tubes (hooks.tubeShape, toggled in the CAD tab);
// square ones have both runs squared to the plane of their bend, with a cube filling the elbow.
//  - Links attached straight to each other (no motor between) are rigid together, so they print
//    as one part: one input connector, the outputs of the last link, and pipes routed from the
//    first input to those outputs. The first link of the run draws and exports the whole part.
//  - A passive joint (a ball or hinge output) ends a part: each side of it gets a small ball where
//    the joint goes (no CAD model for the joints yet), and the next link starts a new part there.
//  - Each pipe end is fused on with a tapered collar: a cone from the full connector face to the
//    pipe, sunk into the connector's solid wall, so it is physically joined rather than touching.
//  - Then the volume the motor before the part sweeps as it turns is cut away, as is the envelope of
//    each motor bolted to an output, and the space in front of every mating face is kept clear.
// The part is first described as a plan (partPlan: pieces, cuts, connectors). This file builds it
// as a mesh for the CAD tab with manifold-3d (WASM, served from vendor/ because the preview host
// blocks loading WebAssembly from a CDN); stepexport.js builds the same plan as a STEP solid.
// To remove the trial, delete the `import './linkfill.js'` line in main.js.
import * as THREE from 'three';
import { state, node, isCapped, isPassive, emit } from './robot.js';
import { localKinematics, forwardKinematics, rodDirection } from './kinematics.js';
import { hooks, cadGeometry, actuatorCadKeys, whenCadReady } from './viewer.js';
import { LINK, PASSIVE } from './constants.js';

const MANIFOLD_URL = new URL('../vendor/manifold-3d-2.5.1/manifold.js', import.meta.url).href;
export const PIPE = { radius: LINK.radius }; // same as the Model view's rods
export const PLATE = { radius: 35, thickness: 3 }; // backing plate behind each output, matching the next part's face
export const COLLAR = {
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

/** Vertex positions of a loaded geometry as [x, y, z] points, optionally moved by `m`. */
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

/** Three perpendicular unit vectors for a tube that runs along d1, then (optionally) d2 ⊥ d1. */
function tubeBasis(d1, d2) {
  let n = d2 ? new THREE.Vector3().crossVectors(d1, d2) : null;
  if (!n || n.lengthSq() < 1e-6) n = new THREE.Vector3().crossVectors(d1, Math.abs(d1.z) < 0.9 ? new THREE.Vector3(0, 0, 1) : new THREE.Vector3(1, 0, 0));
  n.normalize();
  return [d1.clone().normalize(), new THREE.Vector3().crossVectors(n, d1).normalize(), n];
}

/** Every point of an actuator's modules (motor, built-in gearbox, extra stages) in its input frame. */
function actuatorPoints(n) {
  const kin = localKinematics(n);
  const [, ...extra] = kin.segments;
  const [motorKey, ...stageKeys] = actuatorCadKeys(n);
  const pts = points(cadGeometry(motorKey).whole, kin.mount);
  extra.slice(1).forEach((s, i) => pts.push(...points(cadGeometry(stageKeys[i]), s.start)));
  return { pts, kin };
}

// ---- Parts: runs of links with no motor between them ----

const isLink = (n) => n?.type === 'link' && n.parent; // the base is the world anchor, not a part

/** True when link n is fixed straight onto the link before it (not through a passive joint). */
const rigidOnLink = (n) => isLink(node(n.parent.nodeId)) && !isPassive(node(n.parent.nodeId), n.parent.index);

/** The first link of the run of links this one belongs to. */
function partRoot(n) {
  while (rigidOnLink(n)) n = node(n.parent.nodeId);
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
      if (isLink(child) && !isPassive(x, i)) return visit(child, dir);
      // A passive joint ends the part in a small ball where the joint goes.
      const sphere = isCapped(x, i) ? o.sphere : isPassive(x, i) ? PASSIVE.ballRadius : 0;
      const m = toRoot.clone().multiply(frames[x.id].outputs[i]);
      // Only where a joint's ball sits matters, not how far the joint is turned (so moving it doesn't rebuild the part).
      if (isPassive(x, i)) m.makeTranslation(new THREE.Vector3().setFromMatrixPosition(m));
      outputs.push({ m, sphere, child, dir });
    });
  };
  visit(root, null);
  const p = root.parent;
  return { root, links, outputs, shape: hooks.tubeShape, passiveIn: isPassive(node(p.nodeId), p.index) };
}

/** Module files a part's solid needs: the connectors and the motors on either side of it. */
function partCadKeys(part) {
  const keys = new Set(['input', 'output']);
  const parent = node(part.root.parent.nodeId);
  if (parent.type === 'actuator') actuatorCadKeys(parent).forEach((k) => keys.add(k));
  for (const o of part.outputs) if (o.child?.type === 'actuator') actuatorCadKeys(o.child).forEach((k) => keys.add(k));
  return [...keys];
}

/** What a part's solid depends on: its outputs, and the motors on either side of it. */
function signature(part) {
  const parent = node(part.root.parent.nodeId);
  const r3 = (v) => Math.round(v * 1000) / 1000;
  return JSON.stringify({
    shape: part.shape,
    passiveIn: part.passiveIn,
    outputs: part.outputs.map((o) => [o.m.elements.map(r3), o.sphere, o.dir.toArray().map(r3),
      o.child?.type === 'actuator' ? [o.child.motor, o.child.gearboxes, o.child.mount] : null]),
    parent: parent.type === 'actuator' ? [parent.motor, parent.gearboxes] : null,
  });
}

// ---- The plan: what the solid is made of, independent of any geometry library ----

/**
 * Pieces (joined together), cuts (taken away from them) and connectors (joined on last), all in
 * the part's input frame (mm, Z-up):
 *   capsule {a, b, r}, bar {a, b, basis, r}, cube {c, basis, r}, ball {c, r},
 *   disc {frame, r, z0, z1} (a cylinder along frame z), collar {frame, r, z0, z1, tip} (that disc
 *   tapering to a tip: a ball or a cube), hull {pts, frame} (convex hull of points), and the
 *   connectors input {m}, output {m}, sphere {c, r}.
 */
export function partPlan(part) {
  const R = PIPE.radius;
  const input = cadGeometry('input');
  if (!input.boundingBox) input.computeBoundingBox();
  const top = input.boundingBox.max.z;
  // Centre and size of the input's back face (its top).
  const back = points(input).filter((p) => p[2] > top - 0.3);
  const backBox = new THREE.Box3().setFromArray(back.flat());
  const faceRadius = Math.min(backBox.max.x - backBox.min.x, backBox.max.y - backBox.min.y) / 2;
  const centre = backBox.getCenter(new THREE.Vector3());
  const atInput = new THREE.Matrix4().makeTranslation(centre.x, centre.y, top);

  const square = part.shape === 'square';
  // On a passive joint the part starts at a small ball on its input instead of a connector.
  const pieces = [], cuts = [], connectors = [part.passiveIn ? { t: 'sphere', c: new THREE.Vector3(), r: PASSIVE.ballRadius } : { t: 'input', m: new THREE.Matrix4() }];
  const start = part.passiveIn ? new THREE.Vector3() : new THREE.Vector3(centre.x, centre.y, top + COLLAR.length);
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
    const tip = (c) => (square ? { t: 'cube', c, basis, r: R } : { t: 'ball', c, r: R });
    const run = (a, b) => (square ? { t: 'bar', a, b, basis, r: R } : { t: 'capsule', a, b, r: R });

    // Input collar: a cone from the whole back face (sunk into the connector's solid top) to the tube.
    if (!part.passiveIn) pieces.push({ t: 'collar', frame: atInput, r: faceRadius, z0: -COLLAR.sink, z1: PLATE.thickness, tip: tip(start) });
    if (!o.sphere) {
      // Output collar: the backing plate, a cone from it back to the tube, and a short plug into
      // the spigot's solid base so the plate is fused to it, not just touching.
      pieces.push({ t: 'collar', frame: o.m, r: PLATE.radius, z0: -PLATE.thickness, z1: 0, tip: tip(end) });
      pieces.push({ t: 'disc', frame: o.m, r: COLLAR.fuseRadius, z0: -1, z1: COLLAR.sink });
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
    connectors.push(o.sphere ? { t: 'sphere', c: at, r: o.sphere } : { t: 'output', m: o.m });
  }

  // Keep-outs.
  const I = new THREE.Matrix4();
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
      cuts.push({ t: 'disc', frame: I, r: r + SWEEP_CLEARANCE, z0: bin * SWEEP_BIN - 0.01, z1: (bin + 1) * SWEEP_BIN + 0.01 });
    }
  } else if (!part.passiveIn) {
    cuts.push({ t: 'disc', frame: I, r: FACE_CLEARANCE.radius, z0: -FACE_CLEARANCE.depth, z1: 0 });
  }
  for (const o of part.outputs) {
    if (o.sphere) continue;
    cuts.push({ t: 'disc', frame: o.m, r: FACE_CLEARANCE.radius, z0: COLLAR.sink, z1: COLLAR.sink + FACE_CLEARANCE.depth });
    // A motor bolted on never moves relative to the part: cut its envelope where it sits.
    // (Lifted by the collar's sink: below that is the plug inside this part's own spigot.)
    if (o.child?.type === 'actuator') {
      cuts.push({ t: 'hull', pts: actuatorPoints(o.child).pts, frame: o.m.clone().multiply(new THREE.Matrix4().makeTranslation(0, 0, COLLAR.sink)) });
    }
  }
  return { pieces, cuts, connectors };
}

// ---- The plan as a mesh (manifold-3d), for the CAD tab ----

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

const free = (...ms) => ms.forEach((m) => m?.delete());

function cubePoints(c, basis, r) {
  const pts = [];
  for (const a of [-r, r]) for (const b of [-r, r]) for (const d of [-r, r]) {
    pts.push(c.clone().addScaledVector(basis[0], a).addScaledVector(basis[1], b).addScaledVector(basis[2], d).toArray());
  }
  return pts;
}

function toMesh(p) {
  const { Manifold } = wasm;
  const ball = (c, r) => Manifold.sphere(r, SEGMENTS).translate(c.toArray());
  switch (p.t) {
    case 'ball': return ball(p.c, p.r);
    case 'cube': return Manifold.hull(cubePoints(p.c, p.basis, p.r));
    case 'capsule': {
      const A = ball(p.a, p.r), B = ball(p.b, p.r);
      const c = Manifold.hull([A, B]);
      free(A, B);
      return c;
    }
    case 'bar': {
      const along = p.b.clone().sub(p.a).normalize();
      const [u, v] = p.basis.filter((e) => Math.abs(e.dot(along)) < 0.5);
      const pts = [];
      for (const q of [p.a, p.b]) for (const su of [-p.r, p.r]) for (const sv of [-p.r, p.r]) pts.push(q.clone().addScaledVector(u, su).addScaledVector(v, sv).toArray());
      return Manifold.hull(pts);
    }
    case 'disc': return Manifold.cylinder(p.z1 - p.z0, p.r, p.r, SEGMENTS).translate([0, 0, p.z0]).transform(p.frame.elements);
    case 'collar': {
      const tip = toMesh(p.tip);
      const c = Manifold.hull([...circle(p.r, p.z0, p.frame), ...circle(p.r, p.z1, p.frame), tip]);
      free(tip);
      return c;
    }
    case 'hull': return Manifold.hull(p.pts).transform(p.frame.elements);
    case 'input': return toManifold(cadGeometry('input')).transform(p.m.elements);
    case 'output': return toManifold(cadGeometry('output')).transform(p.m.elements);
    case 'sphere': return ball(p.c, p.r);
    default: throw new Error(`Unknown piece ${p.t}`);
  }
}

function build(part) {
  const { Manifold } = wasm;
  const plan = partPlan(part);
  const pieces = plan.pieces.map(toMesh);
  let body = Manifold.union(pieces);
  free(...pieces);
  if (plan.cuts.length) {
    const cuts = plan.cuts.map(toMesh);
    const cut = Manifold.union(cuts);
    const trimmed = body.subtract(cut);
    free(body, cut, ...cuts);
    body = trimmed;
  }
  const parts = [body, ...plan.connectors.map(toMesh)];
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

// ---- Hooks for the viewer, the inspector and the exports ----

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
  if (!isLink(n)) return null;
  const root = partRoot(n);
  const part = describePart(root);
  // Wait for the module files it needs; they redraw the page when they arrive.
  if (!partCadKeys(part).every((k) => cadGeometry(k))) return null;
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

/** The part a link belongs to, once the module files it needs have loaded: { root, links, plan, sig }. */
hooks.linkPartPlan = async (n) => {
  const part = describePart(partRoot(n));
  if (!(await whenCadReady(partCadKeys(part)))) throw new Error('Some module STEP files could not be loaded.');
  return { root: part.root, links: part.links, plan: partPlan(part), sig: signature(part) };
};

/** Triangles (9 floats each, mm, Z-up) of the part a link belongs to, in that part's input frame. */
hooks.linkFillTriangles = async (n) => {
  const part = describePart(partRoot(n));
  if (!(await whenCadReady(partCadKeys(part)))) throw new Error('Some module STEP files could not be loaded.');
  await loadManifold();
  const sig = signature(part);
  let hit = cache.get(sig);
  if (!hit || typeof hit !== 'object') { hit = build(part); cache.set(sig, hit); }
  return hit.triangles;
};
