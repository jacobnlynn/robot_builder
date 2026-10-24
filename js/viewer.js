import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { loadStepMesh } from './stepmesh.js';
import { FAMILY_COLORS, OUTPUT_FLANGE, LINK, MOTORS, GEARBOXES } from './constants.js';
import { isCapped } from './robot.js';

// Robotics convention: Z is up. Must be set before any camera or light is created.
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const COLORS = {
  background: 0xeef1f4,
  select: 0x2f6fe4,
  seam: 0x353d47,
  simpleLine: 0x28313b,
  simpleJoint: 0x6a7889,
};
const DEFAULT_COLORS = { ...COLORS }; // a theme's scene colours replace these (see styles.js)
const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
const GROW_MS = reducedMotion ? 1 : 650;
const ORIGIN = new THREE.Vector3();
const Y_AXIS = new THREE.Vector3(0, 1, 0);

// ---- Geometry helpers (all in a part's local input frame) ----

/** Cylinder along +z from z0 to z0 + length. */
function cylinderZ(radius, length, z0 = 0, segments = 48) {
  const g = new THREE.CylinderGeometry(radius, radius, length, segments);
  g.rotateX(Math.PI / 2);
  g.translate(0, 0, z0 + length / 2);
  return g;
}

function mesh(geometry, material, matrix) {
  const m = new THREE.Mesh(geometry, material);
  if (matrix) { m.matrixAutoUpdate = false; m.matrix.copy(matrix); }
  m.castShadow = true;
  m.receiveShadow = true;
  return m;
}

/** Cylinder between two points, or null when they coincide. */
function rod(p, q, radius, material) {
  const dir = new THREE.Vector3().subVectors(q, p);
  const len = dir.length();
  if (len < 1e-6) return null;
  const m = mesh(new THREE.CylinderGeometry(radius, radius, len, 28), material);
  m.position.copy(p).addScaledVector(dir, 0.5);
  m.quaternion.setFromUnitVectors(Y_AXIS, dir.divideScalar(len));
  return m;
}

function ball(p, radius, material) {
  const m = mesh(new THREE.SphereGeometry(radius, 24, 16), material);
  m.position.copy(p);
  return m;
}

function surface(color, selected) {
  return new THREE.MeshStandardMaterial({
    color,
    metalness: 0.25,
    roughness: 0.55,
    emissive: selected ? COLORS.select : 0x000000,
    emissiveIntensity: selected ? 0.4 : 0,
  });
}

const add = (group, obj) => { if (obj) group.add(obj); };

// ---- Part builders ----

/** CAD-mode geometry. Actuators are placeholders for the real GLB models. */
export function buildCadPart(node, kin, selected = false) {
  const g = new THREE.Group();
  if (node.type === 'actuator') {
    // Input boss: from the mounting face up to the horizontal motor body.
    const motor = kin.segments[0].def;
    g.add(mesh(cylinderZ(motor.radius * 0.8, motor.rise), surface(FAMILY_COLORS.motor, selected)));
    for (const s of kin.segments) {
      const len = s.length, r = s.def.radius;
      g.add(mesh(cylinderZ(r, len), surface(FAMILY_COLORS[s.family], selected), s.start));
      // Dark seam where one module ends or bolts onto the next.
      g.add(mesh(cylinderZ(r + 0.5, 1.2, len - 1.2), surface(COLORS.seam, selected), s.start));
    }
    // Output flange rotates with q; the tab on its rim makes the angle visible.
    const r = Math.max(10, kin.segments.at(-1).def.radius - OUTPUT_FLANGE.inset);
    const t = OUTPUT_FLANGE.thickness;
    const flange = new THREE.Group();
    flange.matrixAutoUpdate = false;
    flange.matrix.copy(kin.outputs[0]);
    flange.userData.output = 0; // re-posed with the joint angle in simulate mode
    flange.add(mesh(cylinderZ(r, t), surface(OUTPUT_FLANGE.color, selected)));
    flange.add(mesh(new THREE.BoxGeometry(5, 7, t).translate(r + 1.5, 0, t / 2), surface(COLORS.seam, selected)));
    g.add(flange);
  } else if (node.type === 'link') {
    const mat = surface(FAMILY_COLORS.link, selected);
    const t = LINK.flangeThickness;
    g.add(mesh(cylinderZ(LINK.flangeRadius * 0.8, t + 2), mat)); // boss on the parent's flange
    g.add(ball(ORIGIN, LINK.radius, mat));
    node.outputs.forEach((_, i) => {
      const elbow = kin.elbows[i];
      const end = new THREE.Vector3().setFromMatrixPosition(kin.outputs[i]);
      add(g, rod(ORIGIN, elbow, LINK.radius, mat));
      add(g, rod(elbow, end, LINK.radius, mat));
      g.add(ball(elbow, LINK.radius, mat));
      if (isCapped(node, i)) g.add(ball(end, node.outputs[i].sphere, mat));
      else {
        g.add(ball(end, LINK.radius, mat));
        g.add(mesh(cylinderZ(LINK.flangeRadius, t, -t), mat, kin.outputs[i]));
      }
    });
  }
  return g;
}

// ---- CAD tab: the designed modules, from their STEP files ----

/**
 * One STEP file per module, read on demand and shared by every part that uses it:
 *  - motor:<family>: the full motor with its built-in first gearbox of that type. Origin at the
 *    centre of the mounting (input) face, input axis up, output shaft pointing along +x.
 *  - extension:<family>: one add-on stage of that type. Origin at the centre of its input face,
 *    shaft pointing up.
 *  - input / output: the connectors a link starts and ends with. Origin at the centre of the
 *    mating face, pointing up (away from the part it mates with for the input, outward for the output).
 * Fusion 360 exports Y-up, so with CAD_UP = 'y' each file is turned +90° about x (its +y becomes +z).
 * Until a file has loaded, a placeholder is drawn. Every module is drawn in its family colour.
 */
export const CAD_FILES = {
  motor: (family) => `models/${family}_gearbox.step`,
  extension: (family) => `models/${family}_extension.step`,
  input: 'models/input.step',
  output: 'models/output.step',
};
export const CAD_UP = 'y'; // 'y' for Fusion 360's default; 'z' for files already exported Z-up
// The motor file's built-in gearbox: everything in front of this plane (mm along the output axis,
// from the input axis) and above the input boss.
const BUILT_IN_GEARBOX_START = -2.94;
const HOUSING_RADIUS = 35.7; // about the output axis; anything reaching lower belongs to the input boss

const cad = new Map(); // key → { status: 'loading' | 'loaded' | 'missing', geometry, promise }
let onCadChange = () => {};
/** Called whenever a module file finishes loading (or fails), so the page can redraw. */
export function setCadListener(fn) { onCadChange = fn; }

const urlFor = (key) => {
  const [kind, family] = key.split(':');
  return typeof CAD_FILES[kind] === 'function' ? CAD_FILES[kind](family) : CAD_FILES[kind];
};

/** Turn a file-frame geometry Z-up and give it flat normals. */
function finish(geometry) {
  if (CAD_UP === 'y') geometry.applyMatrix4(new THREE.Matrix4().makeRotationX(Math.PI / 2));
  geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  return geometry;
}

/** Split triangles: inside(centre x, lowest z) → first geometry, else second. */
function splitTriangles(geometry, inside) {
  const p = geometry.attributes.position.array;
  const a = [], b = [];
  for (let i = 0; i < p.length; i += 9) {
    const x = (p[i] + p[i + 3] + p[i + 6]) / 3, zMin = Math.min(p[i + 2], p[i + 5], p[i + 8]);
    const into = inside(x, zMin) ? a : b;
    for (let k = 0; k < 9; k++) into.push(p[i + k]);
  }
  return [a, b].map((arr) => {
    const g = new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(arr, 3));
    g.computeVertexNormals();
    g.computeBoundingSphere();
    return g;
  });
}

/** Start loading a module file (once). Resolves to its geometry, or null if it could not be read. */
export function loadCad(key) {
  let entry = cad.get(key);
  if (entry) return entry.promise;
  entry = { status: 'loading', geometry: null };
  entry.promise = loadStepMesh(urlFor(key)).then((raw) => {
    const g = finish(raw);
    if (key.startsWith('motor:')) {
      // Colour the built-in gearbox like its type: split it off the motor by position.
      const rise = Object.values(MOTORS)[0].rise;
      const [firstGearbox, motor] = splitTriangles(g, (x, zMin) => x > BUILT_IN_GEARBOX_START && zMin > rise - HOUSING_RADIUS);
      entry.geometry = { motor, firstGearbox, whole: g };
    } else {
      entry.geometry = g;
    }
    entry.status = 'loaded';
    onCadChange();
    return entry.geometry;
  }, (e) => {
    console.warn(e);
    entry.status = 'missing';
    onCadChange();
    return null;
  });
  cad.set(key, entry);
  return entry.promise;
}

/** A module's geometry if it has loaded, else null (and it starts loading; the page redraws when ready). */
export function cadGeometry(key) {
  const entry = cad.get(key);
  if (!entry) { loadCad(key); return null; }
  return entry.geometry;
}

/** The files an actuator needs: its motor (by first gearbox) and one extension per extra stage. */
export function actuatorCadKeys(n) {
  const [first, ...extra] = n.gearboxes.map((k) => GEARBOXES[k].family);
  return [`motor:${first}`, ...extra.map((f) => `extension:${f}`)];
}

/** Wait for these files; resolves to true if all of them loaded. */
export async function whenCadReady(keys) {
  const all = await Promise.all(keys.map(loadCad));
  return all.every(Boolean);
}

/** For the legend: how many files are still loading, and which could not be read. */
export function cadModelStatus() {
  const list = [...cad.entries()];
  return { loading: list.filter(([, e]) => e.status === 'loading').length, missing: list.filter(([, e]) => e.status === 'missing').map(([k]) => k) };
}

/**
 * Optional add-ons, set by modules that main.js imports (each can be removed with its import line):
 *  - linkFill(node, kin): a link's filled solid for the CAD tab, or null (it may arrive later via redraw)
 *  - linkFillError: why the last link fill failed, if one did
 *  - tubeShape / setTubeShape: 'round' or 'square' tubes for the filled links
 * redraw is filled in by createViewer.
 */
export const hooks = { linkFill: null, linkFillError: null, redraw: null };

// One material per colour, shared by every shell that isn't selected.
const shellMaterials = new Map();
function shellMaterial(color, selected) {
  if (selected) return surface(color, true);
  if (!shellMaterials.has(color)) shellMaterials.set(color, surface(color, false));
  return shellMaterials.get(color);
}

/** One shell piece placed at `matrix`, sharing the loaded geometry. */
function shellMesh(geometry, color, matrix, selected) {
  const m = mesh(geometry, shellMaterial(color, selected), matrix);
  m.userData.shared = true; // geometry, and the material unless selected, belong to the shared cache
  m.userData.ownMaterial = selected;
  return m;
}

/**
 * CAD tab: every part as its designed module, each in its family colour.
 * A link is its filled part when that is ready; until then its input connector where it attaches
 * and, on each output, an output connector or the sphere it ends in. The base link isn't drawn.
 */
function buildShellPart(node, kin, selected = false) {
  if (node.type === 'link') {
    const g = new THREE.Group();
    if (!node.parent) return g; // the base is the world anchor; nothing to draw
    const filled = hooks.linkFill?.(node, kin);
    if (filled) { g.add(shellMesh(filled, FAMILY_COLORS.link, new THREE.Matrix4(), selected)); return g; }
    const input = cadGeometry('input'), output = cadGeometry('output');
    if (input) g.add(shellMesh(input, FAMILY_COLORS.link, new THREE.Matrix4(), selected));
    kin.outputs.forEach((m, i) => {
      if (isCapped(node, i)) g.add(ball(new THREE.Vector3().setFromMatrixPosition(m), node.outputs[i].sphere, surface(FAMILY_COLORS.link, selected)));
      else if (output) g.add(shellMesh(output, FAMILY_COLORS.link, m, selected));
    });
    return g;
  }
  if (node.type !== 'actuator') return new THREE.Group();
  const [, first, ...extra] = kin.segments;
  const motor = cadGeometry(`motor:${first.family}`);
  if (!motor) return buildCadPart(node, kin, selected); // placeholder until the motor file loads
  const g = new THREE.Group();
  g.add(shellMesh(motor.motor, FAMILY_COLORS.motor, kin.mount, selected));
  g.add(shellMesh(motor.firstGearbox, FAMILY_COLORS[first.family], kin.mount, selected));
  for (const s of extra) {
    const stage = cadGeometry(`extension:${s.family}`);
    g.add(stage ? shellMesh(stage, FAMILY_COLORS[s.family], s.start, selected)
      : mesh(cylinderZ(s.def.radius, s.length), surface(FAMILY_COLORS[s.family], selected), s.start));
  }
  // The output flange turns with the joint, so it stays a separate part.
  const r = Math.max(10, kin.segments.at(-1).def.radius - OUTPUT_FLANGE.inset);
  const t = OUTPUT_FLANGE.thickness;
  const flange = new THREE.Group();
  flange.matrixAutoUpdate = false;
  flange.matrix.copy(kin.outputs[0]);
  flange.userData.output = 0;
  flange.add(mesh(cylinderZ(r, t), surface(OUTPUT_FLANGE.color, selected)));
  flange.add(mesh(new THREE.BoxGeometry(5, 7, t).translate(r + 1.5, 0, t / 2), surface(COLORS.seam, selected)));
  g.add(flange);
  return g;
}

const BUILDERS = { cad: buildCadPart, sim: buildCadPart, cadfile: buildShellPart };
const builderFor = (view) => BUILDERS[view] ?? buildSimplePart;

/** Simplified mode: cylinders for joints, thin rods for links. */
function buildSimplePart(node, kin, selected = false) {
  const g = new THREE.Group();
  const line = surface(selected ? COLORS.select : COLORS.simpleLine, false);
  if (node.type === 'actuator') {
    const j = new THREE.Vector3().setFromMatrixPosition(kin.joint);
    add(g, rod(ORIGIN, kin.elbow, 2.5, line));
    add(g, rod(kin.elbow, j, 2.5, line));
    g.add(ball(kin.elbow, 2.5, line));
    g.add(mesh(cylinderZ(15, 42, -21), surface(selected ? COLORS.select : COLORS.simpleJoint, false), kin.joint));
  } else if (node.type === 'link') {
    node.outputs.forEach((_, i) => {
      const elbow = kin.elbows[i];
      const end = new THREE.Vector3().setFromMatrixPosition(kin.outputs[i]);
      add(g, rod(ORIGIN, elbow, 2.5, line));
      add(g, rod(elbow, end, 2.5, line));
      g.add(ball(elbow, 2.5, line));
      if (isCapped(node, i)) g.add(ball(end, node.outputs[i].sphere, surface(selected ? COLORS.select : COLORS.simpleJoint, false)));
    });
  }
  return g;
}

/** Coordinate triads on every output, and a clickable marker on each open port. */
function addFramesAndPorts(root, id, node, kin, state, pickables) {
  kin.outputs.forEach((m, i) => {
    if (state.showFrames) {
      const axes = new THREE.AxesHelper(32);
      axes.matrixAutoUpdate = false;
      axes.matrix.copy(m);
      axes.userData.output = i;
      axes.material.depthTest = false;
      axes.material.transparent = true;
      axes.renderOrder = 2;
      root.add(axes);
    }
    if (node.children[i] || isCapped(node, i) || state.view === 'sim') return;
    const sel = state.selection;
    const active = sel?.kind === 'port' && sel.nodeId === id && sel.index === i;
    const marker = new THREE.Group();
    marker.matrixAutoUpdate = false;
    marker.matrix.copy(m);
    const mat = new THREE.MeshBasicMaterial({ color: COLORS.select, transparent: true, opacity: active ? 1 : 0.55, depthTest: false });
    const dot = new THREE.Mesh(new THREE.SphereGeometry(active ? 8 : 6.5, 20, 14), mat);
    dot.renderOrder = 3;
    dot.userData.pick = { kind: 'port', nodeId: id, index: i };
    marker.add(dot);
    pickables.push(dot);
    if (active) {
      const ring = new THREE.Mesh(new THREE.TorusGeometry(15, 1.6, 10, 48), mat);
      ring.renderOrder = 3;
      marker.add(ring);
    }
    root.add(marker);
  });
}

function isShown(obj) {
  for (let o = obj; o; o = o.parent) if (!o.visible) return false;
  return true;
}

// ---- Viewer ----

export function createViewer(container, { onPick }) {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.prepend(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(COLORS.background);

  const camera = new THREE.PerspectiveCamera(38, 1, 1, 20000);
  camera.position.set(700, -800, 560);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 180);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;

  scene.add(new THREE.HemisphereLight(0xffffff, 0xc9d0d8, 1.5));
  const sun = new THREE.DirectionalLight(0xffffff, 1.8);
  sun.position.set(400, -500, 1100);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -900, right: 900, top: 900, bottom: -900, near: 10, far: 3000 });
  sun.shadow.camera.updateProjectionMatrix();
  sun.shadow.bias = -0.0005;
  scene.add(sun);
  const fill = new THREE.DirectionalLight(0xffffff, 0.6);
  fill.position.set(-600, 400, 300);
  scene.add(fill);


  const ray = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  const robot = new THREE.Group();
  scene.add(robot);
  const parts = new Map();
  const pickables = [];
  let current = { state: null, frames: null };

  function clear() {
    robot.traverse((o) => {
      // Shell copies share the loaded model's geometry and materials; keep those.
      if (!o.userData.shared) o.geometry?.dispose();
      if (o.material && (!o.userData.shared || o.userData.ownMaterial)) [].concat(o.material).forEach((m) => m.dispose());
    });
    robot.clear();
    parts.clear();
    pickables.length = 0;
  }

  /** Translucent, unclickable preview of the part being set up at the selected port. */
  function addGhost(state, { node, local, input }) {
    const build = builderFor(state.view);
    const root = new THREE.Group();
    root.matrixAutoUpdate = false;
    root.matrix.copy(input);
    root.add(build(node, local, false));
    if (state.showFrames) {
      for (const m of local.outputs) {
        const axes = new THREE.AxesHelper(32);
        axes.matrixAutoUpdate = false;
        axes.matrix.copy(m);
        root.add(axes);
      }
    }
    root.traverse((o) => {
      if (!o.material) return;
      if (o.userData.shared && !o.userData.ownMaterial) {
        const mats = [].concat(o.material).map((m) => m.clone());
        o.material = mats.length === 1 ? mats[0] : mats;
        o.userData.ownMaterial = true;
      }
      o.castShadow = false;
      for (const m of [].concat(o.material)) {
        m.transparent = true;
        m.opacity = o.isMesh ? 0.45 : 0.6;
        m.depthWrite = !o.isMesh;
      }
    });
    robot.add(root);
  }

  /** Rebuild the scene from the robot tree. Cheap enough to run on every edit. */
  function update(state, frames, preview = null) {
    current = { state, frames, preview };
    clear();
    // Simulate mode shows the Model view.
    const build = builderFor(state.view);
    for (const [id, f] of Object.entries(frames)) {
      const node = state.robot.nodes[id];
      const selected = state.view !== 'sim' && state.selection?.kind === 'node' && state.selection.nodeId === id;
      const part = build(node, f.local, selected);
      part.traverse((o) => {
        if (!o.isMesh) return;
        // onOutput: the mesh turns with this part's own joint (an actuator's output flange).
        let onOutput = false;
        for (let a = o; a && a !== part; a = a.parent) if (a.userData.output != null) onOutput = true;
        o.userData.pick = { kind: 'node', nodeId: id, onOutput };
        pickables.push(o);
      });
      const root = new THREE.Group();
      root.matrixAutoUpdate = false;
      root.userData.base = f.input;
      root.add(part);
      addFramesAndPorts(root, id, node, f.local, state, pickables);
      robot.add(root);
      parts.set(id, root);
    }
    if (preview) addGhost(state, preview);
    grow(performance.now());
  }

  /** New parts scale up out of the port they attach to. */
  function grow(now) {
    if (!current.state) return;
    for (const [id, root] of parts) {
      const t = (now - (current.state.born[id] ?? -Infinity)) / GROW_MS;
      root.visible = t > 0;
      root.matrix.copy(root.userData.base);
      if (t < 1) {
        const s = Math.max(0.001, 1 - Math.pow(1 - Math.max(t, 0), 3));
        root.matrix.multiply(new THREE.Matrix4().makeScale(s, s, s));
      }
      root.matrixWorldNeedsUpdate = true;
    }
  }

  /**
   * Move the existing parts to new joint angles without rebuilding them (simulate mode).
   * Only valid while the robot's structure and part shapes are unchanged since the last update().
   */
  function pose(frames) {
    current.frames = frames;
    for (const [id, root] of parts) {
      const f = frames[id];
      if (!f) continue;
      root.userData.base = f.input;
      root.traverse((o) => {
        if (o.userData.output == null) return;
        o.matrix.copy(f.local.outputs[o.userData.output]);
        o.matrixWorldNeedsUpdate = true;
      });
    }
  }

  /** Part under the pointer: { pick, point } with the hit point in world mm, or null. */
  function pickAt(clientX, clientY) {
    rayAt(clientX, clientY);
    const hit = ray.intersectObjects(pickables, false).find((h) => isShown(h.object) && h.object.userData.pick.kind === 'node');
    return hit ? { pick: hit.object.userData.pick, point: hit.point.clone() } : null;
  }

  function rayAt(clientX, clientY) {
    const rect = renderer.domElement.getBoundingClientRect();
    ndc.set(((clientX - rect.left) / rect.width) * 2 - 1, -((clientY - rect.top) / rect.height) * 2 + 1);
    ray.setFromCamera(ndc, camera);
    return ray;
  }

  // Grab marker for simulate mode: a dot on the grabbed point and a line to where it is being pulled.
  const dragMat = new THREE.MeshBasicMaterial({ color: COLORS.select, depthTest: false, transparent: true });
  const dragDot = new THREE.Mesh(new THREE.SphereGeometry(5, 16, 12), dragMat);
  const dragTarget = new THREE.Mesh(new THREE.SphereGeometry(3.5, 16, 12), dragMat);
  const dragLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([ORIGIN, ORIGIN]),
    new THREE.LineBasicMaterial({ color: COLORS.select, depthTest: false, transparent: true }));
  const dragMarker = new THREE.Group();
  dragMarker.add(dragDot, dragTarget, dragLine);
  dragMarker.traverse((o) => { o.renderOrder = 4; });
  dragMarker.visible = false;
  scene.add(dragMarker);
  function setDragMarker(point, target) {
    dragMarker.visible = !!point;
    if (!point) return;
    dragDot.position.copy(point);
    dragTarget.position.copy(target);
    dragLine.geometry.setFromPoints([point, target]);
  }

  const frameHooks = new Set();

  // Workspace shapes (simulate mode), in world frame. They outlive update(), which only rebuilds the robot.
  const workspaces = new Map();
  function setWorkspace(key, obj) {
    if (workspaces.get(key) === obj) return;
    if (workspaces.has(key)) scene.remove(workspaces.get(key));
    workspaces.delete(key);
    if (obj) { scene.add(obj); workspaces.set(key, obj); }
  }

  function fit() {
    if (!current.frames) return;
    const box = new THREE.Box3();
    const p = new THREE.Vector3();
    for (const f of Object.values(current.frames)) {
      box.expandByPoint(p.setFromMatrixPosition(f.input));
      for (const m of f.outputs) box.expandByPoint(p.setFromMatrixPosition(m));
    }
    box.expandByScalar(80);
    const center = box.getCenter(new THREE.Vector3());
    const radius = box.getSize(new THREE.Vector3()).length() / 2;
    const fov = Math.min(camera.fov, camera.fov * camera.aspect) * Math.PI / 180;
    const dist = Math.max(350, 0.8 * radius / Math.sin(fov / 2));
    const dir = new THREE.Vector3(0.62, -0.68, 0.42).normalize();
    controls.target.copy(center);
    camera.position.copy(center).addScaledVector(dir, dist);
    controls.update();
  }

  // Click (not drag) to select a part or an open port.
  let down = null;
  renderer.domElement.addEventListener('pointerdown', (e) => { down = { x: e.clientX, y: e.clientY }; });
  renderer.domElement.addEventListener('pointerup', (e) => {
    if (!down || Math.hypot(e.clientX - down.x, e.clientY - down.y) > 5) return;
    down = null;
    if (current.state?.view === 'sim') return; // nothing is selected while simulating
    rayAt(e.clientX, e.clientY);
    const hits = ray.intersectObjects(pickables, false).filter((h) => isShown(h.object));
    const hit = hits.find((h) => h.object.userData.pick.kind === 'port') ?? hits[0];
    onPick(hit ? hit.object.userData.pick : null);
  });

  function resize() {
    const w = container.clientWidth, h = container.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(container);
  resize();

  let last = performance.now();
  renderer.setAnimationLoop(() => {
    const now = performance.now();
    const dt = Math.min(0.05, (now - last) / 1000); // seconds; capped so a stalled tab doesn't jump
    last = now;
    for (const fn of frameHooks) fn(dt);
    grow(now);
    controls.update();
    renderer.render(scene, camera);
  });

  /** Redraw with the same robot, e.g. once a shell file has loaded. */
  const redraw = () => { if (current.state) update(current.state, current.frames, current.preview); };
  hooks.redraw = redraw;

  return {
    redraw, update, fit, pose, pickAt, rayAt, setDragMarker, setWorkspace, camera, controls,
    /** Scene colours for a theme (background, select, seam, simpleLine, simpleJoint); {} restores the defaults. */
    setSceneColors(c) {
      Object.assign(COLORS, DEFAULT_COLORS, c);
      scene.background.set(COLORS.background);
    },
    canvas: renderer.domElement,
    onFrame: (fn) => frameHooks.add(fn),
  };
}
