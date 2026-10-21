import * as THREE from 'three';
import { DH_CONVENTION, MOTORS, GEARBOXES } from './constants.js';

const DEG = Math.PI / 180;

/** D-H parameters → homogeneous transform from one frame to the next. */
export function dhToMatrix({ a = 0, alpha = 0, d = 0, theta = 0 }, convention = DH_CONVENTION) {
  const ct = Math.cos(theta * DEG), st = Math.sin(theta * DEG);
  const ca = Math.cos(alpha * DEG), sa = Math.sin(alpha * DEG);
  const m = new THREE.Matrix4();
  if (convention === 'modified') {
    // Rx(α) · Tx(a) · Rz(θ) · Tz(d)
    m.set(
      ct,      -st,      0,   a,
      st * ca,  ct * ca, -sa, -sa * d,
      st * sa,  ct * sa,  ca,  ca * d,
      0, 0, 0, 1);
  } else {
    // Rz(θ) · Tz(d) · Tx(a) · Rx(α)
    m.set(
      ct, -st * ca,  st * sa, a * ct,
      st,  ct * ca, -ct * sa, a * st,
      0,   sa,       ca,      d,
      0, 0, 0, 1);
  }
  return m;
}

/** Corner of the D-H "L" in the input frame: where one offset ends and the other begins. */
export function dhElbow({ a = 0, d = 0 }, convention = DH_CONVENTION) {
  return convention === 'modified' ? new THREE.Vector3(a, 0, 0) : new THREE.Vector3(0, 0, d);
}

/**
 * Position (mm) and rotations (°) about the reference frame's fixed x, y, z axes,
 * applied α about x first, then β about y, then γ about z: R = Rz(γ) · Ry(β) · Rx(α).
 */
export function poseToMatrix({ x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0 }) {
  return new THREE.Matrix4()
    .makeRotationFromEuler(new THREE.Euler(alpha * DEG, beta * DEG, gamma * DEG, 'ZYX'))
    .setPosition(x, y, z);
}

const tidy = (v) => { const r = Math.round(v * 1000) / 1000; return Object.is(r, -0) ? 0 : r; };

export function matrixToPose(m) {
  const e = new THREE.Euler().setFromRotationMatrix(m, 'ZYX');
  const p = new THREE.Vector3().setFromMatrixPosition(m);
  return { x: tidy(p.x), y: tidy(p.y), z: tidy(p.z), alpha: tidy(e.x / DEG), beta: tidy(e.y / DEG), gamma: tidy(e.z / DEG) };
}

/**
 * D-H parameters that build `m`, or the closest ones when none do. D-H has 4 numbers and a
 * general frame needs 6, so the fit keeps the output's z axis (the next joint's axis) exact and
 * gets the rest as close as it can. Returns { dh, posErr (mm), angErr (°) }; both 0 when exact.
 */
export function matrixToDH(m, convention = DH_CONVENTION) {
  const e = m.elements;
  const X = new THREE.Vector3(e[0], e[1], e[2]);
  const Z = new THREE.Vector3(e[8], e[9], e[10]);
  const p = new THREE.Vector3(e[12], e[13], e[14]);
  const xIn = new THREE.Vector3(1, 0, 0), zIn = new THREE.Vector3(0, 0, 1);
  let dh;
  if (convention === 'modified') {
    // Rx(α) · Tx(a) · Rz(θ) · Tz(d): the output z axis has to lie in the input's y-z plane.
    const z = Z.clone().setX(0);
    if (z.lengthSq() < 1e-12) z.copy(zIn);
    z.normalize();
    const x = X.clone().addScaledVector(z, -X.dot(z));
    if (x.lengthSq() < 1e-12) x.copy(xIn);
    x.normalize();
    dh = {
      alpha: Math.atan2(-z.y, z.z) / DEG,
      a: p.x,
      theta: Math.atan2(new THREE.Vector3().crossVectors(xIn, x).dot(z), xIn.dot(x)) / DEG,
      d: p.dot(z),
    };
  } else {
    // Rz(θ) · Tz(d) · Tx(a) · Rx(α): the output x axis is the common normal of the two z axes.
    const x = new THREE.Vector3().crossVectors(zIn, Z);
    if (x.lengthSq() < 1e-12) {
      // Parallel z axes: any normal works, so aim it at the output origin (or along the output x).
      x.set(p.x, p.y, 0);
      if (x.lengthSq() < 1e-12) x.set(X.x, X.y, 0);
      if (x.lengthSq() < 1e-12) x.copy(xIn);
    }
    x.normalize();
    if (x.dot(X) < 0) x.negate(); // of the two directions, take the one nearer the real x axis
    const y = new THREE.Vector3().crossVectors(zIn, x);
    dh = {
      theta: Math.atan2(x.y, x.x) / DEG,
      d: p.z,
      a: p.dot(x),
      alpha: Math.atan2(-Z.dot(y), Z.z) / DEG,
    };
  }
  for (const k in dh) dh[k] = tidy(dh[k]);
  const fit = dhToMatrix(dh, convention);
  const posErr = new THREE.Vector3().setFromMatrixPosition(fit).distanceTo(p);
  const rel = new THREE.Matrix4().extractRotation(fit).transpose().multiply(new THREE.Matrix4().extractRotation(m)).elements;
  const angErr = Math.acos(Math.min(1, Math.max(-1, (rel[0] + rel[5] + rel[10] - 1) / 2))) / DEG;
  return { dh, posErr, angErr };
}

// ---- Reference frames for pose-entered link outputs ----
// A ref is 'input' (the port the link sits on), 'world', or 'nodeId:outputIndex'.
// A link is rigid, so an output measured from some other frame is solved once with
// every joint at 0° (the home pose) and then stays fixed relative to the link's input.

/**
 * Solves every link's local output transforms.
 * Returns { links: {id: Matrix4[]}, home: {id: world Matrix4[] at home pose}, fallbacks: {id: Set<outputIndex>} }.
 */
export function solveLinks(robot) {
  const home = {}, links = {}, fallbacks = {};
  const busy = new Set();
  const inputOf = (id) => {
    const p = robot.nodes[id].parent;
    return p ? outputsOf(p.nodeId)[p.index] : new THREE.Matrix4();
  };
  const frameAt = (ref) => {
    if (ref === 'world') return new THREE.Matrix4();
    const [id, i] = ref.split(':');
    const m = robot.nodes[id] && outputsOf(id)[Number(i)];
    if (!m) throw new Error(`missing frame ${ref}`);
    return m;
  };
  const outputsOf = (id) => {
    if (home[id]) return home[id];
    if (busy.has(id)) throw new Error('circular reference frame');
    busy.add(id);
    try {
      const n = robot.nodes[id];
      const input = inputOf(id);
      let local;
      if (n.type === 'link') {
        local = links[id] = n.outputs.map((o, i) => {
          if (n.spec !== 'pose') return dhToMatrix(o.dh);
          try {
            return poseLocal(o.pose, input, frameAt);
          } catch {
            // The frame is gone or depends on this link; measure from the port instead.
            (fallbacks[id] ??= new Set()).add(i);
            return poseToMatrix(o.pose);
          }
        });
      } else {
        local = localKinematics({ ...n, q: 0 }).outputs;
      }
      return (home[id] = local.map((m) => input.clone().multiply(m)));
    } finally {
      busy.delete(id);
    }
  };
  for (const id of Object.keys(robot.nodes)) outputsOf(id);
  return { links, fallbacks, home };
}

/** Output relative to the link input, from a pose measured in frame `ref` (all home-pose world matrices). */
function poseLocal(pose, input, frameAt) {
  const P = poseToMatrix(pose);
  if (!pose.ref || pose.ref === 'input') return P;
  return input.clone().invert().multiply(frameAt(pose.ref)).multiply(P);
}

/** Home-pose world matrix of a reference frame, given the link's home input frame. */
export function homeFrame(solved, ref, input) {
  if (!ref || ref === 'input') return input.clone();
  if (ref === 'world') return new THREE.Matrix4();
  const [id, i] = ref.split(':');
  return solved.home[id]?.[Number(i)]?.clone() ?? input.clone();
}

/** Re-express a local output transform as a pose measured from `ref`. */
export function poseFromLocal(local, ref, input, solved) {
  const F = homeFrame(solved, ref, input);
  return { ...matrixToPose(F.invert().multiply(input.clone().multiply(local))), ref };
}

// ---- Per-part local kinematics (everything relative to the part's input frame) ----

/** Horizontal distance from the input axis to the actuator output: motor reach (first gearbox included) plus each extra gearbox. */
export function actuatorReach({ motor, gearboxes }) {
  return MOTORS[motor].reach + gearboxes.slice(1).reduce((sum, k) => sum + GEARBOXES[k].length, 0);
}

/**
 * Right-angle actuator: T = Rz(φ) · Tx(reach) · Tz(rise) · Ry(90°) · Rz(q), with reach from actuatorReach.
 * φ (node.mount) is how far the motor is turned about the port's z axis when it is bolted on.
 * The input z axis points up; at φ = 0 the output z axis (the joint axis) points along input +x.
 * Each segment is { start, length }: a frame on the output axis and how far the module runs along it.
 */
function actuatorKinematics(node) {
  const motor = MOTORS[node.motor];
  const mount = new THREE.Matrix4().makeRotationZ((node.mount ?? 0) * DEG);
  const turn = new THREE.Matrix4().makeRotationY(90 * DEG);
  const along = (x) => mount.clone().multiply(new THREE.Matrix4().makeTranslation(x, 0, motor.rise)).multiply(turn);
  // The housing is centred on the input axis. Its back half is drawn as the motor and its
  // front half as the first gearbox, which sits inside it.
  const half = motor.length / 2;
  const [first, ...extra] = node.gearboxes.map((k) => GEARBOXES[k]);
  const segments = [
    { family: 'motor', def: motor, start: along(-half), length: half },
    { family: first.family, def: first, start: along(0), length: half },
  ];
  let x = motor.reach;
  for (const gb of extra) {
    segments.push({ family: gb.family, def: gb, start: along(x), length: gb.length });
    x += gb.length;
  }
  const joint = along(x);
  const output = joint.clone().multiply(new THREE.Matrix4().makeRotationZ(node.q * DEG));
  // mount: the input frame turned by φ, where the motor's own CAD file is placed.
  return { segments, mount, joint, elbow: new THREE.Vector3(0, 0, motor.rise), outputs: [output] };
}

export const DEFAULT_ROD = { lon: 0, lat: 90 }; // straight up the port's z axis

/**
 * Direction a pose-mode rod leaves the port, in the port frame. Longitude turns about the
 * port's z axis from +x toward +y; latitude tilts up from the x-y plane (90° = along +z).
 */
export function rodDirection({ lon = 0, lat = 90 } = DEFAULT_ROD) {
  const c = Math.cos(lat * DEG);
  return new THREE.Vector3(c * Math.cos(lon * DEG), c * Math.sin(lon * DEG), Math.sin(lat * DEG));
}

/**
 * Each output is its own transform from the shared input: D-H, or a pose (solved by solveLinks,
 * passed in as `solved`). A pose output's rod leaves the input along its rod direction, bends at
 * the point on that line nearest the output, and runs straight to the output.
 */
function linkKinematics(node, solved) {
  const outputs = node.outputs.map((o, i) => solved?.[i] ?? (node.spec === 'pose' ? poseToMatrix(o.pose) : dhToMatrix(o.dh)));
  return {
    outputs,
    elbows: node.outputs.map((o, i) => {
      if (node.spec !== 'pose') return dhElbow(o.dh);
      const dir = rodDirection(o.rod);
      const end = new THREE.Vector3().setFromMatrixPosition(outputs[i]);
      return dir.multiplyScalar(Math.max(0, end.dot(dir)));
    }),
  };
}

export function localKinematics(node, solved) {
  // The base is a link too (the root one, with no parent).
  return node.type === 'actuator' ? actuatorKinematics(node) : linkKinematics(node, solved);
}

/**
 * Walk the robot tree from the base, composing transforms.
 * Returns { [nodeId]: { input, outputs[], local } } with input/outputs in world coordinates.
 */
export function forwardKinematics(robot, solved = solveLinks(robot)) {
  const frames = {};
  const visit = (id, input) => {
    const node = robot.nodes[id];
    const local = localKinematics(node, solved.links[id]);
    const outputs = local.outputs.map((m) => input.clone().multiply(m));
    frames[id] = { input, outputs, local };
    node.children.forEach((child, i) => { if (child) visit(child, outputs[i]); });
  };
  visit(robot.rootId, new THREE.Matrix4());
  return frames;
}

// ---- Gear ratio helpers ----

const gcd = (a, b) => (b ? gcd(b, a % b) : a);

/** Product of the stage ratios, reduced: e.g. [cycloidal_18, harmonic_45_2] → [405, 1]. */
export function stackRatio(keys) {
  let n = 1, d = 1;
  for (const k of keys) { n *= GEARBOXES[k].ratio[0]; d *= GEARBOXES[k].ratio[1]; }
  const g = gcd(n, d);
  return [n / g, d / g];
}

/** Always "x:1", e.g. [45, 2] → "22.5:1". */
export function formatRatio([n, d]) {
  return `${+(n / d).toFixed(2)}:1`;
}
