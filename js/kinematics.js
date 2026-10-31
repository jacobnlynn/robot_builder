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
// every joint at 0°, passive ones too (the home pose), and then stays fixed relative to the link's input.

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
 * Rotation of a passive joint on a link output: a hinge turns q about the output's z axis, a ball
 * turns Rz(z) · Ry(y) · Rx(x). Identity for any other output.
 */
export function passiveRotation(o) {
  if (o.end === 'hinge') return new THREE.Matrix4().makeRotationZ((o.q ?? 0) * DEG);
  if (o.end !== 'ball') return new THREE.Matrix4();
  const { x = 0, y = 0, z = 0 } = o.ball ?? {};
  return new THREE.Matrix4().makeRotationZ(z * DEG)
    .multiply(new THREE.Matrix4().makeRotationY(y * DEG))
    .multiply(new THREE.Matrix4().makeRotationX(x * DEG));
}

/**
 * Each output is its own transform from the shared input: D-H, or a pose (solved by solveLinks,
 * passed in as `solved`), then turned by its passive joint if it is a ball or hinge. A pose output's rod leaves the input along its rod direction, bends at
 * the point on that line nearest the output, and runs straight to the output.
 */
function linkKinematics(node, solved) {
  const outputs = node.outputs.map((o, i) => (solved?.[i]?.clone() ?? (node.spec === 'pose' ? poseToMatrix(o.pose) : dhToMatrix(o.dh)))
    .multiply(passiveRotation(o)));
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
 * Walk the robot tree from the base, composing transforms. If the robot has closed loops, their
 * passive joints are first turned so every loop closes (closeLoops).
 * Returns { [nodeId]: { input, outputs[], local } } with input/outputs in world coordinates.
 */
export function forwardKinematics(robot, solved = solveLinks(robot)) {
  if (robot.loops?.length) closeLoops(robot, solved);
  return treeKinematics(robot, solved);
}

function treeKinematics(robot, solved) {
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

// ---- Closed loops ----
// A loop joins two open ball or hinge outputs (robot.loops: [{ a: {nodeId, index}, b }]). The
// robot is still a tree; the loop is a constraint that the two outputs meet. The passive joints on
// the tree path between them are the unknowns, solved (damped least squares, from their current
// angles so the mechanism moves smoothly) so that the outputs share a point, and for two hinges
// also share an axis. The actuators drive the loop; the joints at the two joined outputs only turn
// about their own axis, so they don't take part.

const AXIS_WEIGHT = 100; // mm per radian of hinge-axis misalignment, to weigh it against position
const LOOP_TOL = 0.005; // mm: closed
const LOOP_ITERATIONS = 60;

/** The unknowns: passive joints on the tree paths between each loop's two outputs, as { o, key }. */
export function loopUnknowns(robot) {
  const ports = new Map(); // 'nodeId:index' → { o, n }
  const upFrom = (id) => {
    const path = [];
    for (let n = robot.nodes[id]; n?.parent; n = robot.nodes[n.parent.nodeId]) path.push(`${n.parent.nodeId}:${n.parent.index}`);
    return path;
  };
  for (const { a, b } of robot.loops ?? []) {
    const pa = upFrom(a.nodeId), pb = upFrom(b.nodeId);
    // Drop the shared part of the two paths (from where they meet up to the base).
    while (pa.length && pb.length && pa.at(-1) === pb.at(-1)) { pa.pop(); pb.pop(); }
    for (const key of [...pa, ...pb]) {
      const [id, i] = key.split(':');
      const o = robot.nodes[id]?.outputs?.[Number(i)];
      if (o?.end === 'hinge' || o?.end === 'ball') ports.set(key, o);
    }
  }
  const unknowns = [];
  for (const [port, o] of ports) {
    if (o.end === 'hinge') unknowns.push({ o, key: 'q', port });
    else for (const k of ['x', 'y', 'z']) unknowns.push({ o, key: k, port });
  }
  return unknowns;
}

const getAngle = (u) => (u.key === 'q' ? u.o.q ?? 0 : u.o.ball?.[u.key] ?? 0);
const setAngle = (u, v) => { if (u.key === 'q') u.o.q = v; else u.o.ball = { ...u.o.ball, [u.key]: v }; };

/** How far every loop is from closing: position gaps (mm) and, between two hinges, axis misalignment. */
function loopResidual(robot, solved) {
  const frames = treeKinematics(robot, solved);
  const r = [];
  let gap = 0;
  for (const { a, b } of robot.loops) {
    const A = frames[a.nodeId]?.outputs[a.index], B = frames[b.nodeId]?.outputs[b.index];
    if (!A || !B) continue;
    const ea = A.elements, eb = B.elements;
    const d = [ea[12] - eb[12], ea[13] - eb[13], ea[14] - eb[14]];
    r.push(...d);
    gap = Math.max(gap, Math.hypot(...d));
    if (robot.nodes[a.nodeId].outputs[a.index].end === 'hinge' && robot.nodes[b.nodeId].outputs[b.index].end === 'hinge') {
      const za = new THREE.Vector3(ea[8], ea[9], ea[10]), zb = new THREE.Vector3(eb[8], eb[9], eb[10]);
      r.push(...za.cross(zb).multiplyScalar(AXIS_WEIGHT).toArray()); // zero when parallel (either way round)
    }
  }
  return { r, gap };
}

/** Solve A x = y for a small dense system (Gaussian elimination with pivoting). */
function solveSmall(A, y) {
  const n = y.length, M = A.map((row, i) => [...row, y[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let i = c + 1; i < n; i++) if (Math.abs(M[i][c]) > Math.abs(M[p][c])) p = i;
    [M[c], M[p]] = [M[p], M[c]];
    if (Math.abs(M[c][c]) < 1e-12) continue;
    for (let i = c + 1; i < n; i++) {
      const f = M[i][c] / M[c][c];
      for (let k = c; k <= n; k++) M[i][k] -= f * M[c][k];
    }
  }
  const x = new Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    let s = M[i][n];
    for (let k = i + 1; k < n; k++) s -= M[i][k] * x[k];
    x[i] = Math.abs(M[i][i]) < 1e-12 ? 0 : s / M[i][i];
  }
  return x;
}

/**
 * Turn the loops' passive joints until every loop closes (or as close as it can get). Writes the
 * angles into the outputs and sets robot.loopGap (mm, the widest gap left; 0 when all closed).
 */
export function closeLoops(robot, solved = solveLinks(robot)) {
  const unknowns = loopUnknowns(robot);
  let { r, gap } = loopResidual(robot, solved);
  const cost = (v) => v.reduce((s, x) => s + x * x, 0);
  let c = cost(r), lambda = 1e-3;
  for (let it = 0; it < LOOP_ITERATIONS && unknowns.length && Math.sqrt(c) > LOOP_TOL; it++) {
    // Jacobian by finite differences (residual per degree).
    const x0 = unknowns.map(getAngle), h = 1e-3;
    const J = unknowns.map((u, j) => {
      setAngle(u, x0[j] + h);
      const rj = loopResidual(robot, solved).r;
      setAngle(u, x0[j]);
      return rj.map((v, k) => (v - r[k]) / h);
    }); // J[j] = column j
    const n = unknowns.length;
    const JtJ = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, k) => J[i].reduce((s, v, m) => s + v * J[k][m], 0)));
    const Jtr = J.map((col) => col.reduce((s, v, m) => s + v * r[m], 0));
    let improved = false;
    for (let tries = 0; tries < 8 && !improved; tries++) {
      const A = JtJ.map((row, i) => row.map((v, k) => v + (i === k ? lambda * (JtJ[i][i] + 1e-6) : 0)));
      const dx = solveSmall(A, Jtr.map((v) => -v));
      unknowns.forEach((u, j) => setAngle(u, x0[j] + Math.max(-30, Math.min(30, dx[j]))));
      const next = loopResidual(robot, solved);
      if (cost(next.r) < c) { ({ r, gap } = next); c = cost(r); lambda = Math.max(1e-7, lambda / 3); improved = true; }
      else { unknowns.forEach((u, j) => setAngle(u, x0[j])); lambda *= 10; }
    }
    if (!improved) break;
  }
  // Keep the angles tidy: within ±180°.
  for (const u of unknowns) setAngle(u, ((getAngle(u) + 540) % 360) - 180);
  robot.loopGap = gap < 0.05 ? 0 : gap;
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
