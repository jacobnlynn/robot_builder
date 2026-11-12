// Export URDF: a .zip with robot.urdf and meshes/ (OBJ, mm, scaled to metres in the URDF; MuJoCo,
// ROS/RViz, Gazebo and PyBullet all read OBJ).
// Rigid bodies (URDF links) are what moves together: the base body holds everything up to the
// first joints, and each actuator starts a new body at its output. A body carries the motor
// shells mounted on it (CAD-tab shells, not the full models) and the filled link parts it holds.
// Every actuator is a revolute joint about its output z axis, placed at its output frame with
// every joint at 0°. Limits are the ±JOINT_LIMIT sliders; velocity is the simulate top speed;
// effort is the motor's torque times the joint's total gear ratio.
// Passive joints (ball or hinge link outputs) are unpowered: a hinge is a continuous joint about the
// output's z axis, and a ball (which URDF has no joint for) is three continuous joints about z, y
// and x in turn, through two tiny bodies at the same place. URDF can't describe a closed loop, so
// each loop's joint is left open (a comment in the file says where); the robot is exported as its tree.
// Inertial data assumes everything is one uniform density (DENSITY): each actuator module uses
// the volume, centre of mass and inertia of its full model (MODULES, measured from
// models/<family>_gearbox.step and _extension.step), each link part is measured from its own solid, and a body's
// values combine everything it carries.
import * as THREE from 'three';
import { state, node, isPassive } from './robot.js';
import { FAMILY_COLORS, JOINT_LIMIT } from './constants.js';
import { forwardKinematics, stackRatio } from './kinematics.js';
import { hooks, cadGeometry, actuatorCadKeys, whenCadReady } from './viewer.js';
import { jointSpeed } from './simulate.js';


// ---- Small helpers ----

const fmt = (v) => String(Number(v.toFixed(6)) || 0);
const ratio = (n) => { const [a, b] = stackRatio(n.gearboxes); return a / b; };
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&apos;' }[c]));
const rgba = (hex) => [(hex >> 16) & 255, (hex >> 8) & 255, hex & 255].map((c) => fmt(c / 255)).join(' ') + ' 1';

/** URDF <origin> of a transform, with xyz in metres and rpy (roll, pitch, yaw about fixed x, y, z). */
function origin(m) {
  const p = new THREE.Vector3().setFromMatrixPosition(m).multiplyScalar(0.001);
  const e = new THREE.Euler().setFromRotationMatrix(m, 'ZYX'); // R = Rz(yaw)·Ry(pitch)·Rx(roll)
  return `<origin xyz="${fmt(p.x)} ${fmt(p.y)} ${fmt(p.z)}" rpy="${fmt(e.x)} ${fmt(e.y)} ${fmt(e.z)}"/>`;
}

/** OBJ text from triangle positions (9 floats each), with coincident vertices merged. */
function obj(tri) {
  const index = new Map(), verts = [], faces = [];
  const id = (k) => {
    const key = `${tri[k].toFixed(3)} ${tri[k + 1].toFixed(3)} ${tri[k + 2].toFixed(3)}`;
    let i = index.get(key);
    if (i === undefined) { i = verts.length + 1; index.set(key, i); verts.push(`v ${key}`); }
    return i;
  };
  for (let t = 0; t < tri.length; t += 9) {
    const a = id(t), b = id(t + 3), c = id(t + 6);
    if (a !== b && b !== c && a !== c) faces.push(`f ${a} ${b} ${c}`);
  }
  return new TextEncoder().encode(['# Reconfigurable Robot Builder mesh (mm)', ...verts, ...faces, ''].join('\n'));
}

// Minimal .zip writer (files stored uncompressed).
const CRC = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; t[n] = c >>> 0; }
  return t;
})();
function crc32(bytes) {
  let c = 0xffffffff;
  for (let i = 0; i < bytes.length; i++) c = CRC[(c ^ bytes[i]) & 255] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}
function zip(files) {
  const enc = new TextEncoder();
  const parts = [], central = [];
  let offset = 0;
  for (const f of files) {
    const name = enc.encode(f.name), data = f.data, crc = crc32(data);
    const local = new DataView(new ArrayBuffer(30));
    local.setUint32(0, 0x04034b50, true); local.setUint16(4, 20, true); local.setUint16(8, 0, true);
    local.setUint32(14, crc, true); local.setUint32(18, data.length, true); local.setUint32(22, data.length, true);
    local.setUint16(26, name.length, true);
    parts.push(local.buffer, name, data);
    const cen = new DataView(new ArrayBuffer(46));
    cen.setUint32(0, 0x02014b50, true); cen.setUint16(4, 20, true); cen.setUint16(6, 20, true);
    cen.setUint32(16, crc, true); cen.setUint32(20, data.length, true); cen.setUint32(24, data.length, true);
    cen.setUint16(28, name.length, true); cen.setUint32(42, offset, true);
    central.push(cen.buffer, name);
    offset += 30 + name.length + data.length;
  }
  const size = central.reduce((s, b) => s + b.byteLength, 0);
  const end = new DataView(new ArrayBuffer(22));
  end.setUint32(0, 0x06054b50, true); end.setUint16(8, files.length, true); end.setUint16(10, files.length, true);
  end.setUint32(12, size, true); end.setUint32(16, offset, true);
  return new Blob([...parts, ...central, end.buffer], { type: 'application/zip' });
}

const MOTOR_TORQUE = 0.7; // N·m at the motor shaft, before any gearbox
// The tiny bodies between a ball joint's three axes: 1 g, a 1 cm³ cube's inertia (kg, mm, kg·mm²).
const TOKEN_MASS = { m: 0.001, c: new THREE.Vector3(), I: new THREE.Matrix3().set(1.7e-2, 0, 0, 0, 1.7e-2, 0, 0, 0, 1.7e-2) };
const DENSITY = 1.24e-6; // kg/mm³ (1.24 g/cm³, printed PLA), assumed uniform everywhere

// Full-model mass properties per unit density, in each module's own Z-up frame (mm):
// volume (mm³), centre of mass (mm) and inertia about it (mm⁵: ixx, iyy, izz, ixy, ixz, iyz).
// motor: motor + built-in gearbox (<family>_gearbox.step); extension: one stage (<family>_extension.step).
const MODULES = {
  planetary: {
    motor: { volume: 243812, com: [7.95, -0.22, 47.61], inertia: [2.2242e+08, 2.6452e+08, 2.1941e+08, -2.5799e+05, -1.8499e+07, -4.4105e+05] },
    extension: { volume: 107489, com: [-0.00, 0.00, 31.07], inertia: [5.6763e+07, 5.6749e+07, 7.9764e+07, 6.5091e-02, -1.4196e+03, -6.9335e+00] },
  },
  cycloidal: {
    motor: { volume: 231863, com: [8.32, -0.17, 47.14], inertia: [2.1420e+08, 2.6163e+08, 2.1757e+08, -3.2591e+05, -1.9316e+07, -5.5516e+05] },
    extension: { volume: 106977, com: [0.00, 0.20, 32.37], inertia: [5.3910e+07, 5.3880e+07, 7.6213e+07, 6.3469e+02, -1.9503e+02, 2.9880e+05] },
  },
  harmonic: {
    motor: { volume: 240453, com: [8.63, -0.22, 47.48], inertia: [2.1894e+08, 2.6711e+08, 2.2119e+08, -2.6533e+05, -1.9985e+07, -4.3824e+05] },
    extension: { volume: 109059, com: [-0.00, -0.00, 32.10], inertia: [5.5631e+07, 5.6710e+07, 7.6904e+07, -9.4440e+02, 7.2789e+02, 3.8075e+02] },
  },
};

/** Mass properties of a closed triangle mesh (mm) at uniform density: { m, c, I (Matrix3) }. */
function meshProps(tri) {
  let V = 0;
  const c = [0, 0, 0], C = [0, 0, 0, 0, 0, 0]; // ∫xx, ∫yy, ∫zz, ∫xy, ∫xz, ∫yz
  // ∫ p q over the tetrahedron (0, a, b, c) = det/120 · (Σ p_i q_i + (Σ p_i)(Σ q_i)).
  const sp = (det, p, q) => det / 120 * (p[0] * q[0] + p[1] * q[1] + p[2] * q[2] + (p[0] + p[1] + p[2]) * (q[0] + q[1] + q[2]));
  for (let t = 0; t < tri.length; t += 9) {
    const X = [tri[t], tri[t + 3], tri[t + 6]], Y = [tri[t + 1], tri[t + 4], tri[t + 7]], Z = [tri[t + 2], tri[t + 5], tri[t + 8]];
    const det = X[0] * (Y[1] * Z[2] - Z[1] * Y[2]) - Y[0] * (X[1] * Z[2] - Z[1] * X[2]) + Z[0] * (X[1] * Y[2] - Y[1] * X[2]);
    V += det / 6;
    c[0] += det * (X[0] + X[1] + X[2]) / 24; c[1] += det * (Y[0] + Y[1] + Y[2]) / 24; c[2] += det * (Z[0] + Z[1] + Z[2]) / 24;
    C[0] += sp(det, X, X); C[1] += sp(det, Y, Y); C[2] += sp(det, Z, Z);
    C[3] += sp(det, X, Y); C[4] += sp(det, X, Z); C[5] += sp(det, Y, Z);
  }
  const com = new THREE.Vector3(c[0] / V, c[1] / V, c[2] / V);
  // Second moments about the centre of mass, then the inertia tensor per unit density.
  const xx = C[0] - V * com.x * com.x, yy = C[1] - V * com.y * com.y, zz = C[2] - V * com.z * com.z;
  const xy = C[3] - V * com.x * com.y, xz = C[4] - V * com.x * com.z, yz = C[5] - V * com.y * com.z;
  return props({ volume: V, com: com.toArray(), inertia: [yy + zz, xx + zz, xx + yy, -xy, -xz, -yz] });
}

/** Mass properties from per-unit-density values: m (kg), c (mm), I (kg·mm², Matrix3). */
function props({ volume, com, inertia: [ixx, iyy, izz, ixy, ixz, iyz] }) {
  return {
    m: DENSITY * volume,
    c: new THREE.Vector3(...com),
    I: new THREE.Matrix3().set(ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz).multiplyScalar(DENSITY),
  };
}

/** The same mass, moved by the rigid transform m. */
function moved(p, m) {
  const R = new THREE.Matrix3().setFromMatrix4(m);
  return { m: p.m, c: p.c.clone().applyMatrix4(m), I: R.clone().multiply(p.I).multiply(R.clone().transpose()) };
}

/** Combined mass, centre of mass and inertia about it (parallel-axis theorem), or null if empty. */
function combine(list) {
  const m = list.reduce((s, p) => s + p.m, 0);
  if (!m) return null;
  const c = list.reduce((s, p) => s.addScaledVector(p.c, p.m), new THREE.Vector3()).divideScalar(m);
  const e = new Array(9).fill(0);
  for (const p of list) {
    const d = p.c.clone().sub(c).toArray(), dd = d[0] ** 2 + d[1] ** 2 + d[2] ** 2, q = p.I.elements;
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) e[j * 3 + i] += q[j * 3 + i] + p.m * ((i === j ? dd : 0) - d[i] * d[j]);
  }
  return { m, c, I: new THREE.Matrix3().fromArray(e) };
}

const fmt8 = (v) => (Math.abs(v) < 1e-12 ? '0' : String(Number(v.toPrecision(6))));

/** URDF <inertial> (kg, m, kg·m²). */
function inertial(p) {
  const e = p.I.elements.map((v) => v * 1e-6); // kg·mm² → kg·m² (symmetric, so element order doesn't matter)
  const c = p.c.clone().multiplyScalar(0.001);
  return `<inertial><origin xyz="${fmt(c.x)} ${fmt(c.y)} ${fmt(c.z)}" rpy="0 0 0"/><mass value="${fmt(p.m)}"/>`
    + `<inertia ixx="${fmt8(e[0])}" ixy="${fmt8(e[3])}" ixz="${fmt8(e[6])}" iyy="${fmt8(e[4])}" iyz="${fmt8(e[7])}" izz="${fmt8(e[8])}"/></inertial>`;
}

// ---- The robot ----

/**
 * Build the URDF zip. onProgress({ step, done, total }) reports as it goes.
 * Returns { blob, links, joints }.
 */
export async function exportURDF(onProgress = () => {}) {
  const keys = [...new Set(Object.values(state.robot.nodes).filter((n) => n.type === 'actuator').flatMap(actuatorCadKeys))];
  onProgress({ step: 'Loading the module models', done: 0, total: 1 });
  if (!(await whenCadReady(keys))) throw new Error('Some module models could not be loaded.');
  if (!hooks.linkFillTriangles) throw new Error('Filled links are not available, so links cannot be exported.');

  // Home pose: every joint at 0°, passive ones too, as URDF expects.
  const all = Object.values(state.robot.nodes);
  const saved = all.filter((n) => n.type === 'actuator').map((n) => [n, n.q]);
  const savedPassive = all.filter((n) => n.type === 'link').flatMap((n) => n.outputs.map((o) => [o, o.q, o.ball]));
  saved.forEach(([n]) => { n.q = 0; });
  savedPassive.forEach(([o]) => { o.q = 0; o.ball = { x: 0, y: 0, z: 0 }; });
  let frames;
  try { frames = forwardKinematics(state.robot); } finally {
    saved.forEach(([n, q]) => { n.q = q; });
    savedPassive.forEach(([o, q, ball]) => { o.q = q; o.ball = ball; });
  }

  // Unique, URDF-safe names.
  const used = new Set();
  const nameFor = (s) => {
    let base = String(s).trim().replace(/[^A-Za-z0-9_]+/g, '_').replace(/^_+|_+$/g, '') || 'part', name = base, k = 2;
    while (used.has(name)) name = `${base}_${k++}`;
    used.add(name);
    return name;
  };
  const baseBody = nameFor('base_link');
  const bodyOf = new Map(); // node id → URDF link its input belongs to
  const bodyFrame = new Map([[baseBody, new THREE.Matrix4()]]);
  const visuals = new Map([[baseBody, []]]); // URDF link → [{ mesh, matrix, color }]
  const masses = new Map([[baseBody, []]]); // URDF link → mass properties in its frame
  const joints = [];
  const order = [];
  const newBody = (name, frame) => {
    const b = nameFor(name);
    bodyFrame.set(b, frame.clone());
    visuals.set(b, []);
    masses.set(b, []);
    return b;
  };
  // A passive joint on output i of link n: the body after it (and the joint or joints that reach it).
  const passiveBody = (n, i, body, child) => {
    const o = n.outputs[i], frame = frames[n.id].outputs[i];
    const after = newBody(`${node(child).name}_link`, frame);
    if (o.end === 'hinge') {
      joints.push({ passive: true, name: nameFor(`${n.name}_hinge${n.outputs.length > 1 ? i + 1 : ''}`), parent: body, child: after, axis: '0 0 1' });
      return after;
    }
    // Ball: Rz · Ry · Rx, through two tiny bodies with a token mass so simulators keep them.
    const ball = nameFor(`${n.name}_ball${n.outputs.length > 1 ? i + 1 : ''}`);
    const zBody = newBody(`${ball}_z_link`, frame), yBody = newBody(`${ball}_y_link`, frame);
    for (const b of [zBody, yBody]) masses.get(b).push(TOKEN_MASS);
    joints.push({ passive: true, name: nameFor(`${ball}_z`), parent: body, child: zBody, axis: '0 0 1' },
      { passive: true, name: nameFor(`${ball}_y`), parent: zBody, child: yBody, axis: '0 1 0' },
      { passive: true, name: nameFor(`${ball}_x`), parent: yBody, child: after, axis: '1 0 0' });
    return after;
  };
  const visit = (id, body) => {
    order.push(id);
    bodyOf.set(id, body);
    const n = node(id);
    let childBody = body;
    if (n.type === 'actuator') {
      childBody = nameFor(`${n.name}_link`);
      bodyFrame.set(childBody, frames[id].outputs[0].clone());
      visuals.set(childBody, []);
      masses.set(childBody, []);
      joints.push({ n, name: nameFor(n.name), parent: body, child: childBody });
    }
    n.children.forEach((c, i) => c && visit(c, isPassive(n, i) ? passiveBody(n, i, childBody, c) : childBody));
  };
  visit(state.robot.rootId, baseBody);

  const files = [];
  const meshes = new Map(); // mesh key → file name, written once
  const addMesh = (key, tri) => {
    if (!meshes.has(key)) { meshes.set(key, `meshes/${key}.obj`); files.push({ name: `meshes/${key}.obj`, data: obj(tri) }); }
    return meshes.get(key);
  };
  const triOf = (g) => (g.index ? g.toNonIndexed() : g).attributes.position.array;

  const total = order.length + 1;
  let done = 0;
  const tick = (step) => { onProgress({ step, done: ++done, total }); return new Promise((r) => setTimeout(r)); };

  for (const id of order) {
    const n = node(id), f = frames[id], body = bodyOf.get(id);
    const toBody = bodyFrame.get(body).clone().invert();
    if (n.type === 'actuator') {
      const [, first, ...extra] = f.local.segments;
      const at = (m) => toBody.clone().multiply(f.input).multiply(m);
      const motor = cadGeometry(`motor:${first.family}`);
      visuals.get(body).push({ mesh: addMesh(`motor_${first.family}`, triOf(motor.motor)), matrix: at(f.local.mount), color: FAMILY_COLORS.motor, label: `${n.name} motor` });
      visuals.get(body).push({ mesh: addMesh(`motor_${first.family}_gearbox`, triOf(motor.firstGearbox)), matrix: at(f.local.mount), color: FAMILY_COLORS[first.family], label: `${n.name} ${first.family} gearbox` });
      extra.forEach((s, k) => visuals.get(body).push({ mesh: addMesh(`extension_${s.family}`, triOf(cadGeometry(`extension:${s.family}`))), matrix: at(s.start), color: FAMILY_COLORS[s.family], label: `${n.name} ${s.family} extension ${k + 1}` }));
      masses.get(body).push(moved(props(MODULES[first.family].motor), at(f.local.mount)));
      extra.forEach((s) => masses.get(body).push(moved(props(MODULES[s.family].extension), at(s.start))));
    } else if (hooks.isPartRoot?.(n)) {
      const tri = await hooks.linkFillTriangles(n);
      const key = nameFor(`link_${n.name}`);
      visuals.get(body).push({ mesh: addMesh(key, tri), matrix: toBody.clone().multiply(f.input), color: FAMILY_COLORS.link ?? 0x8a97a8, label: `${n.name}` });
      masses.get(body).push(moved(meshProps(tri), toBody.clone().multiply(f.input)));
    }
    await tick(`Adding ${n.name}`);
  }

  // Materials, one per colour used.
  const colours = [...new Set([...visuals.values()].flat().map((v) => v.color))];
  const material = (c) => `mat_${c.toString(16).padStart(6, '0')}`;
  const lines = [
    '<?xml version="1.0"?>',
    `<!-- ${esc(state.name || 'robot')}: exported from the Reconfigurable Robot Builder. Meshes are in mm (scale 0.001).`,
    `     Joint effort = ${MOTOR_TORQUE} N·m motor torque × gear ratio. Inertial data assumes a uniform ${fmt(DENSITY * 1e6)} g/cm³. -->`,
    `<robot name="${esc(nameFor(state.name || 'robot'))}">`,
    ...colours.map((c) => `  <material name="${material(c)}"><color rgba="${rgba(c)}"/></material>`),
    ...state.robot.loops.map(({ a, b }) => `  <!-- Closed loop not in this file (URDF has no loops): ${esc(node(a.nodeId).name)} output ${a.index + 1} joins ${esc(node(b.nodeId).name)} output ${b.index + 1}. -->`),
  ];
  for (const [body, list] of visuals) {
    lines.push(`  <link name="${body}">`);
    const mass = combine(masses.get(body));
    if (mass) lines.push(`    ${inertial(mass)}`);
    for (const v of list) {
      const geom = `<geometry><mesh filename="${v.mesh}" scale="0.001 0.001 0.001"/></geometry>`;
      lines.push(`    <!-- ${esc(v.label)} -->`,
        `    <visual>${origin(v.matrix)}${geom}<material name="${material(v.color)}"/></visual>`,
        `    <collision>${origin(v.matrix)}${geom}</collision>`);
    }
    lines.push('  </link>');
  }
  const lim = (JOINT_LIMIT * Math.PI) / 180;
  for (const j of joints) {
    const toParent = bodyFrame.get(j.parent).clone().invert().multiply(bodyFrame.get(j.child));
    if (j.passive) {
      lines.push(`  <joint name="${j.name}" type="continuous">`, `    <parent link="${j.parent}"/>`, `    <child link="${j.child}"/>`,
        `    ${origin(toParent)}`, `    <axis xyz="${j.axis}"/>`, '    <dynamics damping="0" friction="0"/>', '  </joint>');
      continue;
    }
    lines.push(`  <joint name="${j.name}" type="revolute">`,
      `    <parent link="${j.parent}"/>`,
      `    <child link="${j.child}"/>`,
      `    ${origin(toParent)}`,
      '    <axis xyz="0 0 1"/>',
      `    <limit lower="${fmt(-lim)}" upper="${fmt(lim)}" effort="${fmt(MOTOR_TORQUE * ratio(j.n))}" velocity="${fmt((jointSpeed(j.n) * Math.PI) / 180)}"/>`,
      '  </joint>');
  }
  lines.push('</robot>', '');
  files.unshift({ name: 'robot.urdf', data: new TextEncoder().encode(lines.join('\n')) });
  await tick('Done');
  const mass = [...masses.values()].flat().reduce((sum, q) => sum + q.m, 0);
  return { blob: zip(files), links: visuals.size, joints: joints.length, mass };
}
