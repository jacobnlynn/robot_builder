import * as THREE from 'three';
import { MarchingCubes } from 'three/addons/objects/MarchingCubes.js';
import { JOINT_LIMIT } from './constants.js';
import { forwardKinematics, solveLinks } from './kinematics.js';

// Workspaces of every end effector (open output), found by sampling: random joint angles within
// the slider limits are pushed through a fast forward-kinematics chain and the tip positions are
// binned into voxels. A smooth surface is then wrapped around the filled voxels.
//  - Reachable: every voxel some sample's tip landed in.
//  - Dexterous: voxels where the tip could point (its output z axis) in nearly every direction.
// Self-collision and the ground are ignored. The base is fixed, so everything is in world frame.

const REACH_RES = 56; // voxels along the longest side
const DEX_RES = 26;
const PAD = 3; // empty voxels around the grid; marching cubes skips the outer two
const REACH_SAMPLES_PER_JOINT = 60000;
const DEX_SAMPLES = 1500000; // per end effector with enough joints
const DEX_MIN_JOINTS = 5; // tip position (3) + pointing direction (2)
const DEX_BINS = 26; // pointing directions: the 26 neighbours of a cube cell
const DEX_COVERAGE = 24; // directions a voxel must reach (of 26) to count as dexterous
const CHUNK = 40000; // samples between yields to the page

/** Fixed transforms between joints: tip = A0 · Rz(q1) · A1 · … · Rz(qk) · Ak (column-major). */
function chains(robot) {
  const saved = Object.values(robot.nodes).map((n) => [n, n.q]);
  for (const [n] of saved) if (n.type === 'actuator') n.q = 0;
  const frames = forwardKinematics(robot, solveLinks(robot));
  for (const [n, q] of saved) if (n.type === 'actuator') n.q = q;

  const list = [];
  for (const n of Object.values(robot.nodes)) {
    n.children.forEach((c, index) => {
      if (c) return;
      // Walk up from the open output to the base, collecting each part's output matrix on the path.
      const steps = [];
      let id = n.id, out = index;
      while (id) {
        const part = robot.nodes[id];
        steps.unshift({ m: frames[id].local.outputs[out], joint: part.type === 'actuator' });
        if (!part.parent) break;
        out = part.parent.index;
        id = part.parent.nodeId;
      }
      const fixed = [];
      let acc = new THREE.Matrix4();
      for (const s of steps) {
        acc = acc.clone().multiply(s.m);
        if (s.joint) { fixed.push(Float64Array.from(acc.elements)); acc = new THREE.Matrix4(); }
      }
      fixed.push(Float64Array.from(acc.elements));
      list.push({ fixed, joints: fixed.length - 1 });
    });
  }
  return list;
}

// out = a · b, 4×4 column-major
function mul(out, a, b) {
  for (let c = 0; c < 4; c++) {
    const b0 = b[c * 4], b1 = b[c * 4 + 1], b2 = b[c * 4 + 2], b3 = b[c * 4 + 3];
    for (let r = 0; r < 4; r++) out[c * 4 + r] = a[r] * b0 + a[4 + r] * b1 + a[8 + r] * b2 + a[12 + r] * b3;
  }
}
// m = m · Rz(q), in place: only the first two columns change
function rotZ(m, q) {
  const c = Math.cos(q), s = Math.sin(q);
  for (let r = 0; r < 4; r++) {
    const x = m[r], y = m[4 + r];
    m[r] = x * c + y * s;
    m[4 + r] = -x * s + y * c;
  }
}

/** Samples one chain: calls visit(x, y, z, zx, zy, zz) with the tip position and pointing axis. */
function sampler(chain) {
  const T = new Float64Array(16), tmp = new Float64Array(16);
  const lim = JOINT_LIMIT * Math.PI / 180;
  return (visit) => {
    T.set(chain.fixed[0]);
    for (let j = 1; j < chain.fixed.length; j++) {
      rotZ(T, (Math.random() * 2 - 1) * lim);
      mul(tmp, T, chain.fixed[j]);
      T.set(tmp);
    }
    visit(T[12], T[13], T[14], T[8], T[9], T[10]);
  };
}

const tick = () => new Promise((r) => setTimeout(r));

/** Voxel grid over a box with a fixed number of cells on its longest side. */
function grid(box, res) {
  const size = box.getSize(new THREE.Vector3());
  const cell = Math.max(size.x, size.y, size.z, 1) / res;
  const R = res + 2 * PAD; // cube, so the marching-cubes object can be scaled uniformly
  const center = box.getCenter(new THREE.Vector3());
  return {
    R, cell, center,
    index(x, y, z) {
      const i = Math.floor((x - center.x) / cell + R / 2);
      const j = Math.floor((y - center.y) / cell + R / 2);
      const k = Math.floor((z - center.z) / cell + R / 2);
      if (i < PAD || j < PAD || k < PAD || i >= R - PAD || j >= R - PAD || k >= R - PAD) return -1;
      return i + R * (j + R * k);
    },
  };
}

function dirBin(x, y, z) {
  const q = (v) => (v > 0.38 ? 2 : v < -0.38 ? 0 : 1);
  const b = q(x) + 3 * q(y) + 9 * q(z); // 0..26, 13 is the (impossible) centre
  return b > 13 ? b - 1 : b;
}

const popcount = (v) => { let c = 0; while (v) { v &= v - 1; c++; } return c; };

/** Fill neighbours of filled voxels, closing the gaps between samples. */
function dilate(field, R) {
  const out = field.slice();
  for (let k = 1; k < R - 1; k++) for (let j = 1; j < R - 1; j++) for (let i = 1; i < R - 1; i++) {
    const q = i + R * (j + R * k);
    if (field[q]) continue;
    if (field[q - 1] || field[q + 1] || field[q - R] || field[q + R] || field[q - R * R] || field[q + R * R]) out[q] = 1;
  }
  return out;
}

/**
 * Compute both workspaces. onProgress(0..1) is called as it goes.
 * Returns { reach, dex, dexSkipped } where reach/dex are { field, g } or null when empty.
 */
export async function computeWorkspaces(robot, onProgress = () => {}) {
  const list = chains(robot).filter((c) => c.joints > 0);
  if (!list.length) return { reach: null, dex: null, dexSkipped: true };
  const dexList = list.filter((c) => c.joints >= DEX_MIN_JOINTS);
  const work = list.reduce((s, c) => s + c.joints * REACH_SAMPLES_PER_JOINT, 0) * 2 + dexList.length * DEX_SAMPLES;
  let done = 0;
  const step = async (n) => { done += n; onProgress(Math.min(1, done / work)); await tick(); };

  // Pass 1: bounding box of the tips.
  const box = new THREE.Box3();
  const p = new THREE.Vector3();
  for (const c of list) {
    const s = sampler(c), n = c.joints * REACH_SAMPLES_PER_JOINT;
    for (let a = 0; a < n; a += CHUNK) {
      for (let b = 0; b < CHUNK; b++) s((x, y, z) => box.expandByPoint(p.set(x, y, z)));
      await step(CHUNK);
    }
  }

  // Pass 2: reachable voxels.
  const rg = grid(box, REACH_RES);
  let field = new Float32Array(rg.R ** 3);
  for (const c of list) {
    const s = sampler(c), n = c.joints * REACH_SAMPLES_PER_JOINT;
    const visit = (x, y, z) => { const q = rg.index(x, y, z); if (q >= 0) field[q] = 1; };
    for (let a = 0; a < n; a += CHUNK) {
      for (let b = 0; b < CHUNK; b++) s(visit);
      await step(CHUNK);
    }
  }
  field = dilate(field, rg.R);
  const reach = { field, g: rg };

  // Pass 3: dexterous voxels, per end effector (one tip can't borrow another's directions).
  let dex = null;
  if (dexList.length) {
    const dg = grid(box, DEX_RES);
    const dfield = new Float32Array(dg.R ** 3);
    let any = false;
    for (const c of dexList) {
      const masks = new Uint32Array(dg.R ** 3);
      const s = sampler(c);
      const visit = (x, y, z, zx, zy, zz) => { const q = dg.index(x, y, z); if (q >= 0) masks[q] |= 1 << dirBin(zx, zy, zz); };
      for (let a = 0; a < DEX_SAMPLES; a += CHUNK) {
        for (let b = 0; b < CHUNK; b++) s(visit);
        await step(CHUNK);
      }
      for (let q = 0; q < masks.length; q++) if (popcount(masks[q]) >= DEX_COVERAGE) { dfield[q] = 1; any = true; }
    }
    if (any) dex = { field: dfield, g: dg };
  }
  onProgress(1);
  return { reach, dex, dexSkipped: !dexList.length };
}

/** See-through surface around the filled voxels of a workspace. */
export function workspaceMesh(ws, color, opacity) {
  const { R, cell, center } = ws.g;
  const material = new THREE.MeshLambertMaterial({
    color, transparent: true, opacity, depthWrite: false, side: THREE.DoubleSide,
  });
  const mc = new MarchingCubes(R, material, false, false, 100000);
  mc.frustumCulled = false;
  mc.isolation = 0.5;
  mc.field.set(ws.field);
  mc.blur(1);
  mc.update();
  // Field value i sits at (i − R/2) / (R/2) in the object's [-1, 1] box; voxel i spans [i, i + 1).
  mc.position.copy(center).addScalar(cell / 2);
  mc.scale.setScalar(cell * R / 2);
  mc.renderOrder = 1;
  return mc;
}
