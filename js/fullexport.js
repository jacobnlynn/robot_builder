// Export STL: the whole robot as one binary STL at its current pose. Every actuator is its full
// model (motor with all interior parts and its built-in gearbox, by the first gearbox's type),
// every extra stage the full extension of its type, and every link part its filled solid
// (connectors, tubes and end spheres, from linkfill.js). Output is in mm, Y-up like the CAD files.
// Each placed part is written straight into its own chunk of the file, so the robot is never held
// in memory as one mesh; the chunks are joined into a Blob at the end.
import * as THREE from 'three';
import { state } from './robot.js';
import { forwardKinematics } from './kinematics.js';
import { hooks, CAD_UP } from './viewer.js';

/** Full-detail files for each gearbox family, in the same frames as motor.stl and gearbox.stl. */
export const FULL_FILES = {
  motor: (family) => `models/${family}_gearbox.stl`,
  extension: (family) => `models/${family}_gearbox_extension.stl`,
};

// The CAD files are Y-up: turn them Z-up to place them, and the result back to Y-up to save.
const toZUp = CAD_UP === 'y' ? new THREE.Matrix4().makeRotationX(Math.PI / 2) : new THREE.Matrix4();
const toFileUp = toZUp.clone().invert();

const sources = new Map(); // url → Promise of Float32Array (9 floats per triangle, file frame)

/** Triangle positions of a binary STL, read straight from its bytes. */
function readSTL(buf, url) {
  const dv = new DataView(buf);
  const count = dv.getUint32(80, true);
  if (84 + count * 50 !== buf.byteLength) throw new Error(`${url} is not a binary STL.`);
  const out = new Float32Array(count * 9);
  for (let t = 0, o = 96; t < count; t++, o += 50) {
    for (let k = 0; k < 9; k++) out[t * 9 + k] = dv.getFloat32(o + k * 4, true);
  }
  return out;
}

function loadSource(url) {
  if (!sources.has(url)) {
    sources.set(url, fetch(url)
      .then((r) => (r.ok ? r.arrayBuffer() : Promise.reject(new Error(`Could not load ${url} (${r.status}).`))))
      .then((buf) => readSTL(buf, url)));
    sources.get(url).catch(() => sources.delete(url)); // let a later export try again
  }
  return sources.get(url);
}

/** One placed part as STL triangle records (50 bytes each): `tri` moved by matrix `m`. */
function placed(tri, m) {
  const count = tri.length / 9;
  const buf = new ArrayBuffer(count * 50), dv = new DataView(buf);
  const e = m.elements;
  const v = new Float32Array(9);
  for (let t = 0; t < count; t++) {
    for (let k = 0; k < 3; k++) {
      const x = tri[t * 9 + k * 3], y = tri[t * 9 + k * 3 + 1], z = tri[t * 9 + k * 3 + 2];
      v[k * 3] = e[0] * x + e[4] * y + e[8] * z + e[12];
      v[k * 3 + 1] = e[1] * x + e[5] * y + e[9] * z + e[13];
      v[k * 3 + 2] = e[2] * x + e[6] * y + e[10] * z + e[14];
    }
    // Normal from the placed corners (right-hand order).
    const ux = v[3] - v[0], uy = v[4] - v[1], uz = v[5] - v[2];
    const wx = v[6] - v[0], wy = v[7] - v[1], wz = v[8] - v[2];
    let nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
    const l = Math.hypot(nx, ny, nz) || 1;
    nx /= l; ny /= l; nz /= l;
    const o = t * 50;
    dv.setFloat32(o, nx, true); dv.setFloat32(o + 4, ny, true); dv.setFloat32(o + 8, nz, true);
    for (let k = 0; k < 9; k++) dv.setFloat32(o + 12 + k * 4, v[k], true);
  }
  return buf;
}

/** What goes in the file: one entry per placed part, { label, load(), matrix }. */
function bill(frames) {
  const items = [];
  for (const n of Object.values(state.robot.nodes)) {
    const f = frames[n.id];
    if (n.type === 'actuator') {
      const [, first, ...extra] = f.local.segments;
      items.push({ label: `${n.name} (motor + ${first.family} gearbox)`, url: FULL_FILES.motor(first.family),
        matrix: f.input.clone().multiply(f.local.mount).multiply(toZUp) });
      extra.forEach((s, k) => items.push({ label: `${n.name} extension ${k + 1} (${s.family})`, url: FULL_FILES.extension(s.family),
        matrix: f.input.clone().multiply(s.start).multiply(toZUp) }));
    } else if (hooks.isPartRoot?.(n)) {
      items.push({ label: `${n.name} (link)`, link: n, matrix: f.input.clone() });
    }
  }
  return items;
}

/**
 * Build the robot's STL. onProgress({ step, done, total }) reports as it goes.
 * Returns { blob, triangles, parts }.
 */
export async function exportRobotSTL(onProgress = () => {}) {
  if (!hooks.linkFillTriangles) throw new Error('Filled links are not available, so links cannot be exported.');
  const frames = forwardKinematics(state.robot);
  const items = bill(frames);
  const urls = [...new Set(items.filter((i) => i.url).map((i) => i.url))];
  const total = urls.length + items.length + 1;
  let done = 0;
  const tick = (step) => { onProgress({ step, done: ++done, total }); return new Promise((r) => setTimeout(r)); };

  // Full models: each file once, however many times it is used.
  for (const url of urls) {
    await loadSource(url);
    await tick(`Loading ${url.split('/').pop()}`);
  }
  const chunks = [];
  let triangles = 0;
  for (const item of items) {
    const tri = item.url ? await loadSource(item.url) : await hooks.linkFillTriangles(item.link);
    const chunk = placed(tri, toFileUp.clone().multiply(item.matrix));
    chunks.push(chunk);
    triangles += tri.length / 9;
    await tick(`Placing ${item.label}`);
  }
  const header = new ArrayBuffer(84);
  const text = `Modular Robot Builder: ${state.name || 'robot'} (mm, Y-up)`.slice(0, 80);
  new Uint8Array(header).set(new TextEncoder().encode(text));
  new DataView(header).setUint32(80, triangles, true);
  const blob = new Blob([header, ...chunks], { type: 'model/stl' });
  await tick('Done');
  return { blob, triangles, parts: items.length };
}

/** Rough size of the export before building it, from the parts list (link parts not counted). */
export function exportPlan() {
  const items = bill(forwardKinematics(state.robot));
  return { motors: items.filter((i) => i.url?.includes('_gearbox.stl')).length, extensions: items.filter((i) => i.url?.includes('extension')).length,
    links: items.filter((i) => i.link).length };
}
