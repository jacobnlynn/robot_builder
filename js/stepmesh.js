// STEP → display mesh, for the CAD tab, the filled links and the URDF meshes. Uses occt-import-js
// (OpenCascade's STEP reader compiled to WebAssembly, about 8 MB), loaded from jsDelivr the first
// time a STEP file is needed. Every triangle of every body in the file comes back as one
// position-only BufferGeometry in mm, in the file's own frame.
import * as THREE from 'three';

const LIB = 'https://cdn.jsdelivr.net/npm/occt-import-js@0.0.23/dist/';
// Display quality: about 250k triangles for a full motor model (absolute mm and radians).
const DISPLAY = { linearUnit: 'millimeter', linearDeflectionType: 'absolute_value', linearDeflection: 1.0, angularDeflection: 0.8 };

let occt = null;
function loadOcct() {
  return (occt ??= new Promise((resolve, reject) => {
    const s = document.createElement('script');
    s.src = `${LIB}occt-import-js.js`;
    s.onload = () => globalThis.occtimportjs({ locateFile: (p) => LIB + p }).then(resolve, reject);
    s.onerror = () => reject(new Error('Could not load the STEP reader (occt-import-js) from jsDelivr.'));
    document.head.append(s);
  }));
}

/** Every body in a STEP file as one triangle-soup geometry (mm, file frame). */
export async function loadStepMesh(url, params = DISPLAY) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`Could not load ${url} (${res.status}).`);
  const bytes = new Uint8Array(await res.arrayBuffer());
  const lib = await loadOcct();
  const result = lib.ReadStepFile(bytes, params);
  if (!result.success) throw new Error(`Could not read ${url} as STEP.`);
  let count = 0;
  for (const m of result.meshes) count += m.index.array.length;
  const out = new Float32Array(count * 3);
  let at = 0;
  for (const m of result.meshes) {
    const p = m.attributes.position.array, idx = m.index.array;
    for (let i = 0; i < idx.length; i++, at += 3) {
      const v = idx[i] * 3;
      out[at] = p[v]; out[at + 1] = p[v + 1]; out[at + 2] = p[v + 2];
    }
  }
  return new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(out, 3));
}
