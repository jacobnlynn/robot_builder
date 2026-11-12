// Export STEP: the whole robot as one STEP assembly (and single link parts as STEP solids).
//  - Each actuator is a sub-assembly named after it, holding its full motor module (by the first
//    gearbox's type) and one extension module per extra stage, from models/*.step. Every module
//    file is written once and placed as many times as it is used, with its own component tree and
//    named bodies intact.
//  - Each printed link part is a real solid, rebuilt from linkfill.js's plan with OpenCascade
//    (opencascade.js, ~50 MB of WebAssembly, loaded from jsDelivr only when exporting): tubes,
//    collars, plates and sphere ends joined to the input/output connector solids, with the same
//    clearance cuts as the CAD tab.
// Placements are written the way Fusion 360 writes its own assemblies (a pair of axis placements per
// instance), in inches like the module files, Y-up, at the current joint angles.
import * as THREE from 'three';
import { ConvexHull } from 'three/addons/math/ConvexHull.js';
import { state } from './robot.js';
import { forwardKinematics } from './kinematics.js';
import { hooks, STEP_FILES, CAD_UP, actuatorCadKeys } from './viewer.js';

const OCC = 'https://cdn.jsdelivr.net/npm/opencascade.js@2.0.0-beta.b5ff984/dist/';
const MM_PER_IN = 25.4;
const toZUp = CAD_UP === 'y' ? new THREE.Matrix4().makeRotationX(Math.PI / 2) : new THREE.Matrix4();
const toFileUp = toZUp.clone().invert();

// ---------------------------------------------------------------- OpenCascade

let ocPromise = null;
const loadOC = () => (ocPromise ??= import(`${OCC}opencascade.full.js`)
  .then((m) => m.default({ locateFile: () => `${OCC}opencascade.full.wasm` })));

const progress = (oc) => new oc.Message_ProgressRange_1();

/** OpenCascade throws raw pointers; give the user something readable. */
function occTry(what, fn) {
  try { return fn(); } catch (e) { throw new Error(`${what} failed in the CAD kernel${typeof e === 'number' ? '' : `: ${e?.message ?? e}`}.`); }
}

function occHelpers(oc) {
  const pnt = (v) => new oc.gp_Pnt_3(v.x, v.y, v.z);
  const dir = (v) => new oc.gp_Dir_4(v.x, v.y, v.z);
  const ax2 = (p, z, x) => (x ? new oc.gp_Ax2_2(pnt(p), dir(z), dir(x)) : new oc.gp_Ax2_3(pnt(p), dir(z)));
  const trsf = (m) => {
    const e = m.elements, t = new oc.gp_Trsf_1();
    t.SetValues(e[0], e[4], e[8], e[12], e[1], e[5], e[9], e[13], e[2], e[6], e[10], e[14]);
    return t;
  };
  const moved = (shape, m) => new oc.BRepBuilderAPI_Transform_2(shape, trsf(m), true).Shape();
  const list = (shapes) => { const l = new oc.TopTools_ListOfShape_1(); shapes.forEach((s) => l.Append_1(s)); return l; };
  const solids = (shape) => {
    let k = 0;
    for (const ex = new oc.TopExp_Explorer_2(shape, oc.TopAbs_ShapeEnum.TopAbs_SOLID, oc.TopAbs_ShapeEnum.TopAbs_SHAPE); ex.More(); ex.Next()) k++;
    return k;
  };
  const boolean = (Op, args, tools, fuzz = FUZZ) => {
    const b = new Op();
    b.SetArguments(list(args));
    b.SetTools(list(tools));
    if (fuzz) b.SetFuzzyValue(fuzz);
    b.Build(progress(oc));
    if (!b.IsDone()) throw new Error('boolean');
    return b.Shape();
  };
  const volume = (shape) => {
    const g = new oc.GProp_GProps_1();
    oc.BRepGProp.VolumeProperties_1(shape, g, false, false, false);
    return g.Mass();
  };
  // How far apart two shapes are (0 when they touch, or when one sits inside the other).
  const distance = (a, b) => {
    const d = new oc.BRepExtrema_DistShapeShape_2(a, b, oc.Extrema_ExtFlag.Extrema_ExtFlag_MIN, oc.Extrema_ExtAlgo.Extrema_ExtAlgo_Grad, progress(oc));
    return !d.IsDone() || d.InnerSolution() ? 0 : d.Value();
  };
  // One join, checked: the result must hold at least the bigger of the two and at most both, and
  // pieces that overlap must merge into one solid. Faces that lie exactly on top of each other (tubes
  // leaving one motor along the same axis) can defeat the kernel, so a failed join is retried with
  // the piece nudged by a hundredth of a millimetre, then with a looser tolerance.
  const NUDGES = [[0, 0, 0], [0.01, 0.007, 0.004], [-0.006, 0.01, -0.008]];
  const join = (a, s, fuzzyFirst) => {
    const va = volume(a), vs = volume(s), na = solids(a);
    // Tubes join best exactly (a tolerance can make the kernel drop overlap: the humanoid's elbow lost
    // a fifth of its volume); the connectors, read from STEP files, need one (exactly, they can hang).
    const exact = NUDGES.map((d) => [d, 0]), fuzzy = NUDGES.map((d) => [d, FUZZ]);
    const tries = [...(fuzzyFirst ? [...fuzzy, ...exact] : [...exact, ...fuzzy]), [[0, 0, 0], 0.01]];
    for (const [[dx, dy, dz], fuzz] of tries) {
      try {
        const piece = dx || dy || dz ? moved(s, new THREE.Matrix4().makeTranslation(dx, dy, dz)) : s;
        const r = boolean(oc.BRepAlgoAPI_Fuse_1, [a], [piece], fuzz), v = volume(r), n = solids(r);
        const overlaps = v < (va + vs) * 0.999;
        // A piece that seems to land apart from the rest must really not touch it: the kernel can
        // miss a ball sitting inside another ball and keep the two as separate solids.
        const apart = !overlaps && n > na && distance(a, piece) > 1e-3;
        if (n && v >= Math.max(va, vs) * 0.999 && v <= (va + vs) * 1.001 && (n <= na || apart)) return r;
      } catch { /* try the next way */ }
    }
    return null;
  };
  // Joined one at a time: given many touching shapes at once, the kernel can quietly return a
  // fraction of them (the hexapod body lost its tubes) or nothing (square tubes). A piece that
  // won't join yet is tried again once the rest are on; one that never joins is an error.
  const fuse = (shapes, fuzzyFirst = false) => {
    shapes = shapes.filter(Boolean);
    if (!shapes.length) return null;
    let out = shapes[0], todo = shapes.slice(1);
    while (todo.length) {
      const failed = [];
      for (const s of todo) { const r = join(out, s, fuzzyFirst); if (r) out = r; else failed.push(s); }
      if (failed.length === todo.length) throw new Error(`${failed.length} of ${shapes.length} pieces could not be joined`);
      todo = failed;
    }
    return out;
  };
  const cut = (body, tools) => {
    if (!body || !tools.length) return body;
    const out = boolean(oc.BRepAlgoAPI_Cut_1, [body], tools);
    if (!solids(out)) throw new Error('the result was empty');
    // Drop the specks the cut can leave (solids of almost no volume where a cut grazes a face).
    const keep = [];
    for (const ex = new oc.TopExp_Explorer_2(out, oc.TopAbs_ShapeEnum.TopAbs_SOLID, oc.TopAbs_ShapeEnum.TopAbs_SHAPE); ex.More(); ex.Next()) {
      if (volume(ex.Current()) >= 1) keep.push(ex.Current());
    }
    if (keep.length === solids(out) || !keep.length) return out;
    const bb = new oc.BRep_Builder(), compound = new oc.TopoDS_Compound();
    bb.MakeCompound(compound);
    keep.forEach((k) => bb.Add(compound, k));
    return compound;
  };
  return { pnt, dir, ax2, trsf, moved, fuse, cut, seen: new Set() };
}

// Connector solids, read once from their STEP files (mm, turned Z-up).
const connectorShapes = new Map();
async function connectorShape(oc, key) {
  if (!connectorShapes.has(key)) {
    const text = await (await fetch(STEP_FILES[key])).text();
    oc.FS.writeFile(`/${key}.step`, text);
    const r = new oc.STEPControl_Reader_1();
    r.ReadFile(`/${key}.step`);
    r.TransferRoots(progress(oc));
    connectorShapes.set(key, occHelpers(oc).moved(r.OneShape(), toZUp));
  }
  return connectorShapes.get(key);
}

/** A convex hull of points as a closed OpenCascade solid (its triangles sewn into a shell). */
function hullSolid(oc, h, pts) {
  const hull = new ConvexHull().setFromPoints(pts.map((p) => new THREE.Vector3(...p)));
  const sew = new oc.BRepBuilderAPI_Sewing(1e-6, true, true, true, false);
  for (const f of hull.faces) {
    const poly = new oc.BRepBuilderAPI_MakePolygon_1();
    let e = f.edge;
    do { poly.Add_1(h.pnt(e.head().point)); e = e.next; } while (e !== f.edge);
    poly.Close();
    sew.Add(new oc.BRepBuilderAPI_MakeFace_15(poly.Wire(), true).Face());
  }
  sew.Perform(progress(oc));
  const solid = new oc.BRepBuilderAPI_MakeSolid_3(oc.TopoDS.Shell_1(sew.SewedShape())).Solid();
  // The sewn faces can come out facing inwards, which makes an inside-out solid (negative volume)
  // that the booleans then subtract instead of add.
  const g = new oc.GProp_GProps_1();
  oc.BRepGProp.VolumeProperties_1(solid, g, false, false, false);
  if (g.Mass() < 0) solid.Reverse();
  return solid;
}

// Cubes are made this much bigger than the square tube they cap, so their faces never lie exactly
// on the tube's: the kernel's booleans can fail on faces that coincide.
const GROW = 0.02; // mm
const SGROW = 0.1; // mm: balls, so they cut through a round tube instead of touching it tangentially
// A join's tolerance (gaps this small count as touching): first choice for the connectors, a
// fallback for the tubes (see join in occHelpers).
const FUZZ = 0.001; // mm

/** One plan item (see linkfill.js partPlan) as an OpenCascade solid, or a list of them. */
function toSolid(oc, h, p, connectors) {
  const { pnt, ax2, moved } = h;
  // The same ball or cube can come up twice (a collar's tip and the tube's end): make it once.
  const once = (key, make) => {
    if (h.seen.has(key)) return [];
    h.seen.add(key);
    return [make()];
  };
  const key = (t, c, r) => `${t}:${c.x.toFixed(4)},${c.y.toFixed(4)},${c.z.toFixed(4)},${r}`;
  const sphere = (c, r) => once(key('sphere', c, r), () => new oc.BRepPrimAPI_MakeSphere_5(pnt(c), r + SGROW).Shape());
  const cylinder = (base, along, r, len) => new oc.BRepPrimAPI_MakeCylinder_3(ax2(base, along), r, len).Shape();
  const box = (origin, x, z, dx, dy, dz) => new oc.BRepPrimAPI_MakeBox_5(ax2(origin, z, x), dx, dy, dz).Shape();
  // A tube of no length (a passive part whose ball sits where it starts) has no direction: the CAD
  // tab leaves its cube out, and so does this.
  const cube = (c, basis, r0) => (basis.some((e) => !(e.lengthSq() > 1e-12)) ? [] : once(key('cube', c, r0), () => {
    const r = r0 + GROW;
    const [x, , z] = basis, y = new THREE.Vector3().crossVectors(z, x);
    return box(c.clone().addScaledVector(x, -r).addScaledVector(y, -r).addScaledVector(z, -r), x, z, 2 * r, 2 * r, 2 * r);
  }));
  const atFrame = (frame, z) => new THREE.Vector3(0, 0, z).applyMatrix4(frame);
  const frameZ = (frame) => new THREE.Vector3().setFromMatrixColumn(frame, 2).normalize();
  switch (p.t) {
    case 'ball': case 'sphere': return sphere(p.c, p.r);
    case 'cube': return cube(p.c, p.basis, p.r);
    case 'capsule': {
      const v = p.b.clone().sub(p.a), len = v.length();
      return [cylinder(p.a, v.normalize(), p.r, len), ...sphere(p.a, p.r), ...sphere(p.b, p.r)];
    }
    case 'bar': {
      const along = p.b.clone().sub(p.a), len = along.length();
      along.normalize();
      const [u] = p.basis.filter((e) => Math.abs(e.dot(along)) < 0.5);
      const y = new THREE.Vector3().crossVectors(along, u);
      return [box(p.a.clone().addScaledVector(u, -p.r).addScaledVector(y, -p.r), u, along, 2 * p.r, 2 * p.r, len)];
    }
    case 'disc': return [cylinder(atFrame(p.frame, p.z0), frameZ(p.frame), p.r, p.z1 - p.z0)];
    case 'collar': {
      // An exact disc (the face that meets the connector stays a true circle), and the taper from it
      // to the tip as the convex hull of points just inside the disc's rims and the tip. Kept a
      // twentieth of a millimetre inside, the hull never lies on the disc's faces: the kernel can hang
      // on a polygon lying almost exactly on a circle.
      const disc = cylinder(atFrame(p.frame, p.z0), frameZ(p.frame), p.r, p.z1 - p.z0);
      const IN = 0.05, ri = p.r - IN;
      const ring = (z) => Array.from({ length: 64 }, (_, k) => {
        const a = (k / 64) * Math.PI * 2;
        return new THREE.Vector3(ri * Math.cos(a), ri * Math.sin(a), z).applyMatrix4(p.frame).toArray();
      });
      const rims = [...ring(p.z0 + IN), ...ring(p.z1 - IN)];
      if (p.tip.t === 'ball') {
        // The convex hull of the disc and the ball, as the CAD tab makes it (a cone aimed at the
        // ball's centre can miss the tube entirely when the tube leaves at an angle), plus the
        // ball itself so its outside stays round.
        const pts = [...rims];
        for (let i = 1; i < 12; i++) for (let j = 0; j < 24; j++) {
          const th = (i / 12) * Math.PI, ph = (j / 24) * Math.PI * 2;
          pts.push([p.tip.c.x + p.tip.r * Math.sin(th) * Math.cos(ph), p.tip.c.y + p.tip.r * Math.sin(th) * Math.sin(ph), p.tip.c.z + p.tip.r * Math.cos(th)]);
        }
        pts.push([p.tip.c.x, p.tip.c.y, p.tip.c.z + p.tip.r], [p.tip.c.x, p.tip.c.y, p.tip.c.z - p.tip.r]);
        return [disc, hullSolid(oc, h, pts), ...sphere(p.tip.c, p.tip.r)];
      }
      // Square tip: the convex hull of the disc and the cube, exactly as the CAD tab makes it. (A loft
      // from the circle to the cube's near face came out smaller, and the kernel often failed to join it.)
      h.seen.add(key('cube', p.tip.c, p.tip.r)); // inside the hull: the tube's own end cube is not needed
      const r = p.tip.r + GROW, [bx, by, bz] = p.tip.basis, corners = [];
      for (const a of [-r, r]) for (const b of [-r, r]) for (const d of [-r, r]) {
        corners.push(p.tip.c.clone().addScaledVector(bx, a).addScaledVector(by, b).addScaledVector(bz, d).toArray());
      }
      return [disc, hullSolid(oc, h, [...rims, ...corners])];
    }
    case 'hull':
      // The motor's real envelope, as in the CAD tab. (A bounding box is far bigger than an L-shaped
      // motor and cut away the tubes of a part with motors bolted all round it, like the hexapod body.)
      return [moved(hullSolid(oc, h, p.pts), p.frame)];
    // A wheel at the end of the part: its whole disc (hub, rim and tyre) as one solid.
    case 'wheel': return [cylinder(atFrame(p.m, -p.w / 2), frameZ(p.m), p.r, p.w)];
    case 'input': return [moved(connectors.input, p.m)];
    case 'output': return [moved(connectors.output, p.m)];
    default: throw new Error(`Unknown piece ${p.t}`);
  }
}

/** A link part's plan as one solid, Y-up (like the module files), in mm. */
async function linkSolid(oc, plan) {
  const h = occHelpers(oc);
  const connectors = { input: await connectorShape(oc, 'input'), output: await connectorShape(oc, 'output') };
  // Every tube of a part brings its own copy of the input collar; the kernel's booleans break on
  // exact duplicates (the hexapod body lost most of its volume), so each distinct item is made once.
  const r4 = (_, v) => (typeof v === 'number' ? Math.round(v * 1e4) / 1e4 : v);
  const make = (items) => {
    h.seen.clear();
    const keys = new Set();
    return items.filter((p) => { const k = JSON.stringify(p, r4); return !keys.has(k) && keys.add(k); })
      .flatMap((p) => toSolid(oc, h, p, connectors));
  };
  let body = occTry('Joining the tubes', () => h.fuse(make(plan.pieces)));
  body = occTry('Cutting clearance around the motors', () => h.cut(body, plan.cuts.length ? [h.fuse(make(plan.cuts))] : []));
  body = occTry('Joining the connectors', () => h.fuse([body, ...make(plan.connectors)], true));
  if (!body) throw new Error('This part has nothing to export.');
  // (No ShapeUpgrade_UnifySameDomain tidy-up: merging faces could hang the kernel on some parts.)
  return h.moved(body, toFileUp);
}

/** A shape as STEP text, in inches. */
function writeStep(oc, shape, name) {
  oc.Interface_Static.SetCVal('write.step.unit', 'INCH');
  oc.Interface_Static.SetCVal('write.step.schema', 'AP214IS');
  oc.Interface_Static.SetCVal('write.step.product.name', name);
  const w = new oc.STEPControl_Writer_1();
  occTry('Writing STEP', () => w.Transfer(shape, oc.STEPControl_StepModelType.STEPControl_AsIs, true, progress(oc)));
  w.Write('/part.step');
  return oc.FS.readFile('/part.step', { encoding: 'utf8' });
}

// ---------------------------------------------------------------- STEP text: parse, renumber, assemble

/** The DATA section's entities: [{ id, body }], the body is everything after '=' up to ';'. */
function parseEntities(text) {
  const start = text.indexOf('DATA;') + 5;
  const out = [];
  let i = start;
  const endsec = text.indexOf('ENDSEC;', start), n = endsec >= 0 ? endsec : text.length;
  while (i < n) {
    const hash = text.indexOf('#', i);
    if (hash < 0 || hash >= n) break;
    let j = hash + 1;
    while (text.charCodeAt(j) >= 48 && text.charCodeAt(j) <= 57) j++;
    const id = Number(text.slice(hash + 1, j));
    j = text.indexOf('=', j) + 1;
    let k = j, inStr = false;
    for (; k < n; k++) {
      const c = text[k];
      if (c === "'") inStr = !inStr; // '' inside a string toggles twice, which is fine
      else if (c === ';' && !inStr) break;
    }
    out.push({ id, body: text.slice(j, k).trim() });
    i = k + 1;
  }
  return out;
}

/** References in an entity body (outside strings). */
function refs(body) {
  const out = [];
  let inStr = false;
  for (let i = 0; i < body.length; i++) {
    const c = body[i];
    if (c === "'") inStr = !inStr;
    else if (c === '#' && !inStr) {
      let j = i + 1;
      while (body.charCodeAt(j) >= 48 && body.charCodeAt(j) <= 57) j++;
      out.push(Number(body.slice(i + 1, j)));
      i = j - 1;
    }
  }
  return out;
}

/** The same body with every reference shifted by `offset`. */
function shift(body, offset) {
  let out = '', inStr = false, last = 0;
  for (let i = 0; i < body.length; i++) {
    const c = body[i];
    if (c === "'") inStr = !inStr;
    else if (c === '#' && !inStr) {
      let j = i + 1;
      while (body.charCodeAt(j) >= 48 && body.charCodeAt(j) <= 57) j++;
      out += body.slice(last, i + 1) + (Number(body.slice(i + 1, j)) + offset);
      last = j;
      i = j - 1;
    }
  }
  return out + body.slice(last);
}

const kind = (body) => body.replace(/^\(\s*/, '').match(/^[A-Z_0-9]+/)?.[0];
const q = (s) => `'${String(s).replace(/'/g, "''")}'`;
const num = (v) => { const s = Number(v.toFixed(10)).toString(); return s.includes('.') || s.includes('e') ? s : `${s}.`; };

/** Builds one STEP file out of generated entities and whole source files. */
class StepWriter {
  constructor() { this.lines = []; this.next = 1; }
  add(body) { const id = this.next++; this.lines.push([id, body]); return id; }
  reserve() { return this.next++; }
  set(id, body) { this.lines.push([id, body]); }

  /**
   * Copy a whole STEP file in, renumbered. Its top product is renamed `name`, and its top shape
   * representation gets an identity placement for assemblies to align with.
   * Returns { pd, sr, origin } (the root product definition, its shape representation, that placement).
   */
  include(text, name) {
    const ents = parseEntities(text);
    const byId = new Map(ents.map((e) => [e.id, e.body]));
    let minId = Infinity;
    for (const e of ents) if (e.id < minId) minId = e.id;
    const offset = this.next - minId; // every id lands at or after this.next
    // Root product definition: never the child of an assembly link.
    const pds = ents.filter((e) => kind(e.body) === 'PRODUCT_DEFINITION');
    const children = new Set(ents.filter((e) => kind(e.body) === 'NEXT_ASSEMBLY_USAGE_OCCURRENCE').map((e) => refs(e.body)[1]));
    const root = pds.find((e) => !children.has(e.id)) ?? pds[0];
    const pdsEnt = ents.find((e) => kind(e.body) === 'PRODUCT_DEFINITION_SHAPE' && refs(e.body).at(-1) === root.id);
    const sdr = ents.find((e) => kind(e.body) === 'SHAPE_DEFINITION_REPRESENTATION' && refs(e.body)[0] === pdsEnt.id);
    const srId = refs(sdr.body)[1];
    const pdf = refs(root.body)[0];
    const productId = refs(byId.get(pdf)).find((r) => kind(byId.get(r) ?? '') === 'PRODUCT');
    let maxId = 0;
    for (const e of ents) {
      let body = e.body;
      if (e.id === productId) body = body.replace(/^PRODUCT\('(?:[^']|'')*','(?:[^']|'')*'/, `PRODUCT(${q(name)},${q(name)}`);
      this.lines.push([e.id + offset, shift(body, offset)]);
      maxId = Math.max(maxId, e.id + offset);
    }
    this.next = maxId + 1;
    // Identity placement inside the root shape representation.
    const o = this.axis(new THREE.Matrix4());
    const sr = srId + offset;
    const at = this.lines.findIndex(([id]) => id === sr);
    this.lines[at][1] = this.lines[at][1].replace(/\(\s*'((?:[^']|'')*)'\s*,\s*\((\s*\))?/, (m, _name, empty) => (empty ? `${m.slice(0, -empty.length)}#${o})` : `${m}#${o},`));
    return { pd: root.id + offset, sr, origin: o };
  }

  /** AXIS2_PLACEMENT_3D for a rigid transform (translation mm → inches). */
  axis(m) {
    const e = m.elements;
    const p = this.add(`CARTESIAN_POINT('',(${num(e[12] / MM_PER_IN)},${num(e[13] / MM_PER_IN)},${num(e[14] / MM_PER_IN)}))`);
    const z = this.add(`DIRECTION('',(${num(e[8])},${num(e[9])},${num(e[10])}))`);
    const x = this.add(`DIRECTION('',(${num(e[0])},${num(e[1])},${num(e[2])}))`);
    return this.add(`AXIS2_PLACEMENT_3D('',#${p},#${z},#${x})`);
  }

  text(fileName) {
    const body = this.lines.sort((a, b) => a[0] - b[0]).map(([id, b]) => `#${id}=${b};`).join('\n');
    return `ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Reconfigurable Robot Builder assembly'),'2;1');
FILE_NAME(${q(fileName)},${q(new Date().toISOString())},(''),(''),'Reconfigurable Robot Builder','Reconfigurable Robot Builder','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 }'));
ENDSEC;
DATA;
${body}
ENDSEC;
END-ISO-10303-21;
`;
  }
}

/** New (sub-)assembly products, in inches, that other products can be placed into. */
function assemblyContext(w) {
  const app = w.add("APPLICATION_CONTEXT('automotive_design')");
  w.add(`APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2001,#${app})`);
  const pc = w.add(`PRODUCT_CONTEXT('',#${app},'mechanical')`);
  const pdc = w.add(`PRODUCT_DEFINITION_CONTEXT('part definition',#${app},'design')`);
  const mm = w.add('(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))');
  const dims = w.add('DIMENSIONAL_EXPONENTS(1.,0.,0.,0.,0.,0.,0.)');
  const inchMeasure = w.add(`LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#${mm})`);
  const inch = w.add(`(CONVERSION_BASED_UNIT('inch',#${inchMeasure})LENGTH_UNIT()NAMED_UNIT(#${dims}))`);
  const rad = w.add('(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))');
  const sr = w.add('(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())');
  const unc = w.add(`UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.0003937),#${inch},'distance_accuracy_value','')`);
  const ctx = w.add(`(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#${unc}))GLOBAL_UNIT_ASSIGNED_CONTEXT((#${inch},#${rad},#${sr}))REPRESENTATION_CONTEXT('',''))`);
  const products = [];
  return {
    /** A new product with an empty shape; place children into it with place(). */
    product(name) {
      const p = w.add(`PRODUCT(${q(name)},${q(name)},'',(#${pc}))`);
      w.add(`PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#${p}))`);
      const f = w.add(`PRODUCT_DEFINITION_FORMATION('','',#${p})`);
      const pd = w.add(`PRODUCT_DEFINITION('design','',#${f},#${pdc})`);
      const pds = w.add(`PRODUCT_DEFINITION_SHAPE('','',#${pd})`);
      const srId = w.reserve();
      w.add(`SHAPE_DEFINITION_REPRESENTATION(#${pds},#${srId})`);
      const item = { pd, sr: srId, items: [w.axis(new THREE.Matrix4())] };
      item.origin = item.items[0];
      products.push(item);
      return item;
    },
    /** Place `child` ({ pd, sr, origin }) into `parent` at transform m (mm), as an instance called `name`. */
    place(parent, child, m, name) {
      const ax = w.axis(m);
      parent.items.push(ax);
      const nauo = w.add(`NEXT_ASSEMBLY_USAGE_OCCURRENCE(${q(name)},${q(name)},'',#${parent.pd},#${child.pd},$)`);
      const pdsn = w.add(`PRODUCT_DEFINITION_SHAPE('','',#${nauo})`);
      const idt = w.add(`ITEM_DEFINED_TRANSFORMATION('','',#${child.origin},#${ax})`);
      const rr = w.add(`(REPRESENTATION_RELATIONSHIP('','',#${child.sr},#${parent.sr})REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#${idt})SHAPE_REPRESENTATION_RELATIONSHIP())`);
      w.add(`CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#${rr},#${pdsn})`);
    },
    /** Write out every product's shape representation (after all placements). */
    finish() { for (const p of products) w.set(p.sr, `SHAPE_REPRESENTATION('',(${p.items.map((i) => `#${i}`).join(',')}),#${ctx})`); },
  };
}

// ---------------------------------------------------------------- Exports

const MODULE_NAME = { motor: (f) => `${f} actuator (motor + ${f} gearbox)`, extension: (f) => `${f} gearbox extension` };

/** One link part as STEP text (a single solid, inches, Y-up). */
export async function exportLinkSTEP(n, onProgress = () => {}) {
  onProgress({ step: 'Loading the CAD kernel', done: 0, total: 3 });
  const oc = await loadOC();
  onProgress({ step: 'Building the part', done: 1, total: 3 });
  const part = await hooks.linkPartPlan(n);
  const name = part.links.map((x) => x.name).join(' + ');
  const shape = await linkSolid(oc, part.plan);
  onProgress({ step: 'Writing STEP', done: 2, total: 3 });
  const text = writeStep(oc, shape, name);
  onProgress({ step: 'Done', done: 3, total: 3 });
  return { blob: new Blob([text], { type: 'application/step' }), name };
}

/**
 * The whole robot as a STEP assembly at its current pose. onProgress({ step, done, total }).
 * Returns { blob, actuators, links }.
 */
export async function exportRobotSTEP(onProgress = () => {}) {
  if (!hooks.linkPartPlan) throw new Error('Filled links are not available, so links cannot be exported.');
  const frames = forwardKinematics(state.robot);
  const nodes = Object.values(state.robot.nodes);
  const actuators = nodes.filter((n) => n.type === 'actuator');
  const linkParts = nodes.filter((n) => hooks.isPartRoot?.(n));
  const moduleKeys = [...new Set(actuators.flatMap(actuatorCadKeys))];
  const total = moduleKeys.length + linkParts.length + 3;
  let done = 0;
  const tick = (step) => { onProgress({ step, done: ++done, total }); return new Promise((r) => setTimeout(r)); };

  const w = new StepWriter();
  const asm = assemblyContext(w);
  const robot = asm.product(state.name || 'robot');

  // Module files, each read and included once.
  const modules = new Map();
  for (const key of moduleKeys) {
    const [k, family] = key.split(':');
    const url = STEP_FILES[k](family);
    const res = await fetch(url);
    if (!res.ok) throw new Error(`Could not load ${url} (${res.status}).`);
    modules.set(key, w.include(await res.text(), MODULE_NAME[k](family)));
    await tick(`Adding ${url.split('/').pop()}`);
  }

  // Actuators: a sub-assembly each, placed at its input; the modules inside it at their own frames.
  for (const n of actuators) {
    const f = frames[n.id];
    const sub = asm.product(n.name);
    const [motorKey, ...stageKeys] = actuatorCadKeys(n);
    asm.place(sub, modules.get(motorKey), f.local.mount.clone().multiply(toZUp), `${n.name} motor`);
    f.local.segments.slice(2).forEach((s, i) => asm.place(sub, modules.get(stageKeys[i]), s.start.clone().multiply(toZUp), `${n.name} extension ${i + 1}`));
    asm.place(robot, sub, toFileUp.clone().multiply(f.input), n.name);
  }
  await tick('Placing the actuators');

  // Link parts: solids from the CAD kernel.
  if (linkParts.length) {
    onProgress({ step: 'Loading the CAD kernel (first time only, about 50 MB)', done, total });
    const oc = await loadOC();
    for (const n of linkParts) {
      const part = await hooks.linkPartPlan(n);
      const name = part.links.map((x) => x.name).join(' + ');
      const solid = await linkSolid(oc, part.plan);
      const inc = w.include(writeStep(oc, solid, name), name);
      asm.place(robot, inc, toFileUp.clone().multiply(frames[n.id].input).multiply(toZUp), name);
      await tick(`Building ${name}`);
    }
  }
  asm.finish();
  await tick('Writing the file');
  const text = w.text(`${state.name || 'robot'}.step`);
  await tick('Done');
  return { blob: new Blob([text], { type: 'application/step' }), actuators: actuators.length, links: linkParts.length };
}

