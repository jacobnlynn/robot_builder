// Robot tree + app state. Every node has one input (its parent port) and an
// array of output ports; children[i] holds the id of the part on output i.
import { DH_CONVENTION, DEFAULT_LINK_DH, MOTORS, GEARBOXES, MIN_GEARBOXES } from './constants.js';
import { dhToMatrix, matrixToPose, solveLinks, DEFAULT_ROD } from './kinematics.js';

// A link output keeps both forms; the link's `spec` ('dh' | 'pose') says which one is used.
// pose = { x, y, z, alpha, beta, gamma, ref } where ref is the frame it is measured from.
// rod = { lon, lat }: in pose mode, the direction the rod leaves the port (longitude about the
// port's z axis from +x, latitude up from its x-y plane). It shapes the rod only; the output
// frame comes from the pose.
// end: 'output' (a connector the next part attaches to) or 'sphere' (the robot ends there in a ball
// of radius `sphere` mm, centred on the output, and nothing can attach).
export const SPHERE_RADIUS = 20;
export const defaultOutput = () => ({
  dh: { ...DEFAULT_LINK_DH },
  pose: { ...matrixToPose(dhToMatrix(DEFAULT_LINK_DH)), ref: 'input' },
  rod: { ...DEFAULT_ROD },
  end: 'output',
  sphere: SPHERE_RADIUS,
});

export const state = {
  robot: null,
  name: 'Untitled robot',
  // { kind: 'node', nodeId } | { kind: 'port', nodeId, index } | null
  selection: null,
  view: 'cad', // 'cad' | 'simple' | 'sim' (simulate: drag parts to move the joints; nothing is selected)
  showFrames: true,
  units: 'mm', // display only: 'mm' | 'in'. Everything is stored in mm.
  born: {}, // nodeId → timestamp the part starts growing in
};

const listeners = new Set();
export function subscribe(fn) { listeners.add(fn); return () => listeners.delete(fn); }
// kinds: 'structure' | 'params' | 'rename' | 'selection' | 'view'
export function emit(kind) { for (const fn of listeners) fn(kind); }

let counter = 0;
const uid = (prefix) => `${prefix}${Date.now().toString(36)}${(counter++).toString(36)}`;

export const node = (id) => state.robot.nodes[id];

/**
 * The base is a link locked in place: the root of the tree, sitting on the world origin.
 * It can't be deleted, but its outputs are edited like any other link's. Starts all zero.
 */
export function baseLink(outputs = [{ dh: { a: 0, alpha: 0, d: 0, theta: 0 } }]) {
  return {
    id: 'base', type: 'link', name: 'Base', fixed: true, spec: 'dh', parent: null,
    outputs: outputs.map((o) => ({ ...o, pose: o.pose ?? { ...matrixToPose(dhToMatrix(o.dh)), ref: 'input' }, rod: o.rod ?? { ...DEFAULT_ROD } })),
    children: outputs.map(() => null),
  };
}

/** True for the locked base, which has no parent and can't be removed. */
export const isFixed = (n) => !n.parent;
/** A link output closed off with a sphere: an end of the robot, not a port. */
export const isCapped = (n, i) => n.type === 'link' && n.outputs[i]?.end === 'sphere';
/** An output something can still be attached to. */
export const isOpenPort = (n, i) => !n.children[i] && !isCapped(n, i);
const firstOpenPort = (n) => n.children.findIndex((_, i) => isOpenPort(n, i));

function emptyRobot() {
  return { rootId: 'base', nodes: { base: baseLink() } };
}

export function newRobot(name = 'Untitled robot') {
  state.robot = emptyRobot();
  state.name = name;
  state.born = { base: performance.now() };
  state.selection = { kind: 'port', nodeId: 'base', index: 0 };
  emit('structure');
}

export function nextName(type) {
  const prefix = type === 'actuator' ? 'J' : 'L';
  const used = new Set(Object.values(state.robot.nodes).map((n) => n.name));
  let i = 1;
  while (used.has(prefix + i)) i++;
  return prefix + i;
}

function attach(part, port) {
  const parent = node(port.nodeId);
  if (!parent || parent.children[port.index]) throw new Error('That port already has a part on it.');
  part.id = uid(part.type === 'actuator' ? 'a' : 'l');
  part.name = part.name?.trim() || nextName(part.type);
  part.parent = { nodeId: port.nodeId, index: port.index };
  state.robot.nodes[part.id] = part;
  parent.children[port.index] = part.id;
  state.born[part.id] = performance.now();
  // Keep building: select the new part's first open output (or the part, if it ends in spheres).
  const free = firstOpenPort(part);
  state.selection = free >= 0 ? { kind: 'port', nodeId: part.id, index: free } : { kind: 'node', nodeId: part.id };
  emit('structure');
  return part;
}

export function addActuator(port, { name, motor, gearboxes, mount = 0 }) {
  if (gearboxes.length < MIN_GEARBOXES) throw new Error('An actuator needs at least one gearbox.');
  return attach({ type: 'actuator', name, motor, gearboxes: [...gearboxes], mount, q: 0, children: [null] }, port);
}

export function addLink(port, { name, spec = 'dh', outputs }) {
  return attach({
    type: 'link',
    name,
    spec,
    outputs: outputs.map((o) => structuredClone(o)),
    children: outputs.map(() => null),
  }, port);
}

/** Ids of a part and everything attached downstream of it. */
export function subtree(id) {
  const out = [];
  const walk = (i) => { out.push(i); node(i).children.forEach((c) => c && walk(c)); };
  walk(id);
  return out;
}

/** Depth-first build order, base first. */
export function buildOrder() {
  return subtree(state.robot.rootId);
}

/**
 * Pose outputs measured from a frame that is about to disappear: re-measure them
 * from their own port, so they stay where they are.
 */
function rehome(match) {
  const solved = solveLinks(state.robot);
  for (const n of Object.values(state.robot.nodes)) {
    if (n.type !== 'link') continue;
    n.outputs.forEach((o, i) => {
      if (match(o.pose.ref)) o.pose = { ...matrixToPose(solved.links[n.id][i]), ref: 'input' };
    });
  }
}

/** Re-measure every pose output from its own port, keeping the robot's shape. */
export function measureAllFromPorts() {
  rehome((ref) => ref !== 'input');
  emit('structure');
}

export function removeNode(id) {
  const n = node(id);
  if (!n || isFixed(n)) return;
  const gone = new Set(subtree(id));
  rehome((ref) => gone.has(ref.split(':')[0]));
  for (const i of gone) { delete state.robot.nodes[i]; delete state.born[i]; }
  node(n.parent.nodeId).children[n.parent.index] = null;
  state.selection = { kind: 'port', nodeId: n.parent.nodeId, index: n.parent.index };
  emit('structure');
}

export function addLinkOutput(id) {
  const n = node(id);
  n.outputs.push(defaultOutput());
  n.children.push(null);
  emit('structure');
}

export function removeLinkOutput(id, index) {
  const n = node(id);
  if (n.outputs.length <= 1 || n.children[index]) return;
  rehome((ref) => ref === `${id}:${index}`);
  // Later outputs shift down one, so renumber anything measured from them.
  for (const m of Object.values(state.robot.nodes)) {
    for (const o of m.outputs ?? []) {
      const [rid, j] = o.pose?.ref.split(':') ?? [];
      if (rid === id && Number(j) > index) o.pose.ref = `${id}:${Number(j) - 1}`;
    }
  }
  n.outputs.splice(index, 1);
  n.children.splice(index, 1);
  n.children.forEach((c, j) => { if (c) node(c).parent.index = j; });
  emit('structure');
}

export function select(selection) {
  if (state.view === 'sim' && selection) return;
  if (selection?.kind === 'port' && isCapped(node(selection.nodeId), selection.index)) return;
  state.selection = selection;
  emit('selection');
}
export function setView(view) {
  state.view = view;
  if (view === 'sim') state.selection = null;
  emit('view');
}
export function setShowFrames(on) { state.showFrames = on; emit('view'); }
export function setUnits(units) { state.units = units; emit('units'); }

// ---- Save / load ----

export function serialize() {
  return JSON.stringify({
    format: 'modular-robot',
    version: 1,
    convention: DH_CONVENTION,
    units: { length: 'mm', angle: 'deg' },
    name: state.name,
    rootId: state.robot.rootId,
    nodes: buildOrder().map(node),
  }, null, 2);
}

export function load(source) {
  const data = structuredClone(typeof source === 'string' ? JSON.parse(source) : source);
  if (data.format !== 'modular-robot' || !Array.isArray(data.nodes)) {
    throw new Error('This is not a robot project file (expected "format": "modular-robot").');
  }
  if (data.convention && data.convention !== DH_CONVENTION) {
    throw new Error(`This project uses ${data.convention} D-H, but the builder is set to ${DH_CONVENTION}.`);
  }
  const nodes = Object.fromEntries(data.nodes.map((n) => [n.id, n]));
  const root = nodes[data.rootId];
  if (!root) throw new Error('The project has no base part.');
  if (root.type === 'base') {
    // Older files had a fixed 12 mm base plate; keep that offset as the base link's D-H d.
    const { id, name, children } = root;
    Object.assign(root, baseLink([{ dh: { a: 0, alpha: 0, d: 12, theta: 0 } }]), { id, name, children });
  }
  root.parent = null;
  root.fixed = true;
  for (const n of data.nodes) {
    if (n.type === 'actuator') {
      if (!MOTORS[n.motor]) throw new Error(`${n.name} uses an unknown motor "${n.motor}".`);
      for (const g of n.gearboxes) if (!GEARBOXES[g]) throw new Error(`${n.name} uses an unknown gearbox "${g}".`);
      if (n.gearboxes.length < MIN_GEARBOXES) throw new Error(`${n.name} has no gearbox; every actuator needs at least one.`);
      n.q ??= 0;
      n.mount ??= 0;
    }
    if (n.type === 'link') {
      n.spec ??= 'dh';
      // Older files only have D-H; give each output a matching pose to switch to.
      for (const o of n.outputs) {
        o.pose ??= { ...matrixToPose(dhToMatrix(o.dh)), ref: 'input' };
        o.pose.ref ??= 'input';
        o.rod ??= { ...DEFAULT_ROD };
        o.end ??= 'output';
        o.sphere ??= SPHERE_RADIUS;
        if ('az' in o.rod) {
          // Earlier files stored the angles to the x, y, z axes; turn them into longitude/latitude.
          const c = ['ax', 'ay', 'az'].map((k) => Math.cos((o.rod[k] ?? 90) * Math.PI / 180));
          const r = Math.hypot(...c) || 1;
          const deg = (v) => Math.round(v * 180 / Math.PI * 1000) / 1000;
          o.rod = { lon: deg(Math.atan2(c[1], c[0])), lat: deg(Math.asin(Math.max(-1, Math.min(1, c[2] / r)))) };
        }
      }
    }
    for (const c of n.children) if (c && !nodes[c]) throw new Error(`${n.name} points to a missing part "${c}".`);
  }

  state.robot = { rootId: data.rootId, nodes };
  state.name = data.name || 'Untitled robot';

  // Stagger the birth times in build order so the robot assembles on screen.
  const now = performance.now();
  state.born = {};
  buildOrder().forEach((id, k) => { state.born[id] = now + k * 240; });

  // Start on the first open port so the next step is obvious.
  state.selection = null;
  for (const id of buildOrder()) {
    const i = firstOpenPort(node(id));
    if (i >= 0) { state.selection = { kind: 'port', nodeId: id, index: i }; break; }
  }
  emit('structure');
}
