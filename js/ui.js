import * as store from './robot.js';
import { state, node } from './robot.js';
import {
  MOTORS, GEARBOXES, FAMILY_LABELS, FAMILY_COLORS, MAX_GEARBOXES, MIN_GEARBOXES, JOINT_LIMIT, PASSIVE,
  DH_CONVENTION, MM_PER_IN,
} from './constants.js';
import {
  forwardKinematics, solveLinks, stackRatio, formatRatio, actuatorReach,
  dhToMatrix, matrixToDH, poseToMatrix, homeFrame, poseFromLocal, localKinematics, loopUnknowns,
} from './kinematics.js';
import { cadModelStatus, hooks } from './viewer.js';
import { initSim, jointSpeed } from './simulate.js';
import { computeWorkspaces, workspaceMesh } from './workspace.js';
import { exportRobotSTEP, exportLinkSTEP } from './stepexport.js';
import { exportURDF } from './urdf.js';
import * as THREE from 'three';

const $ = (s, root = document) => root.querySelector(s);
const $$ = (s, root = document) => [...root.querySelectorAll(s)];
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const hex = (n) => '#' + n.toString(16).padStart(6, '0');
const slug = (s) => (s || 'robot').trim().toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '') || 'robot';

/** Fixed-point without "-0.0". */
function num(v, digits) {
  const r = Number(v.toFixed(digits));
  return (Object.is(r, -0) ? 0 : r).toFixed(digits);
}

// ---- Length units: stored in mm, shown in state.units ----
const UNITS = { mm: { scale: 1, digits: 1, field: 3, step: 1 }, in: { scale: MM_PER_IN, digits: 3, field: 4, step: 0.05 } };
const unit = () => UNITS[state.units];
const toUnit = (mm) => mm / unit().scale;
const fromUnit = (v) => v * unit().scale;
/** "56.5 mm" / "2.225 in" */
const len = (mm, digits = unit().digits) => `${num(toUnit(mm), digits)} ${state.units}`;
/** Value for an editable length field, without trailing zeros. */
const lenField = (mm) => String(Number(toUnit(mm).toFixed(unit().field)));

// D-H fields in the order the convention applies them. 'len' fields follow the unit setting.
const DH_FIELDS = {
  standard: [['theta', 'θ', '°'], ['d', 'd', 'len'], ['a', 'a', 'len'], ['alpha', 'α', '°']],
  modified: [['alpha', 'α', '°'], ['a', 'a', 'len'], ['theta', 'θ', '°'], ['d', 'd', 'len']],
};
const CONVENTION_FORMULA = {
  standard: 'Rz(θ) · Tz(d) · Tx(a) · Rx(α)',
  modified: 'Rx(α) · Tx(a) · Rz(θ) · Tz(d)',
};
const CONVENTION_SHORT = { standard: 'Standard D-H', modified: 'Modified D-H' };
const POSE_FIELDS = [['x', 'x', 'len'], ['y', 'y', 'len'], ['z', 'z', 'len'], ['alpha', 'α', '°'], ['beta', 'β', '°'], ['gamma', 'γ', '°']];
const LENGTH_KEYS = new Set(['a', 'd', 'x', 'y', 'z']);
const ROD_FIELDS = [['lon', 'Longitude', '°'], ['lat', 'Latitude', '°']];
const POSE_FORMULA = 'Trans(x, y, z) · Rz(γ) · Ry(β) · Rx(α)';
const PORT_NAME = { ball: 'Open ball joint', hinge: 'Open hinge' }; // Assembly list rows for open passive joints
const TYPE_LABEL = { base: 'Base', actuator: 'Actuator', link: 'Link' };
const CHIP = { base: ['B', '#7b8592'], link: ['L', '#8a97a8'] };

const freshActuator = () => ({ name: '', motor: Object.keys(MOTORS)[0], gearboxes: ['planetary_5'], mount: 0 });
const freshLink = () => ({ name: '', spec: 'dh', outputs: [store.defaultOutput()] });

const ui = {
  viewer: null,
  frames: {},
  solved: null, // solveLinks result: link transforms and home-pose frames
  addTab: 'actuator',
  drafts: { actuator: freshActuator(), link: freshLink() },
  draftPort: null, // 'nodeId:index' the drafts were set up for; a different port starts them fresh
  draftRobot: null,
  armedDelete: null,
  collapsed: new Set(), // ids of parts whose downstream parts are hidden in the assembly
  search: '', // Assembly search box: when set, the list shows only the parts whose names match
  treeView: false, // the tree window (box-and-line diagram of the assembly) is open
  formulaOpen: new Set(), // Robot data: end effectors whose Jacobian formula is unfolded
  formulaCode: {}, // Robot data: Python for each end effector's Jacobian, for the Copy button
  // Simulate-mode workspaces. Computed on first use and kept until the robot's parts change.
  ws: { sig: null, data: null, meshes: {}, show: { reach: false, dex: false }, busy: null, progress: 0 },
};

function resetDraftsForPort() {
  const sel = state.selection;
  if (sel?.kind !== 'port') return;
  const key = `${sel.nodeId}:${sel.index}`;
  if (key === ui.draftPort && state.robot === ui.draftRobot) return;
  ui.drafts = { actuator: freshActuator(), link: freshLink() };
  ui.draftPort = key;
  ui.draftRobot = state.robot;
}

/** Which kind of part the selected port is set up to take: only a link fits a ball or hinge joint. */
const addTabFor = (sel) => (sel?.kind === 'port' && store.isPassive(node(sel.nodeId), sel.index) ? 'link' : ui.addTab);

/** The part being set up at the selected port, for the viewer's live preview. */
function draftPreview() {
  const sel = state.selection;
  if (sel?.kind !== 'port' || !ui.frames[sel.nodeId]) return null;
  const tab = addTabFor(sel);
  const d = ui.drafts[tab];
  const input = ui.frames[sel.nodeId].outputs[sel.index];
  if (tab === 'actuator') {
    const n = { type: 'actuator', motor: d.motor, gearboxes: d.gearboxes, mount: d.mount, q: 0, children: [null] };
    return { node: n, local: localKinematics(n), input };
  }
  const n = { type: 'link', spec: d.spec, outputs: d.outputs, children: d.outputs.map(() => null) };
  const home = linkInputHome(null);
  return { node: n, local: localKinematics(n, d.outputs.map((_, i) => outputLocal(d, i, home))), input };
}

const refreshViewer = () => ui.viewer.update(state, ui.frames, draftPreview());

export function initUI(viewer) {
  ui.viewer = viewer;
  store.subscribe(onChange);
  bindTopbar();
  bindTree();
  bindInspector();
  bindJoints();
  bindFileDialog();
  bindDataDialog();
  bindExportDialog();
  bindKeys();
  initSim(viewer, {
    // While dragging, only the sliders follow; everything else catches up when the drag ends.
    onLive: () => {
      for (const n of Object.values(state.robot.nodes)) {
        if (n.type !== 'actuator') continue;
        $$(`[data-q="${n.id}"]`).forEach((el) => { el.value = n.q; });
        $$(`[data-q-out="${n.id}"]`).forEach((el) => { el.textContent = `${Math.round(n.q)}°`; });
      }
    },
    onEnd: () => store.emit('params'),
  });
}

function onChange(kind) {
  ui.solved = solveLinks(state.robot);
  ui.frames = forwardKinematics(state.robot, ui.solved);
  resetDraftsForPort();
  refreshViewer();
  if (kind === 'selection' || kind === 'structure') revealSelection();
  if (kind !== 'params' || !document.activeElement?.matches?.('#tree [data-q], #tree [data-pj]')) renderTree();
  else syncPassiveSliders();
  renderStatus();
  renderViewChrome();
  if (kind === 'structure' || kind === 'selection' || kind === 'units' || kind === 'view') {
    ui.armedDelete = null;
    renderInspector();
  } else {
    renderMatrices();
  }
  renderRobotData(); // live while the window is open
  syncWorkspaces();
}

// ---------------------------------------------------------------- Top bar

function bindTopbar() {
  $('#tube-toggle').addEventListener('click', (e) => {
    const b = e.target.closest('[data-tube]');
    if (b && b.dataset.tube !== hooks.tubeShape) hooks.setTubeShape(b.dataset.tube);
  });
  $$('[data-view]').forEach((b) => b.addEventListener('click', () => store.setView(b.dataset.view)));
  // The units toggle lives in the status bar, which is re-rendered, so listen on the bar itself.
  $('#status').addEventListener('click', (e) => {
    const b = e.target.closest('[data-units]');
    if (b) store.setUnits(b.dataset.units);
  });
  $('#project-name').addEventListener('input', (e) => { state.name = e.target.value; });

  // New: a blank robot. With parts on the robot, the first click asks, and a second within 3 s discards it.
  const btn = $('#btn-new');
  let timer = null;
  const disarm = () => { clearTimeout(timer); timer = null; btn.textContent = 'New'; btn.classList.remove('is-armed'); };
  btn.addEventListener('click', () => {
    const hasParts = Object.keys(state.robot.nodes).length > 1;
    if (hasParts && !timer) {
      btn.textContent = 'Discard robot?';
      btn.classList.add('is-armed');
      timer = setTimeout(disarm, 3000);
      return;
    }
    disarm();
    store.newRobot();
    ui.viewer.fit();
  });
}

function renderViewChrome() {
  $$('[data-view]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.view === state.view)));
  $('#vp-hint').innerHTML = state.view === 'sim'
    ? 'Drag a part to move the joints &middot; Drag empty space to orbit<br>Right-drag to pan &middot; Scroll to zoom'
    : state.view === 'anim' ? 'Drag to orbit &middot; Right-drag to pan &middot; Scroll to zoom<br>Build and play an animation in the panel on the right'
    : 'Drag to orbit &middot; Right-drag to pan &middot; Scroll to zoom<br>Click a blue dot to attach a part';
  const name = $('#project-name');
  if (document.activeElement !== name) name.value = state.name;

  const item = (cls, color, label) => `<span class="legend-item"><span class="swatch ${cls}" style="--c:${color}"></span>${label}</span>`;
  // Round or square tubes for the filled links (CAD tab, when the link fill trial is loaded).
  const tubes = $('#tube-toggle');
  tubes.hidden = state.view !== 'cadfile' || !hooks.setTubeShape;
  if (!tubes.hidden) {
    tubes.innerHTML = `<span class="vp-tools-label">Tubes</span><div class="segmented" role="group" aria-label="Tube shape">
      ${['round', 'square'].map((s) => `<button type="button" data-tube="${s}" aria-pressed="${hooks.tubeShape === s}">${s === 'round' ? 'Round' : 'Square'}</button>`).join('')}</div>`;
  }
  if (state.view === 'cadfile') {
    const st = cadModelStatus();
    const note = [
      ...(st.loading ? [`Loading ${st.loading} STEP model${st.loading === 1 ? '' : 's'}…`] : []),
      ...st.missing.map((k) => `Could not load ${k.replace(':', ' ')} model`),
    ];
    $('#legend').innerHTML = [item('', hex(FAMILY_COLORS.motor), 'Motor'),
      ...Object.entries(FAMILY_LABELS).map(([k, l]) => item('', hex(FAMILY_COLORS[k]), l)),
      item('', hex(FAMILY_COLORS.link), 'Link connectors'),
      ...note.map((t) => `<span class="legend-item">${t}</span>`),
      ...(hooks.linkFillError ? [`<span class="legend-item">Links not filled: ${esc(hooks.linkFillError)}</span>`] : [])].join('');
    return;
  }
  $('#legend').innerHTML = state.view !== 'simple'
    ? [item('', hex(FAMILY_COLORS.motor), 'Motor'),
       ...Object.entries(FAMILY_LABELS).map(([k, l]) => item('', hex(FAMILY_COLORS[k]), l)),
       item('', hex(FAMILY_COLORS.link), 'Link')].join('')
    : [item('swatch-joint', '#6a7889', 'Joint (axis = z)'), item('swatch-line', '#28313b', 'Link')].join('');
}

// ---------------------------------------------------------------- Assembly tree

function chip(n) {
  let [letter, color] = (store.isFixed(n) ? CHIP.base : CHIP[n.type]) ?? ['J', hex(FAMILY_COLORS.motor)];
  if (n.type === 'actuator' && n.gearboxes.length) color = hex(FAMILY_COLORS[GEARBOXES[n.gearboxes.at(-1)].family]);
  return `<span class="chip" style="--c:${color}" aria-hidden="true">${letter}</span>`;
}

function meta(n) {
  if (n.type === 'actuator') return formatRatio(stackRatio(n.gearboxes));
  if (store.isFixed(n)) return 'locked';
  if (n.outputs.length > 1) return `${n.outputs.length} outputs`;
  const e = ui.frames[n.id]?.local.outputs[0].elements;
  return e ? len(Math.hypot(e[12], e[13], e[14]), state.units === 'in' ? 2 : 0) : '';
}

/** Joint slider shown under an actuator's row in the Assembly list. */
const jointSlider = (n) => `<div class="tree-joint">
    <input type="range" min="${-JOINT_LIMIT}" max="${JOINT_LIMIT}" step="1" value="${n.q}" data-q="${n.id}" aria-label="${esc(n.name || 'Joint')} angle">
    <output data-q-out="${n.id}">${Math.round(n.q)}°</output></div>`;

/** "name · output 2" (or just the name when the part has one output). */
const portName = (p) => { const n = node(p.nodeId); return n.outputs?.length > 1 ? `${n.name} · output ${p.index + 1}` : n.name; };

/** Passive joints a closed loop sets ('nodeId:index'): their sliders only show the angle. */
const loopDriven = () => new Set(loopUnknowns(state.robot).map((u) => u.port));

/** Move every passive joint slider to its joint's angle (a closed loop turns joints as the motors move). */
function syncPassiveSliders() {
  for (const el of $$('[data-pj]')) {
    const [id, i, axis] = el.dataset.pj.split(':');
    const o = node(id)?.outputs[Number(i)];
    if (!o || el === document.activeElement) continue;
    const v = axis === 'q' ? o.q ?? 0 : o.ball?.[axis] ?? 0;
    el.value = v;
    const out = $(`[data-pj-out="${el.dataset.pj}"]`);
    if (out) out.textContent = `${Math.round(v)}°`;
  }
}

/** Sliders for a passive joint (link n's output i): one for a hinge, x / y / z for a ball. */
function passiveSliders(n, i, cls) {
  const o = n.outputs[i];
  const driven = loopDriven().has(`${n.id}:${i}`);
  const one = (key, label, value) => `<div class="${cls}">
    ${key === 'q' ? '' : `<span class="pj-axis">${label}</span>`}
    <input type="range" min="${-PASSIVE.limit}" max="${PASSIVE.limit}" step="1" value="${value}" data-pj="${n.id}:${i}:${key}" aria-label="${esc(n.name)} ${o.end} ${label}"${driven ? ' disabled title="Set by the closed loop"' : ''}>
    <output data-pj-out="${n.id}:${i}:${key}">${Math.round(value)}°</output></div>`;
  return o.end === 'hinge' ? one('q', 'angle', o.q ?? 0) : ['x', 'y', 'z'].map((k) => one(k, k, o.ball?.[k] ?? 0)).join('');
}

/** The sliders under a link's row in the Assembly list when it sits on a ball or hinge joint. */
const passiveRowSliders = (n) => (n.type === 'link' && n.parent && store.isPassive(node(n.parent.nodeId), n.parent.index)
  ? passiveSliders(node(n.parent.nodeId), n.parent.index, 'tree-joint tree-passive') : '');

/** The first part of every limb: what's attached to each output of a link with several outputs. */
const limbRoots = () => Object.values(state.robot.nodes)
  .filter((n) => n.type === 'link' && n.children.length > 1)
  .flatMap((n) => n.children.filter((c) => c && hiddenCount(c)));

/** Parts attached after this one (not counting open ports). */
const hiddenCount = (id) => store.subtree(id).length - 1;

/** − / + button that collapses everything attached after a part; nothing for a part with nothing after it. */
function caret(n, cls) {
  if (!hiddenCount(n.id)) return `<span class="${cls} is-empty" aria-hidden="true"></span>`;
  const shut = ui.collapsed.has(n.id);
  return `<button type="button" class="${cls}" data-collapse="${n.id}" aria-expanded="${!shut}"
    aria-label="${shut ? 'Expand' : 'Collapse'} ${esc(n.name)}" title="${shut ? 'Show' : 'Hide'} the parts after ${esc(n.name)}">${shut ? '+' : '−'}</button>`;
}

function renderTree() {
  const sel = state.selection;
  const portOn = (id, i) => sel?.kind === 'port' && sel.nodeId === id && sel.index === i;
  const nodeOn = (id) => sel?.kind === 'node' && sel.nodeId === id;
  $('#btn-tree-view').setAttribute('aria-pressed', String(ui.treeView));

  // Tree window (when open): boxes top-down, joined by lines; branches sit side by side.
  if (ui.treeView) {
    const box = (id) => {
      const n = node(id);
      const shut = ui.collapsed.has(id);
      return `<li><div class="ot-node">
          <button type="button" class="ot-box tree-row${nodeOn(id) ? ' is-selected' : ''}" data-node="${id}" title="${esc(n.name)} · ${esc(meta(n))}">
            ${chip(n)}<span class="tree-name">${esc(n.name || 'Unnamed')}</span></button>
          ${caret(n, 'ot-caret')}
        </div>
        ${shut ? `<ul><li><button type="button" class="tree-hidden ot-more" data-collapse="${id}">+${hiddenCount(id)} hidden</button></li></ul>`
          : `<ul>${n.children.map((c, i) => (c ? box(c) : store.isCapped(n, i) ? `<li><div class="ot-node">
          <div class="ot-box ot-port tree-end"><span class="end-dot" aria-hidden="true"></span><span class="tree-name">Sphere</span></div>
        </div></li>` : `<li><div class="ot-node">
          <button type="button" class="ot-box ot-port tree-port${portOn(id, i) ? ' is-selected' : ''}" data-port="${id}:${i}">
            <span class="port-dot" aria-hidden="true"></span><span class="tree-name">${n.children.length > 1 ? `Output ${i + 1}` : 'Open'}</span></button>
        </div></li>`)).join('')}</ul>`}</li>`;
    };
    $('#tree-diagram').innerHTML = `<ul class="otree" aria-label="Robot parts">${box(state.robot.rootId)}</ul>`;
  }

  // Assembly panel list.
  const joined = (id, i) => store.loopAt(id, i);
  const port = (id, i) => (joined(id, i)
    ? `<li><button type="button" class="tree-port tree-joined${portOn(id, i) ? ' is-selected' : ''}" data-port="${id}:${i}">
      <span class="loop-mark" aria-hidden="true">⟲</span><span class="tree-name">Joined to ${esc(portName(joined(id, i).other))}</span><span class="tree-meta">loop</span></button></li>`
    : store.isCapped(node(id), i)
    ? `<li><div class="tree-port tree-end"><span class="end-dot" aria-hidden="true"></span><span class="tree-name">Sphere end</span>
      <span class="tree-meta">${len(node(id).outputs[i].sphere, state.units === 'in' ? 2 : 0)}</span></div></li>`
    : `<li><button type="button" class="tree-port${portOn(id, i) ? ' is-selected' : ''}" data-port="${id}:${i}">
      <span class="port-dot" aria-hidden="true"></span><span class="tree-name">${PORT_NAME[node(id).outputs?.[i]?.end] ?? 'Open port'}</span><span class="tree-meta">attach</span></button></li>`);
  const row = (n, fold) => `<li><div class="tree-line">${fold ? caret(n, 'tree-caret') : ''}<button type="button" class="tree-row${nodeOn(n.id) ? ' is-selected' : ''}" data-node="${n.id}">
      ${chip(n)}<span class="tree-name">${esc(n.name || 'Unnamed')}</span><span class="tree-meta">${esc(meta(n))}</span></button></div>
      ${n.type === 'actuator' ? jointSlider(n) : passiveRowSliders(n)}</li>`;
  const roots = limbRoots();
  const fold = $('#btn-collapse-all');
  fold.hidden = !roots.length;
  fold.textContent = roots.length && roots.every((id) => ui.collapsed.has(id)) ? 'Expand all' : 'Collapse all';

  // Search: a flat list of the parts whose names match, in build order.
  const q = ui.search.trim().toLowerCase();
  if (q) {
    const hits = store.buildOrder().map(node).filter((n) => (n.name || '').toLowerCase().includes(q));
    $('#tree').innerHTML = hits.length
      ? `<ul class="tree" role="tree" aria-label="Matching parts">${hits.map((n) => row(n, false)).join('')}</ul>`
      : `<p class="empty">No part is called “${esc(ui.search.trim())}”.</p>`;
    return;
  }

  // Serial chains stay flat; only a link with several outputs indents.
  const chain = (id) => {
    const n = node(id);
    let html = row(n, true);
    if (ui.collapsed.has(id)) {
      return html + `<li><button type="button" class="tree-hidden" data-collapse="${id}">+${hiddenCount(id)} hidden · show</button></li>`;
    }
    if (n.children.length === 1) {
      html += n.children[0] ? chain(n.children[0]) : port(id, 0);
    } else {
      html += n.children.map((c, i) => `<li class="branch">
        <div class="branch-label">${esc(n.name)} · output ${i + 1}</div>
        <ul>${c ? chain(c) : port(id, i)}</ul></li>`).join('');
    }
    return html;
  };
  $('#tree').innerHTML = `<ul class="tree" role="tree" aria-label="Robot parts">${chain(state.robot.rootId)}</ul>`;
}

/** Expand any collapsed part that hides the current selection. */
function revealSelection() {
  const sel = state.selection;
  let n = sel && node(sel.nodeId);
  if (n && sel.kind === 'port') ui.collapsed.delete(n.id);
  for (let p = n?.parent; p; p = node(p.nodeId).parent) ui.collapsed.delete(p.nodeId);
}

function bindTree() {
  // Tree view opens in its own window. It doesn't block the page, so picking a part in it
  // updates the inspector and 3D view behind it.
  const dlg = $('#tree-dialog');
  $('#btn-tree-view').addEventListener('click', () => {
    if (dlg.open) return dlg.close();
    ui.treeView = true;
    dlg.show();
    renderTree();
  });
  $('#tree-close').addEventListener('click', () => dlg.close());
  dlg.addEventListener('close', () => { ui.treeView = false; renderTree(); });
  document.addEventListener('keydown', (e) => {
    // Esc closes the window only; it shouldn't also clear the selection.
    if (e.key === 'Escape' && dlg.open) { dlg.close(); e.stopImmediatePropagation(); }
  });

  // The list and the tree window share one click handler.
  const onClick = (e) => {
    const fold = e.target.closest('[data-collapse]');
    if (fold) {
      const id = fold.dataset.collapse;
      if (!ui.collapsed.delete(id)) ui.collapsed.add(id);
      return renderTree();
    }
    const row = e.target.closest('[data-node]');
    if (row) return store.select({ kind: 'node', nodeId: row.dataset.node });
    const port = e.target.closest('[data-port]');
    if (port) {
      const [nodeId, i] = port.dataset.port.split(':');
      store.select({ kind: 'port', nodeId, index: Number(i) });
    }
  };
  $('#tree').addEventListener('click', onClick);
  $('#tree-diagram').addEventListener('click', onClick);

  $('#tree-search').addEventListener('input', (e) => { ui.search = e.target.value; renderTree(); });
  $('#btn-collapse-all').addEventListener('click', () => {
    const roots = limbRoots();
    if (roots.every((id) => ui.collapsed.has(id))) roots.forEach((id) => ui.collapsed.delete(id));
    else roots.forEach((id) => ui.collapsed.add(id));
    renderTree();
  });
}

// ---------------------------------------------------------------- Joints

function bindJoints() {
  // Joint sliders live in both the Assembly list and the inspector; keep them in sync.
  document.addEventListener('input', (e) => {
    const id = e.target.dataset?.q;
    if (!id || !node(id)) return;
    const n = node(id);
    n.q = Number(e.target.value);
    $$(`[data-q="${id}"]`).forEach((el) => { if (el !== e.target) el.value = n.q; });
    $$(`[data-q-out="${id}"]`).forEach((el) => { el.textContent = `${n.q}°`; });
    store.emit('params');
  });
  // Passive joint sliders (ball or hinge link outputs), in the Assembly list and the inspector.
  document.addEventListener('input', (e) => {
    const key = e.target.dataset?.pj;
    if (!key) return;
    const [id, i, axis] = key.split(':');
    const o = node(id)?.outputs[Number(i)];
    if (!o) return;
    const v = Number(e.target.value);
    if (axis === 'q') o.q = v;
    else o.ball = { ...o.ball, [axis]: v };
    $$(`[data-pj="${key}"]`).forEach((el) => { if (el !== e.target) el.value = v; });
    $$(`[data-pj-out="${key}"]`).forEach((el) => { el.textContent = `${v}°`; });
    store.emit('params');
  });
}

// ---------------------------------------------------------------- Inspector

function renderInspector() {
  const el = $('#inspector');
  const sel = state.selection;
  const n = sel && node(sel.nodeId);
  if (state.view === 'sim') {
    el.innerHTML = simInspector();
    return;
  }
  if (state.view === 'anim') return; // js/animate.js fills the panel
  if (!n) {
    el.innerHTML = `
      <div class="insp-section insp-head">
        <div class="eyebrow">Nothing selected</div>
        <h2 class="insp-title">Pick a port to keep building</h2>
        <p class="muted">Blue dots in the viewport and “Open port” rows in the assembly are free outputs.
        Select one to attach an actuator or a link, or select a part to edit it.</p>
      </div>`;
    return;
  }
  el.innerHTML = sel.kind === 'port' ? attachInspector(n, sel.index) : partInspector(n);
  renderMatrices();
}

/** Simulate mode: how to use it, and each joint's top speed. */
function simInspector() {
  const acts = store.buildOrder().map(node).filter((n) => n.type === 'actuator');
  const rows = acts.map((n) => `<tr><th scope="row">${esc(n.name || 'Unnamed')}</th>
    <td>${formatRatio(stackRatio(n.gearboxes))}</td><td>${num(jointSpeed(n), 0)} °/s</td></tr>`).join('');
  return `
    <div class="insp-section insp-head">
      <div class="eyebrow">Simulate</div>
      <h2 class="insp-title">Drag the robot</h2>
      <p class="muted">Grab any part in the view and pull it. The joints between the base and that part
      turn to follow, each no faster than its top speed: the motor speed divided by its gear reduction,
      so more reduction means a slower joint.
      Switch back to Model, CAD or Simplified to keep building.</p>
    </div>
    <div class="insp-section insp-body">
      ${acts.length ? `<table class="data-table sim-table">
        <thead><tr><th scope="col">Joint</th><th scope="col">Ratio</th><th scope="col">Top speed</th></tr></thead>
        <tbody>${rows}</tbody></table>` : '<p class="empty">Add actuators to have something to move.</p>'}
      ${acts.length ? workspaceButtons() : ''}
    </div>`;
}

// ---------------------------------------------------------------- Workspaces (simulate mode)

const WS = {
  reach: { label: 'Reachable Workspace', color: 0xe02020, opacity: 0.16 },
  dex: { label: 'Dexterous Workspace', color: 0x10b040, opacity: 0.3 },
};

/** Everything about the robot except the joint angles, which don't change its workspace. */
const workspaceSig = () => JSON.stringify(Object.values(state.robot.nodes).map(({ q, name, ...rest }) => rest));

function workspaceNote() {
  const w = ui.ws;
  if (w.busy) return `Sampling joint angles… ${Math.round(w.progress * 100)}%`;
  if (!w.data) return 'Each shape takes a few seconds to compute the first time.';
  const notes = [];
  if (w.show.reach) notes.push(w.data.reach ? 'Red: every point an end effector can reach.' : 'Nothing moves, so there is no reachable workspace.');
  if (w.show.dex) {
    notes.push(w.data.dex ? 'Green: points an end effector can reach while pointing in almost any direction.'
      : w.data.dexSkipped ? 'No dexterous workspace: pointing anywhere needs at least 5 joints before an end effector, and none has that many.'
        : 'No dexterous workspace: no point was found where an end effector can point in almost every direction.');
  }
  return notes.join(' ') || 'Click a button to show its shape.';
}

function workspaceButtons() {
  return `<div class="ws-buttons">
      ${Object.entries(WS).map(([k, v]) => `<button type="button" class="btn ws-${k}" data-ws="${k}" aria-pressed="${ui.ws.show[k]}"
        ${ui.ws.busy ? 'disabled' : ''}>${v.label}</button>`).join('')}
    </div>
    <p class="muted ws-note" id="ws-note">${workspaceNote()}</p>`;
}

/** Show or hide the shapes to match the toggles; drop them if the robot changed or simulate is off. */
function syncWorkspaces() {
  const w = ui.ws;
  if (w.data && w.sig !== workspaceSig()) {
    for (const [k, m] of Object.entries(w.meshes)) { ui.viewer.setWorkspace(k, null); m.geometry.dispose(); m.material.dispose(); }
    w.data = null;
    w.meshes = {};
  }
  if (state.view !== 'sim' || (!w.data && !w.busy)) w.show = { reach: false, dex: false };
  for (const k of Object.keys(WS)) {
    const ws = w.data?.[k];
    if (w.show[k] && ws && !w.meshes[k]) w.meshes[k] = workspaceMesh(ws, WS[k].color, WS[k].opacity);
    ui.viewer.setWorkspace(k, w.show[k] && ws ? w.meshes[k] : null);
  }
}

async function toggleWorkspace(kind) {
  const w = ui.ws;
  w.show[kind] = !w.show[kind];
  if (w.show[kind] && !w.data && !w.busy) {
    const sig = workspaceSig();
    w.busy = sig;
    w.progress = 0;
    renderInspector();
    const data = await computeWorkspaces(state.robot, (p) => {
      w.progress = p;
      const note = $('#ws-note');
      if (note) note.textContent = workspaceNote();
    });
    w.busy = null;
    if (sig === workspaceSig()) Object.assign(w, { sig, data, meshes: {} });
  }
  syncWorkspaces();
  if (state.view === 'sim') renderInspector();
}

/** Inspector for an output joined into a closed loop: where it goes, and Unjoin. */
function joinedInspector(n, index, other) {
  const gap = state.robot.loopGap;
  return `
    <div class="insp-section insp-head">
      <div class="eyebrow">Closed loop · ${n.outputs[index].end} joint</div>
      <h2 class="insp-title">${esc(portName({ nodeId: n.id, index }))}</h2>
      <p class="muted">Joined to ${esc(portName(other))}. The passive joints between the two turn so they meet${
        n.outputs[index].end === 'hinge' && node(other.nodeId).outputs[other.index].end === 'hinge' ? ' with their hinge axes lined up' : ''}.</p>
      ${gap ? `<p class="hint error">A loop can't close at these motor angles: ${len(gap)} apart.</p>` : ''}
      <button type="button" class="btn" data-act="unjoin">Unjoin</button>
    </div>`;
}

/** For an open ball or hinge output: join it to another open one to close a loop. */
function loopSection(n, index) {
  const options = store.loopCandidates({ nodeId: n.id, index });
  return `
    <div class="insp-section">
      <div class="label-row"><span class="label">Close a loop</span><span class="hint">instead of attaching a link</span></div>
      ${options.length ? `<p class="hint">Join this joint to another open ball or hinge joint. The joints between them then turn so the two meet.</p>
      <div class="loop-row">
        <select id="f-loop" aria-label="Joint to join to">${options.map((p) => `<option value="${p.nodeId}:${p.index}">${esc(portName(p))} (${node(p.nodeId).outputs[p.index].end})</option>`).join('')}</select>
        <button type="button" class="btn" data-act="join">Join</button>
      </div>` : '<p class="hint">Set another link output to a ball or hinge joint, and leave it open, to join it here.</p>'}
    </div>`;
}

function attachInspector(n, index) {
  const joinedTo = store.loopAt(n.id, index);
  if (joinedTo) return joinedInspector(n, index, joinedTo.other);
  const tab = addTabFor(state.selection);
  const d = ui.drafts[tab];
  const where = n.children.length > 1 ? `${n.name} · output ${index + 1}` : n.name;
  const passive = store.isPassive(n, index);
  return `
    <div class="insp-section insp-head">
      <div class="eyebrow">Attach to</div>
      <h2 class="insp-title">${esc(where)}</h2>
      <div class="tabs" role="tablist" aria-label="Part type">
        ${['actuator', 'link'].map((t) => `<button type="button" role="tab" class="tab" aria-selected="${t === tab}"
          data-act="tab" data-tab="${t}"${passive && t === 'actuator' ? ' disabled title="Only a link can attach to a ball or hinge joint"' : ''}>${t === 'actuator' ? 'Actuator' : 'Link'}</button>`).join('')}
      </div>
      ${passive ? `<p class="hint">This output is a ${n.outputs[index].end === 'ball' ? 'ball' : 'hinge'} joint, so the next part is a link that turns freely on it.</p>` : ''}
    </div>
    <div class="insp-section insp-body">
      <div class="field">
        <label for="f-name">Name</label>
        <input id="f-name" type="text" data-field="name" value="${esc(d.name)}" placeholder="${store.nextName(tab)}" autocomplete="off">
      </div>
      ${tab === 'actuator' ? actuatorFields(d) : linkFields(d, null)}
      <button type="button" class="btn btn-primary btn-block" data-act="add">Add ${tab}</button>
    </div>
    ${passive ? loopSection(n, index) : ''}`;
}

function partInspector(n) {
  const count = store.subtree(n.id).length;
  const fixed = store.isFixed(n);
  const body = n.type === 'actuator' ? jointField(n) + actuatorFields(n) : linkFields(n, n);
  return `
    <div class="insp-section insp-head">
      <div class="eyebrow">${fixed ? 'Base link · locked in place at the world origin' : TYPE_LABEL[n.type]}${n.type === 'actuator' ? ` · ${formatRatio(stackRatio(n.gearboxes))}` : ''}</div>
      <input class="title-input" id="f-title" type="text" data-field="name" value="${esc(n.name)}" aria-label="Part name" autocomplete="off">
    </div>
    <div class="insp-section insp-body">${body}</div>
    ${matrixSection(n.outputs?.length > 1 ? 'Output frames in world' : 'Output frame in world')}
    <div class="insp-section insp-actions">
      ${n.type === 'link' && !fixed && hooks.linkPartPlan ? '<button type="button" class="btn" data-act="export-link-step">Export link STEP</button>' : ''}
      ${n.type === 'link' && !fixed && hooks.linkPart?.(n).length > 1 ? `<p class="hint part-note">Prints as one piece: ${hooks.linkPart(n).map(esc).join(' + ')}</p>` : ''}
      ${fixed ? '' : `<button type="button" class="btn btn-danger" data-act="delete">${count > 1 ? `Delete with ${count - 1} attached` : 'Delete part'}</button>`}
    </div>`;
}

function jointField(n) {
  return `
    <div class="field">
      <label for="f-q">Joint angle q</label>
      <div class="range-row">
        <input type="range" id="f-q" min="${-JOINT_LIMIT}" max="${JOINT_LIMIT}" step="1" value="${n.q}" data-q="${n.id}">
        <output data-q-out="${n.id}">${n.q}°</output>
      </div>
    </div>`;
}

function gearboxOptions(selected) {
  return Object.entries(FAMILY_LABELS).map(([family, label]) => `
    <optgroup label="${label}">
      ${Object.entries(GEARBOXES).filter(([, g]) => g.family === family)
        .map(([k, g]) => `<option value="${k}"${k === selected ? ' selected' : ''}>${g.label}</option>`).join('')}
    </optgroup>`).join('');
}

function actuatorFields(d) {
  const motor = MOTORS[d.motor];
  return `
    <div class="field">
      <div class="label-row"><label for="f-motor">Motor</label>${actuatorInfo(d)}</div>
      <select id="f-motor" data-field="motor">
        ${Object.entries(MOTORS).map(([k, m]) => `<option value="${k}"${k === d.motor ? ' selected' : ''}>${esc(m.label)}</option>`).join('')}
      </select>
    </div>
    <div class="field">
      <div class="label-row"><label for="f-mount">Mount angle φ</label><span class="hint">turn about the port's z axis</span></div>
      <div class="range-row">
        <input type="range" id="f-mount" min="-180" max="180" step="15" value="${d.mount ?? 0}" data-field="mount">
        <input type="number" class="mount-num" step="15" value="${d.mount ?? 0}" data-field="mount" aria-label="Mount angle in degrees">
      </div>
    </div>
    <div class="field">
      <div class="label-row"><span class="label">Gearbox stack</span><span class="hint">motor → output</span></div>
      <ol class="stack">
        <li class="stack-item stack-motor">
          <span class="swatch" style="--c:${hex(FAMILY_COLORS.motor)}"></span>
          <span class="stack-name">${esc(motor.label)}</span>
        </li>
        ${d.gearboxes.map((k, i) => `
        <li class="stack-item">
          <span class="swatch" style="--c:${hex(FAMILY_COLORS[GEARBOXES[k].family])}"></span>
          <select id="f-gb-${i}" data-field="gearbox" data-i="${i}" aria-label="Gearbox ${i + 1}">${gearboxOptions(k)}</select>
          <button type="button" class="icon-btn" data-act="remove-gearbox" data-i="${i}" aria-label="Remove gearbox ${i + 1}"${d.gearboxes.length <= MIN_GEARBOXES
            ? ' disabled title="Every actuator needs at least one gearbox"' : ''}>×</button>
        </li>`).join('')}
      </ol>
      <button type="button" class="btn btn-ghost btn-small btn-start" data-act="add-gearbox"${d.gearboxes.length >= MAX_GEARBOXES ? ' disabled' : ''}>+ Add gearbox</button>
    </div>`;
}

/** Small "i" button that opens the actuator's input → output displacement. */
function actuatorInfo(d) {
  const motor = MOTORS[d.motor];
  const extra = d.gearboxes.length - 1;
  const perGearbox = GEARBOXES[d.gearboxes[0]].length;
  return `
    <details class="info">
      <summary aria-label="Motor displacement" title="Motor displacement">i</summary>
      <div class="info-pop">
        <dl class="xform">
          <dt>Vertical</dt><dd>${len(motor.rise)}</dd>
          <dt>Horizontal</dt><dd>${len(actuatorReach(d))}</dd>
        </dl>
        <p class="hint">Output turned 90° from the input. Each extra gearbox adds ${len(perGearbox)}${extra ? ` (${extra} here)` : ''}.</p>
        <p class="formula">Rz(φ) · Tx(h) · Tz(v) · Ry(90°) · Rz(q)</p>
      </div>
    </details>`;
}

/** d holds the outputs being edited; n is the live node (for port occupancy), or null for a draft. */
function linkFields(d, n) {
  const pose = d.spec === 'pose';
  const refs = pose ? refOptions(n) : null;
  const cell = (i, field, value, [key, sym, kind]) => `
    <label class="dh-cell">
      <span class="dh-sym">${sym}<small>${kind === 'len' ? state.units : kind}</small></span>
      <input id="${field}-${i}-${key}" type="number" step="${kind === 'len' ? unit().step : 5}" value="${kind === 'len' ? lenField(value) : value}"
        data-field="${field}" data-i="${i}" data-key="${key}">
    </label>`;
  return `
    <div class="field">
      <span class="label">Enter outputs as</span>
      <div class="tabs" role="group" aria-label="How outputs are entered">
        <button type="button" class="tab" aria-pressed="${!pose}" data-act="spec" data-spec="dh">D-H parameters</button>
        <button type="button" class="tab" aria-pressed="${pose}" data-act="spec" data-spec="pose">XYZ + angles</button>
      </div>
    </div>
    <div class="field">
      <div class="label-row"><span class="label">Outputs</span><span class="hint">${pose ? 'Position + rotation' : CONVENTION_SHORT[DH_CONVENTION]}</span></div>
      <p class="formula">T = ${pose ? POSE_FORMULA : CONVENTION_FORMULA[DH_CONVENTION]}</p>
      ${pose ? '<p class="hint">α, β, γ turn the output about the x, y, then z axis of the frame it is measured from.</p>' : ''}
      ${d.outputs.map((o, i) => {
        const used = Boolean(n?.children[i]);
        if (pose && !refs.some(([v]) => v === o.pose.ref)) o.pose.ref = 'input';
        const fellBack = pose && n && ui.solved.fallbacks[n.id]?.has(i);
        return `
        <fieldset class="dh">
          <legend>Output ${i + 1}${used ? ' <span class="tag">in use</span>' : ''}</legend>
          ${d.outputs.length > 1 ? `<button type="button" class="icon-btn dh-remove" data-act="remove-output" data-i="${i}"
            aria-label="Remove output ${i + 1}"${used ? ' disabled title="Delete the part on this output first"' : ''}>×</button>` : ''}
          ${endField(o, i, used, n)}
          ${pose ? `
          <div class="field ref-field">
            <label for="ref-${i}">Measured from</label>
            <select id="ref-${i}" data-field="ref" data-i="${i}">
              ${refs.map(([v, label]) => `<option value="${esc(v)}"${v === o.pose.ref ? ' selected' : ''}>${esc(label)}</option>`).join('')}
            </select>
            ${o.pose.ref !== 'input' ? '<span class="hint">Measured with every joint at 0°, then fixed to this link.</span>' : ''}
            ${fellBack ? '<span class="hint error">That frame depends on this link, so it is measured from the port instead.</span>' : ''}
          </div>
          <div class="dh-grid pose-grid">${POSE_FIELDS.map((f) => cell(i, 'pose', o.pose[f[0]], f)).join('')}</div>
          <div class="label-row rod-head"><span class="label">Rod leaves the port at</span><span class="hint">lat 90° = straight up z</span></div>
          <div class="dh-grid rod-grid">${ROD_FIELDS.map((f) => cell(i, 'rod', o.rod[f[0]], f)).join('')}</div>`
          : `<div class="dh-grid">${DH_FIELDS[DH_CONVENTION].map((f) => cell(i, 'dh', o.dh[f[0]], f)).join('')}</div>
          ${lossyStill(o) ? `<p class="hint error lossy">This XYZ pose has no exact D-H form (D-H has 4 numbers, a pose needs 6).
            These are the closest D-H values: the output is ${len(o.lossy.posErr)} and ${num(o.lossy.angErr, 1)}° off.
            Switch back to XYZ + angles to restore the exact pose.</p>` : ''}`}
        </fieldset>`;
      }).join('')}
      <button type="button" class="btn btn-ghost btn-small btn-start" data-act="add-output">+ Add output</button>
    </div>`;
}

/**
 * Output: a connector to keep building from, a sphere that ends the robot there, or a passive
 * joint (ball or hinge) that the next link turns freely on. n is the live link, or null for a draft.
 */
function endField(o, i, used, n) {
  const sphere = o.end === 'sphere';
  const onIt = n?.children[i] && node(n.children[i]);
  const tab = (end, label, why) => `<button type="button" class="tab" aria-pressed="${(o.end ?? 'output') === end}" data-act="end" data-end="${end}" data-i="${i}"${why ? ` disabled title="${why}"` : ''}>${label}</button>`;
  const joint = onIt?.type === 'actuator' ? 'Only a link can sit on a joint: delete the actuator on this output first' : '';
  return `<div class="field end-field">
      <span class="label">Output</span>
      <div class="tabs end-tabs" role="group" aria-label="Output ${i + 1} ends in">
        ${tab('output', 'Another output')}
        ${tab('sphere', 'Sphere end', used ? 'Delete the part on this output first' : '')}
        ${tab('ball', 'Ball joint', joint)}
        ${tab('hinge', 'Hinge joint', joint)}
      </div>
      ${sphere ? `<label class="end-size">Radius <input type="number" min="1" step="${unit().step}" value="${lenField(o.sphere)}" data-field="sphere" data-i="${i}">
        <span class="hint">${state.units}</span></label>` : ''}
      ${o.end === 'hinge' ? '<p class="hint">A hinge: the next link turns about this output’s z axis. Only a link can attach to it.</p>' : ''}
      ${o.end === 'ball' ? '<p class="hint">A ball joint: the next link turns freely, Rz(z) · Ry(y) · Rx(x). Only a link can attach to it.</p>' : ''}
      ${n && store.loopAt(n.id, i) ? `<p class="hint">Joined to ${esc(portName(store.loopAt(n.id, i).other))} in a closed loop.</p>`
        : n && (o.end === 'hinge' || o.end === 'ball') ? `<div class="pj-sliders">${passiveSliders(n, i, 'range-row pj-row')}</div>` : ''}
    </div>`;
}

/** Frames a link output can be measured from: its port (default), the world, or any part's output not built on this link. */
function refOptions(n) {
  const sel = state.selection;
  // The base has no port: its input is the world origin, so "World origin" covers it.
  const port = n ? n.parent : { nodeId: sel.nodeId, index: sel.index };
  const downstream = new Set(n ? store.subtree(n.id) : []);
  const opts = port ? [['input', 'The port it attaches to'], ['world', 'World origin (ground)']] : [['input', 'World origin (ground)']];
  for (const id of store.buildOrder()) {
    if (downstream.has(id)) continue;
    const m = node(id);
    m.children.forEach((_, i) => {
      if (id === port?.nodeId && i === port.index) return; // same frame as "the port"
      opts.push([`${id}:${i}`, m.children.length > 1 ? `${m.name} · output ${i + 1}` : `${m.name} output`]);
    });
  }
  return opts;
}

/** True while an output's D-H values are still the closest-fit ones made from a pose that D-H can't express exactly. */
function lossyStill(o) {
  return o.lossy && Object.keys(o.lossy.dh).every((k) => o.dh[k] === o.lossy.dh[k]);
}

/** Home-pose world frame of the port the edited link sits on (or would sit on). */
function linkInputHome(n) {
  const p = n ? n.parent : state.selection; // both carry { nodeId, index }
  if (!p) return new THREE.Matrix4(); // the base sits on the world origin
  return ui.solved.home[p.nodeId][p.index].clone();
}

/** Output i of the edited link relative to its input, as currently entered. */
function outputLocal(d, i, input) {
  const o = d.outputs[i];
  if (d.spec !== 'pose') return dhToMatrix(o.dh);
  return homeFrame(ui.solved, o.pose.ref, input).premultiply(input.clone().invert()).multiply(poseToMatrix(o.pose));
}

function matrixSection(title) {
  return `<div class="insp-section"><h3 class="section-title">${title}</h3><div id="matrices"></div></div>`;
}

function matrixHTML(m) {
  const e = m.elements; // column-major
  let cells = '';
  for (let r = 0; r < 4; r++) {
    for (let c = 0; c < 4; c++) {
      const v = e[c * 4 + r];
      const cls = r === 3 ? 'is-dim' : c === 3 ? 'is-pos' : '';
      cells += `<span class="${cls}">${c === 3 && r < 3 ? num(toUnit(v), unit().digits) : num(v, c === 3 ? 1 : 3)}</span>`;
    }
  }
  return `<div class="matrix">${cells}</div>`;
}

function renderMatrices() {
  const host = $('#matrices');
  const sel = state.selection;
  const f = sel && ui.frames[sel.nodeId];
  if (!host || !f) return;
  const list = sel.kind === 'port'
    ? [[null, f.outputs[sel.index]]]
    : f.outputs.map((m, i) => [f.outputs.length > 1 ? `Output ${i + 1}` : null, m]);
  host.innerHTML = list.map(([label, m]) => `${label ? `<div class="matrix-label">${label}</div>` : ''}${matrixHTML(m)}`).join('')
    + `<p class="hint matrix-note">Rotation | translation (${state.units}), base frame → this frame</p>`;
}

// ---------------------------------------------------------------- Robot data window
// Everything at the current joint angles, in the base (world) frame, in the chosen length unit.

/** Every open output is an end effector (a foot, a gripper mount, an arm tip…). */
function endEffectors() {
  const list = [];
  for (const id of store.buildOrder()) {
    const n = node(id);
    n.children.forEach((c, i) => {
      if (!c) list.push({ id, index: i, name: n.children.length > 1 ? `${n.name} · output ${i + 1}` : n.name });
    });
  }
  return list;
}

/** Actuators between the base and a part (that part included), base first. */
function chainJoints(id) {
  const out = [];
  for (let n = node(id); n; n = n.parent && node(n.parent.nodeId)) if (n.type === 'actuator') out.unshift(n);
  return out;
}

/**
 * Geometric Jacobian of an end effector, one column per joint on its chain:
 * linear part z × (p_ee − p_joint) in the length unit, angular part z (axis of the joint).
 */
function jacobianOf(ee) {
  const e = ui.frames[ee.id].outputs[ee.index].elements;
  return chainJoints(ee.id).map((j) => {
    const m = ui.frames[j.id].outputs[0].elements; // on the joint axis, z = axis
    const z = [m[8], m[9], m[10]];
    const r = [e[12] - m[12], e[13] - m[13], e[14] - m[14]];
    const v = [z[1] * r[2] - z[2] * r[1], z[2] * r[0] - z[0] * r[2], z[0] * r[1] - z[1] * r[0]];
    return { joint: j, col: [...v.map(toUnit), ...z] };
  });
}

/** Eigenvalues of a symmetric matrix (cyclic Jacobi rotations). */
function symmetricEigenvalues(A) {
  const n = A.length, a = A.map((r) => r.slice());
  for (let sweep = 0; sweep < 60; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += a[p][q] * a[p][q];
    if (off < 1e-20) break;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        if (Math.abs(a[p][q]) < 1e-30) continue;
        const theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
        const t = (theta >= 0 ? 1 : -1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1), s = t * c;
        for (let k = 0; k < n; k++) {
          const kp = a[k][p], kq = a[k][q];
          a[k][p] = c * kp - s * kq;
          a[k][q] = s * kp + c * kq;
        }
        for (let k = 0; k < n; k++) {
          const pk = a[p][k], qk = a[q][k];
          a[p][k] = c * pk - s * qk;
          a[q][k] = s * pk + c * qk;
        }
      }
    }
  }
  return a.map((r, i) => r[i]);
}

/** Rank, manipulability, condition number and singular values of a 6 × k Jacobian. */
function jacobianStats(cols) {
  const k = cols.length;
  const JtJ = cols.map((a) => cols.map((b) => a.col.reduce((s, v, i) => s + v * b.col[i], 0)));
  const sigma = symmetricEigenvalues(JtJ).map((l) => Math.sqrt(Math.max(0, l))).sort((x, y) => y - x);
  const top = sigma.slice(0, Math.min(6, k));
  const rank = top.filter((s) => s > 1e-6 * (sigma[0] || 1)).length;
  const manip = top.reduce((p, s) => p * s, 1);
  const cond = rank < top.length ? Infinity : top[0] / top.at(-1);
  return { sigma, rank, manip, cond };
}

/** For every motor, the next motor(s) reached through links only (plus the base to the first ones). */
function motorPairs() {
  const pairs = [];
  const walk = (from, id, via) => {
    for (const c of node(id).children) {
      if (!c) continue;
      const n = node(c);
      if (n.type === 'actuator') pairs.push({ from, to: n, via });
      else walk(from, c, [...via, n.name]);
    }
  };
  const root = node(state.robot.rootId);
  walk(null, root.id, [root.name]);
  for (const id of store.buildOrder()) if (node(id).type === 'actuator') walk(node(id), id, []);
  return pairs;
}

function tableHTML(colHeads, rowHeads, rows, fmt) {
  return `<div class="mtx-wrap"><table class="mtx">
    <thead><tr><th></th>${colHeads.map((h) => `<th>${esc(h)}</th>`).join('')}</tr></thead>
    <tbody>${rows.map((r, i) => `<tr><th>${rowHeads[i]}</th>${r.map((v) => `<td>${fmt(v, i)}</td>`).join('')}</tr>`).join('')}</tbody>
  </table></div>`;
}

const vec3 = (v, f = (x) => num(x, 3)) => `(${v.map(f).join(', ')})`;

/**
 * The chain from the base to an end effector as constant matrices, with every joint at 0°:
 * T_e(q) = A0 · Rz(q1) · A1 · Rz(q2) · … · Rz(qk) · Ak. Returns { A: [Matrix4], joints: [node] }.
 */
function chainConstants(ee) {
  const acts = Object.values(state.robot.nodes).filter((n) => n.type === 'actuator');
  const saved = acts.map((n) => n.q);
  acts.forEach((n) => { n.q = 0; });
  let frames;
  try { frames = forwardKinematics(state.robot, ui.solved); } finally { acts.forEach((n, i) => { n.q = saved[i]; }); }
  // Path from the base to the end effector, with the output taken at each part.
  const path = [];
  for (let n = node(ee.id), out = ee.index; n; ) {
    path.unshift({ n, out });
    if (!n.parent) break;
    out = n.parent.index;
    n = node(n.parent.nodeId);
  }
  const A = [], joints = [];
  let acc = new THREE.Matrix4();
  for (const { n, out } of path) {
    acc = acc.clone().multiply(frames[n.id].local.outputs[out]); // an actuator's output at 0°; its q turns after it
    if (n.type === 'actuator') { A.push(acc); joints.push(n); acc = new THREE.Matrix4(); }
  }
  A.push(acc);
  return { A, joints };
}

const SUB = '₀₁₂₃₄₅₆₇₈₉';
const sub = (i) => String(i).split('').map((c) => SUB[c]).join('');

/** The Jacobian of an end effector as a formula in q, its constants, and the same formula as Python. */
function jacobianFormulaHTML(ee) {
  const { A, joints } = chainConstants(ee);
  const k = joints.length;
  const key = `${ee.id}:${ee.index}`;
  const chain = ['A₀', ...joints.map((_, i) => `Rz(q${sub(i + 1)}) · A${sub(i + 1)}`)].join(' · ');
  const cols = joints.map((j, i) => {
    const before = i === 0 ? 'A₀' : ['A₀', ...joints.slice(0, i).map((_, t) => `Rz(q${sub(t + 1)})·A${sub(t + 1)}`)].join('·');
    return `<li><b>${esc(j.name)}</b> (q${sub(i + 1)}): T${sub(i + 1)}(q) = ${before}, &nbsp;
      J${sub(i + 1)}(q) = [ z${sub(i + 1)} × (p<sub>e</sub> − p${sub(i + 1)}) ; z${sub(i + 1)} ]
      ${i === 0 ? `<span class="hint">(constant: z₁ = ${vec3(A[0].elements.slice(8, 11))}, p₁ = ${vec3(A[0].elements.slice(12, 15), (x) => num(toUnit(x), unit().digits))} ${state.units})</span>` : ''}</li>`;
  }).join('');
  const r = (v) => String(Number(v.toFixed(6)) || 0);
  const pyMat = (m) => {
    const e = m.elements;
    return `np.array([${[0, 1, 2, 3].map((row) => `[${[0, 1, 2, 3].map((c) => r(e[c * 4 + row])).join(', ')}]`).join(', ')}])`;
  };
  const py = `import numpy as np

# ${ee.name}: T_e(q) = A0 · Rz(q1) · A1 · ... · Rz(q${k}) · A${k}   (lengths in mm)
# Joints, in order: ${joints.map((j) => j.name).join(', ')}
A = [
${A.map((m, i) => `    ${pyMat(m)},  # A${i}`).join('\n')}
]

def Rz(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def jacobian(q):
    """6 x ${k} geometric Jacobian in the base frame; q in radians. Rows: vx, vy, vz (mm/rad), wx, wy, wz."""
    T, axes = A[0], []
    for i, qi in enumerate(q):
        axes.append((T[:3, 2].copy(), T[:3, 3].copy()))  # joint i's axis z_i and a point p_i on it
        T = T @ Rz(qi) @ A[i + 1]
    pe = T[:3, 3]
    return np.column_stack([np.concatenate([np.cross(z, pe - p), z]) for z, p in axes])
`;
  ui.formulaCode[key] = py;
  return `<details class="data-formula" data-formula="${esc(key)}"${ui.formulaOpen.has(key) ? ' open' : ''}>
    <summary>Formula J(q) for any joint angles</summary>
    <p>T<sub>e</sub>(q) = ${chain}</p>
    <p class="hint">Each A is a constant matrix set by the robot's geometry (below); Rz(q) turns by joint angle q about z.
      T<sub>i</sub>(q) is the frame on joint i's axis: z<sub>i</sub> is its third column (the axis) and p<sub>i</sub> its
      position; p<sub>e</sub> is the position of T<sub>e</sub>(q).</p>
    <ol class="formula-cols">${cols}</ol>
    <div class="data-grid">${A.map((m, i) => `<div><h4 class="data-sub">A${sub(i)}</h4>${matrixHTML(m)}</div>`).join('')}</div>
    <div class="label-row"><span class="label">Python (numpy)</span>
      <button type="button" class="btn btn-small" data-copy-formula="${esc(key)}">Copy</button></div>
    <pre class="formula-code">${esc(py)}</pre>
  </details>`;
}

function renderRobotData() {
  if (!$('#data-dialog').open) return;
  const u = state.units, d = unit().digits;
  const acts = store.buildOrder().map(node).filter((n) => n.type === 'actuator');
  const ees = endEffectors();
  const f = ui.frames;

  // 1. Jacobians, then the joint values that go with them
  let html = `<section class="data-section">
    <h3 class="data-title">Jacobians</h3>
    <p class="hint">${acts.length} DoF · ${ees.length} end effector${ees.length === 1 ? '' : 's'} (open outputs) · current joint angles.
      Geometric Jacobian in the base frame: each column is one joint between the base and that end effector;
      rows v (${u}/rad) are linear velocity and ω (rad/rad) angular velocity per unit joint rate.</p>`;
  for (const ee of ees) {
    const cols = jacobianOf(ee);
    html += `<h4 class="data-sub">${esc(ee.name)}</h4>`;
    if (!cols.length) { html += '<p class="muted">No joints between the base and this point, so it cannot move.</p>'; continue; }
    const rows = [0, 1, 2, 3, 4, 5].map((r) => cols.map((c) => c.col[r]));
    html += tableHTML(cols.map((c) => c.joint.name), ['v<sub>x</sub>', 'v<sub>y</sub>', 'v<sub>z</sub>', 'ω<sub>x</sub>', 'ω<sub>y</sub>', 'ω<sub>z</sub>'],
      rows, (v, r) => num(v, r < 3 ? d : 3));
    const st = jacobianStats(cols);
    html += `<p class="data-stats">Rank <b>${st.rank}</b> of ${Math.min(6, cols.length)}
      · manipulability √det(JJᵀ) <b>${st.manip.toPrecision(4)}</b>
      · condition <b>${Number.isFinite(st.cond) ? st.cond.toPrecision(4) : '∞ (singular)'}</b>
      · singular values ${st.sigma.map((s) => s.toPrecision(4)).join(', ')}
      <span class="hint">(mixed units: ${u} and rad)</span></p>`;
    html += jacobianFormulaHTML(ee);
  }
  html += `<h4 class="data-sub">Joints</h4>`;
  html += acts.length ? `<div class="mtx-wrap"><table class="data-table">
    <thead><tr><th>Joint</th><th>q</th><th>Gearboxes</th><th>Ratio</th><th>Mount φ</th><th>Axis (world)</th><th>Point on axis (world, ${u})</th></tr></thead>
    <tbody>${acts.map((n) => {
      const m = f[n.id].outputs[0].elements;
      return `<tr><th>${esc(n.name)}</th><td>${num(n.q, 1)}°</td>
        <td>${n.gearboxes.map((k) => esc(GEARBOXES[k].label)).join(' → ')}</td>
        <td>${formatRatio(stackRatio(n.gearboxes))}</td><td>${num(n.mount ?? 0, 1)}°</td>
        <td>${vec3([m[8], m[9], m[10]])}</td><td>${vec3([m[12], m[13], m[14]], (x) => num(toUnit(x), d))}</td></tr>`;
    }).join('')}</tbody></table></div>` : '<p class="muted">No joints yet.</p>';
  html += '</section><hr class="data-sep">';

  // 2. End-effector transforms
  html += `<section class="data-section"><h3 class="data-title">End-effector transforms</h3>
    <p class="hint">Base frame → end effector. Rotation | translation (${u}).</p><div class="data-grid">`;
  for (const ee of ees) html += `<div><h4 class="data-sub">${esc(ee.name)}</h4>${matrixHTML(f[ee.id].outputs[ee.index])}</div>`;
  html += '</div></section><hr class="data-sep">';

  // 3. Motor to motor, links included
  html += `<section class="data-section"><h3 class="data-title">Motor to motor</h3>
    <p class="hint">Input of one motor → input of the next, including that motor (at its current angle) and every link between them.
      Rotation | translation (${u}). "Base" is the world origin.</p><div class="data-grid">`;
  const pairs = motorPairs();
  if (!pairs.length) html += '<p class="muted">No motors yet.</p>';
  for (const p of pairs) {
    const from = p.from ? f[p.from.id].input : new THREE.Matrix4();
    const T = from.clone().invert().multiply(f[p.to.id].input);
    html += `<div><h4 class="data-sub">${esc(p.from ? p.from.name : 'Base')} → ${esc(p.to.name)}</h4>${matrixHTML(T)}
      <p class="hint">${p.via.length ? `via ${p.via.map(esc).join(', ')}` : 'directly attached'}</p></div>`;
  }
  html += '</div></section>';
  $('#data-body').innerHTML = html;
}

function bindDataDialog() {
  const dlg = $('#data-dialog');
  $('#btn-data').addEventListener('click', () => { dlg.showModal(); renderRobotData(); });
  // Formula sections stay unfolded across the live re-renders.
  $('#data-body').addEventListener('toggle', (e) => {
    const key = e.target.dataset?.formula;
    if (key) { if (e.target.open) ui.formulaOpen.add(key); else ui.formulaOpen.delete(key); }
  }, true);
  $('#data-body').addEventListener('click', async (e) => {
    const b = e.target.closest('[data-copy-formula]');
    if (!b) return;
    try {
      await navigator.clipboard.writeText(ui.formulaCode[b.dataset.copyFormula]);
      b.textContent = 'Copied';
    } catch {
      b.textContent = 'Select and copy';
    }
    setTimeout(() => { b.textContent = 'Copy'; }, 1500);
  });
  $('#data-close').addEventListener('click', () => dlg.close());
  dlg.addEventListener('click', (e) => { if (e.target === dlg) dlg.close(); });
}

/** The object the inspector is editing: the selected part (live) or the add-part draft. */
function editing() {
  const sel = state.selection;
  if (sel?.kind === 'node') return { data: node(sel.nodeId), live: true };
  return { data: ui.drafts[addTabFor(sel)], live: false };
}

function changed(live, kind) {
  if (live) return store.emit(kind);
  refreshViewer(); // draft edits show up in the preview straight away
  if (kind === 'structure') renderInspector();
}

function bindInspector() {
  const el = $('#inspector');

  el.addEventListener('input', (e) => {
    const t = e.target, field = t.dataset.field;
    const { data, live } = editing();
    if (field === 'name') {
      data.name = t.value;
      changed(live, 'rename');
    } else if (field === 'mount') {
      const v = parseFloat(t.value);
      if (!Number.isFinite(v)) return;
      data.mount = v;
      // The slider and the number box mirror each other.
      $$('[data-field="mount"]', el).forEach((m) => { if (m !== t) m.value = v; });
      changed(live, 'params');
    } else if (field === 'dh') {
      const v = parseFloat(t.value);
      if (!Number.isFinite(v)) return;
      const o = data.outputs[Number(t.dataset.i)];
      o.dh[t.dataset.key] = LENGTH_KEYS.has(t.dataset.key) ? fromUnit(v) : v;
      // Edited by hand, so it is no longer the closest fit to an old pose.
      if (o.lossy && !lossyStill(o)) { delete o.lossy; t.closest('fieldset').querySelector('.lossy')?.remove(); }
      changed(live, 'params');
    } else if (field === 'pose') {
      const v = parseFloat(t.value);
      if (!Number.isFinite(v)) return;
      data.outputs[Number(t.dataset.i)].pose[t.dataset.key] = LENGTH_KEYS.has(t.dataset.key) ? fromUnit(v) : v;
      changed(live, 'params');
    } else if (field === 'sphere') {
      const v = parseFloat(t.value);
      if (!Number.isFinite(v) || v <= 0) return;
      data.outputs[Number(t.dataset.i)].sphere = fromUnit(v);
      changed(live, 'params');
    } else if (field === 'rod') {
      const v = parseFloat(t.value);
      if (!Number.isFinite(v)) return;
      const o = data.outputs[Number(t.dataset.i)];
      o.rod[t.dataset.key] = v;
      changed(live, 'params');
    }
  });

  el.addEventListener('change', (e) => {
    const t = e.target, field = t.dataset.field;
    const { data, live } = editing();
    if (field === 'motor') { data.motor = t.value; changed(live, 'structure'); }
    if (field === 'gearbox') { data.gearboxes[Number(t.dataset.i)] = t.value; changed(live, 'structure'); }
    if (field === 'name' && live && !data.name.trim()) { data.name = store.nextName(data.type); store.emit('structure'); }
    if (field === 'ref') {
      // Re-express the same output in the new frame, so the link stays put and the numbers change.
      const i = Number(t.dataset.i);
      const input = linkInputHome(live ? data : null);
      data.outputs[i].pose = poseFromLocal(outputLocal(data, i, input), t.value, input, ui.solved);
      changed(live, 'structure');
    }
  });

  el.addEventListener('click', (e) => {
    const ws = e.target.closest('[data-ws]');
    if (ws && !ws.disabled) toggleWorkspace(ws.dataset.ws);
  });
  el.addEventListener('click', (e) => {
    const b = e.target.closest('[data-act]');
    if (!b || b.disabled) return;
    const { data, live } = editing();
    const i = Number(b.dataset.i);
    switch (b.dataset.act) {
      case 'tab':
        ui.addTab = b.dataset.tab;
        refreshViewer();
        renderInspector();
        break;
      case 'add-gearbox':
        data.gearboxes.push(data.gearboxes.at(-1) ?? 'planetary_5');
        changed(live, 'structure');
        break;
      case 'remove-gearbox':
        if (data.gearboxes.length <= MIN_GEARBOXES) break;
        data.gearboxes.splice(i, 1);
        changed(live, 'structure');
        break;
      case 'spec': {
        const spec = b.dataset.spec;
        if (spec === data.spec) break;
        // Carry the current transform over, so the link keeps its shape.
        const input = linkInputHome(live ? data : null);
        data.outputs.forEach((o, j) => {
          if (spec === 'dh') {
            const fit = matrixToDH(outputLocal(data, j, input));
            o.dh = fit.dh;
            // Not every pose has a D-H form. Remember the exact pose so switching back restores it.
            if (fit.posErr > 0.01 || fit.angErr > 0.01) o.lossy = { dh: { ...fit.dh }, pose: { ...o.pose }, posErr: fit.posErr, angErr: fit.angErr };
            else delete o.lossy;
          } else {
            o.pose = lossyStill(o)
              ? { ...o.lossy.pose }
              : poseFromLocal(dhToMatrix(o.dh), o.pose?.ref ?? 'input', input, ui.solved);
            delete o.lossy;
          }
        });
        data.spec = spec;
        changed(live, 'structure');
        break;
      }
      case 'end': {
        const o = data.outputs[i];
        if (o.end === b.dataset.end) break;
        store.setEnd(o, b.dataset.end);
        if (live && !store.isPassive(data, i)) store.unjoinLoop({ nodeId: data.id, index: i }, true); // only joints close loops
        changed(live, 'structure'); // the output stops (or starts) being a port, or a joint
        break;
      }
      case 'add-output':
        if (live) store.addLinkOutput(data.id);
        else { data.outputs.push(store.defaultOutput()); changed(false, 'structure'); }
        break;
      case 'remove-output':
        if (live) store.removeLinkOutput(data.id, i);
        else { data.outputs.splice(i, 1); changed(false, 'structure'); }
        break;
      case 'join': {
        const sel = state.selection;
        const [nodeId, index] = $('#f-loop').value.split(':');
        store.joinLoop({ nodeId: sel.nodeId, index: sel.index }, { nodeId, index: Number(index) });
        break;
      }
      case 'unjoin':
        store.unjoinLoop({ nodeId: state.selection.nodeId, index: state.selection.index });
        break;
      case 'add': {
        const sel = state.selection;
        const draft = structuredClone(data);
        data.name = '';
        const port = { nodeId: sel.nodeId, index: sel.index };
        if (addTabFor(sel) === 'actuator') store.addActuator(port, draft);
        else store.addLink(port, draft);
        break;
      }
      case 'delete':
        requestDelete(data.id);
        break;
      case 'export-link-step': {
        let name = data.name;
        ui.runExport({
          heading: 'Export link STEP', showNote: false,
          wait: 'Building the part as a solid. The first export loads the CAD kernel (about 50 MB).',
          job: async (onProgress) => { const r = await exportLinkSTEP(data, onProgress); name = r.name; return r; },
          file: () => `${slug(name)}.step`,
          summary: (r) => `${r.name}, one solid (inches, Y-up)`,
        });
        break;
      }
    }
  });
}

/** Deleting a part with others attached takes a second click. */
function requestDelete(id) {
  const n = node(id);
  if (!n || store.isFixed(n)) return;
  const count = store.subtree(id).length;
  if (count > 1 && ui.armedDelete !== id) {
    ui.armedDelete = id;
    const b = $('[data-act="delete"]');
    if (b) { b.textContent = `Confirm: delete ${count} parts`; b.classList.add('is-armed'); }
    return;
  }
  store.removeNode(id);
}

// ---------------------------------------------------------------- Status bar

function renderStatus() {
  const nodes = Object.values(state.robot.nodes);
  const dof = nodes.filter((n) => n.type === 'actuator').length;
  const passive = nodes.reduce((k, n) => k + (n.type === 'link' ? n.children.filter((c, i) => c && store.isPassive(n, i)).length : 0), 0);
  const loops = state.robot.loops.length;
  const open = nodes.reduce((k, n) => k + n.children.filter((_, i) => store.isOpenPort(n, i)).length, 0);
  const sel = state.selection;
  const f = sel && ui.frames[sel.nodeId];
  let pos = '';
  if (f) {
    const e = (sel.kind === 'port' ? f.outputs[sel.index] : f.outputs[0]).elements;
    const d = unit().digits;
    pos = `Selected frame  x ${num(toUnit(e[12]), d)}  y ${num(toUnit(e[13]), d)}  z ${num(toUnit(e[14]), d)} ${state.units}`;
  }
  $('#status').innerHTML = `
    <span>${nodes.length - 1} parts</span><span>${dof} DoF</span>${passive ? `<span>${passive} passive joint${passive === 1 ? '' : 's'}</span>` : ''}${loops ? `<span>${loops} closed loop${loops === 1 ? '' : 's'}</span>` : ''}<span>${open} open ports</span>
    ${state.robot.loopGap ? `<span class="status-warn">Loop can't close: ${len(state.robot.loopGap)} apart</span>` : ''}
    <span class="status-spacer"></span>
    <span class="mono">${pos}</span>
    <span>${CONVENTION_SHORT[DH_CONVENTION]} · deg</span>
    <div class="segmented units-toggle" role="group" aria-label="Length units">
      ${['mm', 'in'].map((u) => `<button type="button" data-units="${u}" aria-pressed="${u === state.units}">${u}</button>`).join('')}
    </div>`;
}

// ---------------------------------------------------------------- Project file

function download(blob, filename) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = filename;
  document.body.append(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

function bindFileDialog() {
  const dlg = $('#file-dialog'), text = $('#file-text'), err = $('#file-error'), copy = $('#file-copy');
  const tryLoad = (src) => {
    try {
      store.load(src);
      dlg.close();
      ui.viewer.fit();
    } catch (x) {
      err.textContent = `Could not load the project. ${x.message}`;
    }
  };
  $('#btn-file').addEventListener('click', () => {
    text.value = store.serialize();
    err.textContent = '';
    dlg.showModal();
  });
  $('#file-close').addEventListener('click', () => dlg.close());
  $('#file-cancel').addEventListener('click', () => dlg.close());
  // The dialog has no padding, so a click whose target is the dialog itself landed on the backdrop.
  dlg.addEventListener('click', (e) => { if (e.target === dlg) dlg.close(); });
  copy.addEventListener('click', async () => {
    try {
      await navigator.clipboard.writeText(text.value);
      copy.textContent = 'Copied';
      setTimeout(() => { copy.textContent = 'Copy'; }, 1500);
    } catch {
      text.select();
    }
  });
  $('#file-download').addEventListener('click', () => {
    download(new Blob([text.value], { type: 'application/json' }), `${slug(state.name)}.json`);
  });
  $('#file-load').addEventListener('click', () => tryLoad(text.value));
  $('#file-input').addEventListener('change', async (e) => {
    const file = e.target.files[0];
    if (file) tryLoad(await file.text());
    e.target.value = '';
  });

  // Top bar Import JSON.
  $('#btn-import').addEventListener('click', () => $('#import-input').click());
  $('#import-input').addEventListener('change', async (e) => {
    const file = e.target.files[0];
    e.target.value = '';
    if (!file) return;
    const src = await file.text();
    try {
      store.load(src);
      ui.viewer.fit();
    } catch (x) {
      // Show the file and the reason, so it can be fixed and loaded from the window.
      text.value = src;
      err.textContent = `Could not import ${file.name}. ${x.message}`;
      dlg.showModal();
    }
  });
}

// ---------------------------------------------------------------- Exports (STEP, URDF, link STEP)

function bindExportDialog() {
  const dlg = $('#export-dialog');
  const title = $('#export-title'), step = $('#export-step'), bar = $('#export-progress'), detail = $('#export-detail');
  const note = $('.export-note', dlg), again = $('#export-again');
  let result = null, busy = false, save = null;
  const mb = (bytes) => `${(bytes / 1048576).toFixed(bytes < 10485760 ? 1 : 0)} MB`;
  const close = () => { if (!busy) dlg.close(); };

  /** Run one export in the window: progress while it builds, then the download and a summary. */
  const run = async ({ heading, wait, job, file, summary, showNote }) => {
    if (busy) return dlg.showModal();
    busy = true;
    result = null;
    again.hidden = true;
    title.textContent = heading;
    note.hidden = !showNote;
    step.textContent = 'Getting ready…';
    detail.textContent = wait;
    bar.removeAttribute('value'); // indeterminate until the first step
    dlg.showModal();
    try {
      result = await job(({ step: s, done, total }) => {
        step.textContent = `${s}…`;
        bar.max = total;
        bar.value = done;
      });
      save = () => download(result.blob, file());
      step.textContent = 'Done: the download has started.';
      detail.textContent = `${summary(result)}, ${mb(result.blob.size)}.`;
      again.hidden = false;
      save();
    } catch (x) {
      step.textContent = 'The export did not finish.';
      detail.textContent = x.message;
      bar.value = 0;
    } finally {
      busy = false;
    }
  };

  ui.runExport = run;
  $('#btn-export-step').addEventListener('click', () => run({
    heading: 'Export STEP', showNote: true, job: exportRobotSTEP,
    wait: 'The full models are large and every link is built as a solid, so this takes a little while. The first export loads the CAD kernel (about 50 MB).',
    file: () => `${slug(state.name)}.step`,
    summary: (r) => `${r.actuators} actuators and ${r.links} link parts as one STEP assembly (inches, Y-up)`,
  }));
  $('#btn-export-urdf').addEventListener('click', () => run({
    heading: 'Export URDF', showNote: false, job: exportURDF,
    wait: 'Building robot.urdf and its meshes from the CAD shells.',
    file: () => `${slug(state.name)}-urdf.zip`,
    summary: (r) => `${r.links} bodies, ${r.joints} joints, about ${r.mass.toFixed(2)} kg (robot.urdf + meshes/)`,
  }));
  again.addEventListener('click', () => result && save?.());
  $('#export-close').addEventListener('click', close);
  $('#export-done').addEventListener('click', close);
  dlg.addEventListener('cancel', (e) => { if (busy) e.preventDefault(); });
}

// ---------------------------------------------------------------- Keyboard

function bindKeys() {
  document.addEventListener('keydown', (e) => {
    if (e.target.closest?.('input, select, textarea, dialog')) return;
    if (e.key === 'Escape') store.select(null);
    else if (e.key === 'f' || e.key === 'F') ui.viewer.fit();
    else if ((e.key === 'Delete' || e.key === 'Backspace') && state.selection?.kind === 'node') {
      e.preventDefault();
      requestDelete(state.selection.nodeId);
    }
  });
}
