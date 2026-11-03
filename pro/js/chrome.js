// The pro workspace around the builder: the commands behind the menu bar, the ribbon and the keyboard;
// the left dock's tabs and its Loops list; and, over the 3D view, a view cube and axes that turn with
// the camera, a label on the selected part, tags for what's shown, a ground grid and screenshots.
// Everything here drives the same store and UI as the classic layout; nothing is a separate copy of it.
import * as THREE from 'three';
import * as store from './robot.js';
import { state, node } from './robot.js';
import { forwardKinematics, solveLinks, stackRatio, formatRatio } from './kinematics.js';

const $ = (s) => document.querySelector(s);
const $$ = (s) => [...document.querySelectorAll(s)];
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const click = (sel) => { const el = $(sel); if (el && !el.hidden && !el.disabled) el.click(); };
const slug = (s) => (s || 'robot').trim().toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '') || 'robot';
const deg = (v) => `${v < 0 ? '−' : ''}${Math.abs(v).toFixed(1)}°`;

const VIEW_NAME = { cad: 'Build', cadfile: 'CAD', simple: 'Simplified', sim: 'Simulate', anim: 'Animate' };
// Camera directions (from the target toward the camera). Front looks along +y, as in CAD.
const CAMERA = {
  iso: [0.62, -0.68, 0.42], top: [0, -0.02, 1], bottom: [0, 0.02, -1],
  front: [0, -1, 0], back: [0, 1, 0], right: [1, 0, 0], left: [-1, 0, 0],
};
const CUBE_FACES = [
  { key: 'top', label: 'Top', n: [0, 0, 1], up: [0, 1, 0] },
  { key: 'bottom', label: 'Bottom', n: [0, 0, -1], up: [0, -1, 0] },
  { key: 'front', label: 'Front', n: [0, -1, 0], up: [0, 0, 1] },
  { key: 'back', label: 'Back', n: [0, 1, 0], up: [0, 0, 1] },
  { key: 'right', label: 'Right', n: [1, 0, 0], up: [0, 0, 1] },
  { key: 'left', label: 'Left', n: [-1, 0, 0], up: [0, 0, 1] },
];
const SNAP_TIME = 0.3; // s

// Viewing toggles, remembered in this browser.
const PREFS_KEY = 'rbpro.prefs';
const prefs = { grid: true, callout: true, timeline: true };
try { Object.assign(prefs, JSON.parse(localStorage.getItem(PREFS_KEY) || '{}')); } catch { /* storage blocked */ }
const savePrefs = () => { try { localStorage.setItem(PREFS_KEY, JSON.stringify(prefs)); } catch { /* storage blocked */ } };

const SHORTCUTS = [
  ['Project', [['Alt+N', 'New blank robot'], ['Alt+E', 'Examples'], ['Ctrl+O', 'Open JSON'], ['Ctrl+S', 'Save JSON']]],
  ['Edit', [['Ctrl+Z', 'Undo'], ['Ctrl+Y or Ctrl+Shift+Z', 'Redo'], ['Alt+↑', 'Select the parent part'], ['Tab', 'Next open port'], ['Esc', 'Deselect'], ['Del', 'Delete the selected part']]],
  ['View', [['1 – 5', 'Build, CAD, Simplified, Simulate, Animate'], ['F', 'Fit to robot'], ['0 · 7 · 8 · 9', 'Isometric, top, front, right'], ['Alt+F', 'Joint frames'], ['G', 'Ground grid'], ['L', 'Selection label'], ['T', 'Timeline']]],
  ['Motion', [['Space', 'Play or pause the animation'], ['W', 'Walk (legged robots)']]],
  ['Other', [['Alt+D', 'Robot data'], ['Alt+P', 'Screenshot'], ['?', 'This list']]],
];

export function initChrome(viewer, { animator, history }) {
  let frames = {};
  const solve = () => { frames = forwardKinematics(state.robot, solveLinks(state.robot)); };

  // ---------------------------------------------------------------- Camera

  let snap = null; // { from, to, target, dist, t }
  function snapTo(key) {
    const target = viewer.controls.target.clone();
    const dist = viewer.camera.position.distanceTo(target);
    const from = viewer.camera.position.clone().sub(target).normalize();
    const to = new THREE.Vector3(...CAMERA[key]).normalize();
    snap = { from, to, target, dist, t: 0 };
  }
  viewer.onFrame((dt) => {
    if (snap) {
      snap.t = Math.min(SNAP_TIME, snap.t + dt);
      const u = snap.t / SNAP_TIME, k = u * u * (3 - 2 * u);
      const dir = snap.from.clone().lerp(snap.to, k);
      if (dir.lengthSq() < 1e-6) dir.copy(snap.to); // opposite directions: go straight there
      viewer.camera.position.copy(snap.target).addScaledVector(dir.normalize(), snap.dist);
      if (u >= 1) snap = null;
    }
    drawCube();
    drawTriad();
    placeCallout();
  });

  // ---------------------------------------------------------------- View cube (CSS faces turned with the camera)

  const cube = $('#view-cube');
  const HALF = 27; // px: half the face size
  const S = new THREE.Matrix4().makeScale(1, -1, 1); // CSS y points down
  const faces = CUBE_FACES.map((f) => {
    const el = document.createElement('button');
    el.type = 'button';
    el.className = 'cube-face';
    el.textContent = f.label;
    el.setAttribute('aria-label', `${f.label} view`);
    el.addEventListener('click', () => snapTo(f.key));
    cube.append(el);
    const n = new THREE.Vector3(...f.n);
    // The face's own frame: as if a camera looked straight at it, so its label reads upright then.
    const look = new THREE.Matrix4().lookAt(n, new THREE.Vector3(), new THREE.Vector3(...f.up));
    const place = new THREE.Matrix4().makeTranslation(n.x * HALF, n.y * HALF, n.z * HALF).multiply(look).multiply(S);
    return { el, n, place };
  });
  const view = new THREE.Matrix4(), m = new THREE.Matrix4(), v = new THREE.Vector3();
  function drawCube() {
    view.makeRotationFromQuaternion(viewer.camera.quaternion).transpose();
    for (const f of faces) {
      v.copy(f.n).applyMatrix4(view);
      f.el.style.visibility = v.z > 0.02 ? 'visible' : 'hidden';
      f.el.style.zIndex = String(Math.round(v.z * 100) + 100);
      m.copy(S).multiply(view).multiply(f.place);
      f.el.style.transform = `matrix3d(${m.elements.map((x) => x.toFixed(5)).join(',')})`;
    }
  }

  // ---------------------------------------------------------------- Axes triad (bottom left)

  const triad = $('#triad');
  const AXES = [['x', [1, 0, 0]], ['y', [0, 1, 0]], ['z', [0, 0, 1]]];
  function drawTriad() {
    const parts = AXES.map(([name, a]) => {
      v.set(...a).applyMatrix4(view);
      return { name, x: v.x * 20, y: -v.y * 20, z: v.z };
    }).sort((p, q) => p.z - q.z);
    triad.innerHTML = parts.map((p) => `<line x1="0" y1="0" x2="${p.x.toFixed(1)}" y2="${p.y.toFixed(1)}" class="tr-${p.name}"></line>
      <text x="${(p.x * 1.32).toFixed(1)}" y="${(p.y * 1.32 + 3.5).toFixed(1)}" class="tr-label">${p.name.toUpperCase()}</text>`).join('')
      + '<circle r="2.2" class="tr-origin"></circle>';
  }

  // ---------------------------------------------------------------- Selection label

  const callout = $('#callout');
  const vp = $('#viewport');
  const where = new THREE.Vector3();
  function calloutWorld() {
    const sel = state.selection;
    const f = sel && frames[sel.nodeId];
    if (!f) return null;
    if (sel.kind === 'port') return f.outputs[sel.index];
    return node(sel.nodeId).type === 'actuator' ? f.outputs[0] : f.input;
  }
  function renderCallout() {
    const sel = state.selection;
    const n = sel && node(sel.nodeId);
    callout.hidden = !prefs.callout || !n || !frames[n.id];
    if (callout.hidden) return;
    let title = n.name || 'Unnamed', line;
    if (sel.kind === 'port') {
      title = n.children.length > 1 ? `${n.name} · output ${sel.index + 1}` : n.name;
      line = { ball: 'Open ball joint', hinge: 'Open hinge' }[n.outputs?.[sel.index]?.end] ?? 'Open port: attach a part';
    } else if (n.type === 'actuator') {
      line = `q ${deg(n.q)} · ${formatRatio(stackRatio(n.gearboxes))}`;
    } else {
      const outs = n.outputs.length;
      line = store.isFixed(n) ? 'Base · fixed' : `Link · ${outs} output${outs === 1 ? '' : 's'}`;
    }
    callout.innerHTML = `<span class="co-title">${esc(title)}</span><span class="co-line">${esc(line)}</span>`;
  }
  function placeCallout() {
    if (callout.hidden) return;
    const w = calloutWorld();
    if (!w) return;
    where.setFromMatrixPosition(w).project(viewer.camera);
    const off = where.z > 1 || Math.abs(where.x) > 1.2 || Math.abs(where.y) > 1.2;
    callout.style.visibility = off ? 'hidden' : 'visible';
    const x = (where.x + 1) / 2 * vp.clientWidth, y = (1 - where.y) / 2 * vp.clientHeight;
    callout.style.transform = `translate(${Math.round(x + 14)}px, ${Math.round(y - 14)}px)`;
  }

  // ---------------------------------------------------------------- Tags over the view, the Properties note

  /** The saved pose the joints are at, if any. */
  function currentPose() {
    const p = state.poses.find((x) => Object.entries(x.q).every(([id, q]) => !node(id) || Math.abs(node(id).q - q) < 0.05));
    return p?.name;
  }
  function renderTags() {
    const tags = ['Perspective', VIEW_NAME[state.view] ?? state.view];
    if (state.view === 'anim' && animator.anim) tags.push(`Animation: ${animator.anim.name}`);
    else if (state.poses.length) tags.push(`Pose: ${currentPose() ?? 'custom'}`);
    $('#vp-tags').innerHTML = tags.map((t) => `<span class="vp-tag">${esc(t)}</span>`).join('');
    const sel = state.selection, n = sel && node(sel.nodeId);
    $('#props-note').textContent = !n ? (state.view === 'sim' || state.view === 'anim' ? VIEW_NAME[state.view] : 'Nothing selected')
      : sel.kind === 'port' ? 'Open port' : store.isFixed(n) ? 'Base' : n.type === 'actuator' ? 'Actuator' : 'Link';
  }

  // ---------------------------------------------------------------- Ground grid

  /** Lowest point of the robot as it stands now (sphere ends counted to their bottom), never above the base. */
  function groundHeight() {
    let z = 0;
    const p = new THREE.Vector3();
    for (const [id, f] of Object.entries(frames)) {
      const n = node(id);
      z = Math.min(z, p.setFromMatrixPosition(f.input).z);
      f.outputs.forEach((o, i) => {
        const r = n.type === 'link' && n.outputs[i]?.end === 'sphere' ? n.outputs[i].sphere ?? 0 : 0;
        z = Math.min(z, p.setFromMatrixPosition(o).z - r);
      });
    }
    return z;
  }
  let groundZ = 0;
  const placeGround = () => viewer.setGround(groundZ, prefs.grid);

  // ---------------------------------------------------------------- Dock tabs and the Loops list

  const TABS = ['assembly', 'poses', 'loops'];
  function showTab(key) {
    for (const k of TABS) {
      const on = k === key;
      $(`#tab-${k}`).setAttribute('aria-selected', String(on));
      $(`#tab-${k}`).tabIndex = on ? 0 : -1;
      $(`#pane-${k}`).hidden = !on;
    }
  }
  for (const k of TABS) $(`#tab-${k}`).addEventListener('click', () => showTab(k));
  $('.dock-tabs').addEventListener('keydown', (e) => {
    if (e.key !== 'ArrowRight' && e.key !== 'ArrowLeft') return;
    const i = TABS.findIndex((k) => $(`#tab-${k}`).getAttribute('aria-selected') === 'true');
    const next = TABS[(i + (e.key === 'ArrowRight' ? 1 : TABS.length - 1)) % TABS.length];
    showTab(next);
    $(`#tab-${next}`).focus();
  });

  const portName = (p) => { const n = node(p.nodeId); return n.outputs?.length > 1 ? `${n.name} · output ${p.index + 1}` : n.name; };
  function renderLoops() {
    const loops = state.robot.loops;
    $('#count-loops').textContent = loops.length || '';
    $('#count-poses').textContent = state.poses.length || '';
    $('#loops-list').innerHTML = loops.length ? `
      ${state.robot.loopGap ? '<p class="hint error">A loop can\'t close at these motor angles.</p>' : ''}
      <ul class="loops">${loops.map(({ a, b }, i) => `<li class="loop-item">
        <span class="loop-mark" aria-hidden="true">⟲</span>
        <span class="loop-ends"><button type="button" class="link-btn" data-loop-go="${i}:a">${esc(portName(a))}</button>
          <span class="loop-joint">${esc(node(a.nodeId).outputs[a.index].end)} ↔ ${esc(node(b.nodeId).outputs[b.index].end)}</span>
          <button type="button" class="link-btn" data-loop-go="${i}:b">${esc(portName(b))}</button></span>
        <button type="button" class="btn btn-small" data-loop-unjoin="${i}">Unjoin</button></li>`).join('')}</ul>`
      : '<p class="hint">No closed loops. To make one, set two link outputs to ball or hinge joints, select one, and join it to the other in Properties.</p>';
  }
  $('#loops-list').addEventListener('click', (e) => {
    const go = e.target.closest('[data-loop-go]');
    if (go) {
      const [i, end] = go.dataset.loopGo.split(':');
      if (store.isMotionView()) store.setView('cad');
      store.select({ kind: 'port', ...state.robot.loops[i][end] });
    }
    const un = e.target.closest('[data-loop-unjoin]');
    if (un) store.unjoinLoop(state.robot.loops[un.dataset.loopUnjoin].a);
  });

  // ---------------------------------------------------------------- Commands

  /** Open ports in build order. */
  const openPorts = () => store.buildOrder().map(node).flatMap((n) => n.children.map((_, i) => (store.isOpenPort(n, i) ? { nodeId: n.id, index: i } : null)).filter(Boolean));
  function nextPort() {
    const ports = openPorts();
    if (!ports.length) return;
    if (store.isMotionView()) store.setView('cad');
    const sel = state.selection;
    const i = sel?.kind === 'port' ? ports.findIndex((p) => p.nodeId === sel.nodeId && p.index === sel.index) : -1;
    store.select({ kind: 'port', ...ports[(i + 1) % ports.length] });
  }
  function selectParent() {
    const sel = state.selection, n = sel && node(sel.nodeId);
    if (!n) return;
    if (sel.kind === 'port') return store.select({ kind: 'node', nodeId: n.id });
    if (n.parent) store.select({ kind: 'node', nodeId: n.parent.nodeId });
  }
  function screenshot() {
    const a = document.createElement('a');
    a.href = viewer.screenshot();
    a.download = `${slug(state.name)}.png`;
    a.click();
  }
  function openShortcuts() {
    $('#keys-body').innerHTML = SHORTCUTS.map(([group, rows]) => `<section class="keys-group"><h3>${group}</h3>
      <dl>${rows.map(([k, what]) => `<dt><kbd>${esc(k)}</kbd></dt><dd>${esc(what)}</dd>`).join('')}</dl></section>`).join('');
    $('#keys-dialog').showModal();
  }
  $('#keys-close').addEventListener('click', () => $('#keys-dialog').close());

  const toggle = (key) => {
    prefs[key] = !prefs[key];
    savePrefs();
    applyPrefs();
  };
  function applyPrefs() {
    document.body.classList.toggle('no-timeline', !prefs.timeline);
    placeGround();
    renderCallout();
    renderToggles();
  }
  function renderToggles() {
    const on = { frames: state.showFrames, grid: prefs.grid, callout: prefs.callout, timeline: prefs.timeline };
    for (const [k, v] of Object.entries(on)) {
      $$(`.rb-tool[data-cmd="${k}"]`).forEach((b) => b.setAttribute('aria-pressed', String(v)));
      $$(`.menu-item[data-cmd="${k}"]`).forEach((b) => { b.setAttribute('role', 'menuitemcheckbox'); b.setAttribute('aria-checked', String(v)); });
    }
    $$('.menu-item[data-cmd^="view:"]').forEach((b) => { b.setAttribute('role', 'menuitemradio'); b.setAttribute('aria-checked', String(b.dataset.cmd === `view:${state.view}`)); });
    $$('[data-cmd="undo"]').forEach((b) => { b.disabled = !history.canUndo; });
    $$('[data-cmd="redo"]').forEach((b) => { b.disabled = !history.canRedo; });
    const walk = $('#btn-walk');
    $$('.menu-item[data-cmd="walk"]').forEach((b) => { b.disabled = walk.hidden; });
    const sel = state.selection;
    $$('[data-cmd="delete"]').forEach((b) => { b.disabled = !(sel?.kind === 'node' && !store.isFixed(node(sel.nodeId))); });
    $$('[data-cmd="parent"]').forEach((b) => { b.disabled = !sel; });
    $$('[data-cmd="next-port"]').forEach((b) => { b.disabled = !openPorts().length; });
  }

  const COMMANDS = {
    new: () => click('#btn-new'),
    examples: () => click('#btn-examples'),
    undo: () => history.undo(),
    redo: () => history.redo(),
    parent: selectParent,
    'next-port': nextPort,
    deselect: () => store.select(null),
    // The inspector's own Delete button, so a part with others attached still asks first.
    delete: () => { if (state.selection?.kind === 'node') click('#inspector [data-act="delete"]'); },
    fit: () => viewer.fit(),
    frames: () => store.setShowFrames(!state.showFrames),
    grid: () => toggle('grid'),
    callout: () => toggle('callout'),
    timeline: () => toggle('timeline'),
    data: () => click('#btn-data'),
    tree: () => click('#btn-tree-view'),
    workspace: () => {
      store.setView('sim');
      requestAnimationFrame(() => { const b = $('[data-ws="reach"]'); if (b && b.getAttribute('aria-pressed') !== 'true') b.click(); });
    },
    walk: () => click('#btn-walk'),
    zero: () => { showTab('poses'); click('[data-pose-zero]'); },
    'save-pose': () => { showTab('poses'); click('[data-pose-save]'); },
    'export-step': () => click('#btn-export-step'),
    'export-urdf': () => click('#btn-export-urdf'),
    screenshot,
    shortcuts: openShortcuts,
  };
  function run(cmd) {
    if (cmd.startsWith('view:')) return store.setView(cmd.slice(5));
    if (cmd.startsWith('cam:')) return snapTo(cmd.slice(4));
    COMMANDS[cmd]?.();
  }
  document.addEventListener('click', (e) => {
    const b = e.target.closest('[data-cmd]');
    if (b && !b.disabled) run(b.dataset.cmd);
  });

  // ---------------------------------------------------------------- Keyboard

  const VIEW_KEYS = { 1: 'cad', 2: 'cadfile', 3: 'simple', 4: 'sim', 5: 'anim' };
  const CAM_KEYS = { 0: 'iso', 7: 'top', 8: 'front', 9: 'right' };
  document.addEventListener('keydown', (e) => {
    const typing = e.target.closest?.('input, select, textarea, [contenteditable]');
    const ctrl = e.ctrlKey || e.metaKey;
    const k = e.key.toLowerCase();
    if (ctrl && !e.altKey) {
      if (typing && k !== 's' && k !== 'o') return; // let text fields keep their own undo
      if (k === 'z' && !e.shiftKey) { e.preventDefault(); history.undo(); }
      else if (k === 'y' || (k === 'z' && e.shiftKey)) { e.preventDefault(); history.redo(); }
      else if (k === 's') { e.preventDefault(); click('#btn-file'); }
      else if (k === 'o') { e.preventDefault(); click('#btn-import'); }
      return;
    }
    if (typing || document.querySelector('dialog[open]:modal')) return;
    if (e.altKey) {
      const alt = { n: 'new', e: 'examples', d: 'data', p: 'screenshot', f: 'frames' }[k];
      if (alt) { e.preventDefault(); run(alt); }
      else if (e.key === 'ArrowUp') { e.preventDefault(); selectParent(); }
      return;
    }
    if (VIEW_KEYS[e.key]) store.setView(VIEW_KEYS[e.key]);
    else if (CAM_KEYS[e.key]) snapTo(CAM_KEYS[e.key]);
    else if (k === 'g') toggle('grid');
    else if (k === 'l') toggle('callout');
    else if (k === 't') toggle('timeline');
    else if (k === 'w') click('#btn-walk');
    else if (e.key === '?') openShortcuts();
    else if (e.key === ' ' && state.view === 'anim' && !e.target.closest?.('button')) { e.preventDefault(); animator.toggle(); }
    else if (e.key === 'Tab' && !e.shiftKey && (e.target === document.body || e.target.tagName === 'CANVAS')) { e.preventDefault(); nextPort(); }
  });

  // ---------------------------------------------------------------- Keeping up

  const dirty = $('#doc-dirty');
  let dirtyFrame = 0;
  const renderDirty = () => {
    cancelAnimationFrame(dirtyFrame);
    dirtyFrame = requestAnimationFrame(() => { dirty.hidden = !history.dirty; });
  };
  store.subscribe((kind) => {
    solve();
    if (kind === 'structure') { groundZ = groundHeight(); placeGround(); }
    renderCallout();
    renderTags();
    renderLoops();
    renderToggles();
    renderDirty();
  });
  history.onChange(() => { renderToggles(); renderDirty(); });
  animator.onChange(() => { renderTags(); renderToggles(); });
  document.addEventListener('input', (e) => { if (e.target.id === 'project-name') renderDirty(); });

  solve();
  groundZ = groundHeight();
  applyPrefs();
  renderTags();
  renderLoops();
  drawCube();
  return { showTab };
}
