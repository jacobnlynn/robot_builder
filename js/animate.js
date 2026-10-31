// Animations: a series of saved poses, played in order. The Animate view (beside Simulate) puts the
// controls in the right-hand panel: pick or start an animation, add saved poses as its steps, set how
// long each one holds, put them in order (drag the grip, or the arrows), then Play. Between two poses
// every joint moves at once, none faster than its top speed times the animation's speed setting, so
// the joint with the furthest to go (for its speed) sets how long the move takes.
// Saved in the robot's JSON file as `animations: [{ name, loop, speed, pause, walk, steps: [{ pose, hold }] }]`:
// a step names a saved pose, so saving that pose again changes the animation too. The legged examples
// each mark one animation `walk: true`; the Walk button in the 3D view (shown only then) plays it.
import * as store from './robot.js';
import { state, node } from './robot.js';
import { forwardKinematics, solveLinks } from './kinematics.js';
import { jointSpeed } from './simulate.js';

const MIN_MOVE = 0.4; // s at 100% speed: even a small move takes this long, so it reads as motion
const EASE_PEAK = 1.5; // an eased move peaks at 1.5× its average speed, so it takes that much longer
const SPEEDS = [10, 25, 50, 100]; // % of each joint's top speed
const HOLD = 0.3; // s: the hold a new step starts with
const $ = (s) => document.querySelector(s);
const $$ = (s) => [...document.querySelectorAll(s)];
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const secs = (t) => `${t.toFixed(1)} s`;

const actuators = () => store.buildOrder().map(node).filter((n) => n.type === 'actuator');
const anglesNow = () => Object.fromEntries(actuators().map((n) => [n.id, n.q]));
const blank = (name) => ({ name, loop: true, speed: 50, pause: true, steps: [] });

/** Seconds to move every joint from `from` to `to` at `speed` % of top speed. */
function moveTime(from, to, a) {
  const slowest = Math.max(0, ...actuators().map((n) => Math.abs(to[n.id] - from[n.id]) / jointSpeed(n)));
  return Math.max(MIN_MOVE, slowest * (a.pause ? EASE_PEAK : 1)) * 100 / a.speed;
}

/**
 * The animation laid out in time: a hold at each step's pose, then a move to the next (and, when it
 * loops, back to the first). `start` gives the angles of joints that no pose names.
 * Returns { segs: [{ t0, dur, from, to, step, move }], total, first }.
 */
function timeline(a, start) {
  const poses = new Map(state.poses.map((p) => [p.name, p]));
  const keys = [];
  let prev = start;
  a.steps.forEach((s, step) => {
    const p = poses.get(s.pose);
    if (!p) return; // the pose was deleted: skip the step
    const q = { ...prev };
    for (const id of Object.keys(q)) if (id in p.q) q[id] = p.q[id];
    keys.push({ q, hold: s.hold, step });
    prev = q;
  });
  const segs = [];
  let t = 0;
  keys.forEach((k, j) => {
    segs.push({ t0: t, dur: k.hold, from: k.q, to: k.q, step: k.step });
    t += k.hold;
    const last = j === keys.length - 1;
    if (last && !(a.loop && keys.length > 1)) return;
    const next = keys[last ? 0 : j + 1];
    // Looping back: a joint a whole number of turns from where it started is already there.
    const to = last ? Object.fromEntries(Object.entries(next.q).map(([id, v]) => [id, (k.q[id] - v) % 360 === 0 ? k.q[id] : v])) : next.q;
    const dur = moveTime(k.q, to, a);
    segs.push({ t0: t, dur, from: k.q, to, step: next.step, move: true });
    t += dur;
  });
  return { segs, total: t, first: keys[0]?.q ?? null };
}

/** The angles, and the step being shown, `t` seconds into a timeline. */
function sample(track, t, ease) {
  const seg = track.segs.find((s) => t < s.t0 + s.dur) ?? track.segs.at(-1);
  const u = seg.dur ? Math.min(1, Math.max(0, (t - seg.t0) / seg.dur)) : 1;
  const k = seg.move && ease ? u * u * (3 - 2 * u) : u;
  const q = {};
  for (const id of Object.keys(seg.from)) q[id] = seg.from[id] + (seg.to[id] - seg.from[id]) * k;
  return { q, step: seg.step };
}

export function initAnimate(viewer) {
  const host = $('#inspector');
  let current = 0; // which animation (index into state.animations)
  let naming = null; // null, or 'new' / 'copy' while the name form is open
  let track = null; // timeline(...) of the current animation
  let start = null; // angles the timeline starts from (for joints no pose names)
  let t = 0; // s into the timeline
  let playing = false;
  let lead = null; // { from, to, time, t }: the move to the first pose before playing from the start
  let shownStep = -1;
  let grabbed = false; // pointer is down on a step's grip, so the row may be dragged
  let dragFrom = null;
  let shown = false; // the Animate view is open
  let robot = null; // the robot those belong to (a new one starts on its first animation)

  const anim = () => state.animations[current];
  const active = () => state.view === 'anim';
  const walkIndex = () => state.animations.findIndex((a) => a.walk);
  const walking = () => playing && current === walkIndex();

  /** Walk button: only for a robot with a walk; pressed while the walk plays. */
  function renderWalk() {
    const b = $('#btn-walk');
    b.hidden = walkIndex() < 0;
    b.setAttribute('aria-pressed', String(walking()));
    b.innerHTML = walking() ? '&#10074;&#10074; Stop walking' : '&#9654; Walk';
  }

  function rebuild() {
    const a = anim();
    if (!start || (!playing && t === 0)) start = anglesNow();
    track = a ? timeline(a, start) : null;
    if (track) t = Math.min(t, track.total);
  }

  /** Put the joints at `q`: the sliders and the 3D view follow, the rest catches up on 'params'. */
  function apply(q) {
    for (const n of actuators()) {
      if (!(n.id in q)) continue;
      n.q = Math.round(q[n.id] * 10) / 10;
      $$(`[data-q="${n.id}"]`).forEach((el) => { el.value = n.q; });
      $$(`[data-q-out="${n.id}"]`).forEach((el) => { el.textContent = `${Math.round(n.q)}°`; });
    }
    viewer.pose(forwardKinematics(state.robot, solveLinks(state.robot)));
  }

  // ---- Panel

  function stepRow(s, i, a) {
    const known = state.poses.some((p) => p.name === s.pose);
    const options = state.poses.map((p) => `<option${p.name === s.pose ? ' selected' : ''}>${esc(p.name)}</option>`).join('')
      + (known ? '' : `<option selected value="${esc(s.pose)}">${esc(s.pose)} (deleted)</option>`);
    return `<li class="stack-item anim-step${known ? '' : ' is-missing'}${i === shownStep ? ' is-current' : ''}" draggable="true" data-step="${i}">
      <span class="anim-grip" aria-hidden="true" title="Drag to move this step">⋮⋮</span>
      <span class="anim-num">${i + 1}</span>
      <select data-step-pose="${i}" aria-label="Step ${i + 1} pose">${options}</select>
      <input type="number" class="anim-hold" data-step-hold="${i}" min="0" max="60" step="0.1" value="${s.hold}" aria-label="Step ${i + 1} hold, seconds" title="Hold this pose for this many seconds">
      <span class="hint">s</span>
      <button type="button" class="icon-btn anim-move" data-step-up="${i}" aria-label="Move step ${i + 1} up"${i ? '' : ' disabled'}>↑</button>
      <button type="button" class="icon-btn anim-move" data-step-down="${i}" aria-label="Move step ${i + 1} down"${i < a.steps.length - 1 ? '' : ' disabled'}>↓</button>
      <button type="button" class="icon-btn" data-step-del="${i}" aria-label="Remove step ${i + 1}">&times;</button>
    </li>`;
  }

  function render() {
    if (!active()) return;
    const list = state.animations;
    const a = anim();
    const form = naming ? `<form class="anim-form">
        <input type="text" id="anim-new-name" aria-label="Animation name" maxlength="40"
          value="${esc(naming === 'copy' && a ? `${a.name} copy` : `Animation ${list.length + 1}`)}">
        <button type="submit" class="btn btn-small btn-primary">${naming === 'copy' ? 'Save' : 'Create'}</button>
        <button type="button" class="btn btn-small" data-anim="cancel">Cancel</button>
      </form>` : '';
    const total = track?.total ?? 0;
    host.innerHTML = `
      <div class="insp-section insp-head">
        <div class="eyebrow">Animate</div>
        <h2 class="insp-title">Play a series of poses</h2>
        <p class="muted">Add saved poses as steps, put them in order, then Play. Every joint moves at once,
        none faster than its top speed.</p>
      </div>
      <div class="insp-section insp-body">
        <div class="field">
          <div class="label-row"><label for="anim-pick">Animation</label>
            <span class="hint">${list.length ? `${list.length} saved with this robot` : 'none yet'}</span></div>
          <div class="anim-row">
            ${list.length ? `<select id="anim-pick">${list.map((x, i) => `<option value="${i}"${i === current ? ' selected' : ''}>${esc(x.name)}</option>`).join('')}</select>` : '<span class="empty anim-none">No animations yet.</span>'}
            ${naming ? '' : `<button type="button" class="btn btn-small" data-anim="new" title="Start an empty animation">New</button>
            ${a ? '<button type="button" class="btn btn-small" data-anim="copy" title="Save these steps as a new animation">Save as&hellip;</button>' : ''}`}
          </div>
          ${form}
        </div>
        ${a ? `
        <div class="field">
          <label for="anim-name">Name</label>
          <div class="anim-row">
            <input type="text" id="anim-name" value="${esc(a.name)}" maxlength="40">
            <button type="button" class="btn btn-small btn-ghost anim-delete" data-anim="delete" title="Delete this animation">Delete</button>
          </div>
        </div>
        <div class="anim-transport">
          <button type="button" class="btn btn-primary anim-play" data-anim="play"${track?.first ? '' : ' disabled'}>${playing ? '❚❚ Pause' : '▶ Play'}</button>
          <button type="button" class="btn" data-anim="stop" title="Back to the first pose"${track?.first ? '' : ' disabled'}>■ Stop</button>
          <button type="button" class="btn anim-loop" data-anim="loop" aria-pressed="${a.loop}" title="Play again from the start when it ends">⟲ Loop ${a.loop ? 'on' : 'off'}</button>
        </div>
        <div class="anim-scrub">
          <input type="range" id="anim-scrub" min="0" max="${total}" step="0.01" value="${t}" aria-label="Time"${total ? '' : ' disabled'}>
          <output id="anim-time">${secs(t)} / ${secs(total)}</output>
        </div>
        <div class="anim-options">
          <label class="anim-option">Speed
            <select id="anim-speed">${SPEEDS.concat(SPEEDS.includes(a.speed) ? [] : [a.speed]).sort((x, y) => x - y)
              .map((v) => `<option value="${v}"${v === a.speed ? ' selected' : ''}>${v}% of top speed</option>`).join('')}</select>
          </label>
          <label class="anim-option"><input type="checkbox" id="anim-pause"${a.pause ? ' checked' : ''}> Ease to a stop at each pose</label>
        </div>
        <div class="field">
          <div class="label-row"><span class="label">Steps</span><span class="hint">Hold each pose for this many seconds</span></div>
          ${a.steps.length ? `<ol class="stack anim-steps">${a.steps.map((s, i) => stepRow(s, i, a)).join('')}</ol>`
            : '<p class="empty">No steps yet. Add poses below.</p>'}
        </div>
        <div class="field">
          <span class="label">Add a step</span>
          ${state.poses.length ? `<div class="pose-list">${state.poses.map((p) => `<button type="button" class="btn btn-small" data-add-pose="${esc(p.name)}" title="Add ${esc(p.name)} at the end">+ ${esc(p.name)}</button>`).join('')}</div>`
            : '<p class="hint">This robot has no saved poses. Set the joints with the sliders on the left, then Save pose.</p>'}
        </div>` : `<p class="hint">An animation is a list of saved poses. Click New to start one.</p>`}
      </div>`;
    if (naming) { const input = $('#anim-new-name'); input.focus(); input.select(); }
  }

  /** Only what changes while playing: the time, the scrubber, the button and the current step. */
  function renderTransport(step) {
    const scrub = $('#anim-scrub');
    if (scrub && document.activeElement !== scrub) scrub.value = t;
    const time = $('#anim-time');
    if (time) time.textContent = `${secs(t)} / ${secs(track?.total ?? 0)}`;
    const play = $('.anim-play');
    if (play) play.textContent = playing ? '❚❚ Pause' : '▶ Play';
    renderWalk();
    if (step !== shownStep) {
      shownStep = step;
      $$('.anim-step').forEach((li) => li.classList.toggle('is-current', Number(li.dataset.step) === step));
    }
  }

  /** The animation changed: lay it out again and redraw the panel. */
  function edited() {
    rebuild();
    render();
    renderWalk();
  }

  // ---- Playback

  function play() {
    if (!track?.first) return;
    if (t >= track.total && !anim().loop) t = 0;
    if (t === 0) {
      start = anglesNow();
      rebuild();
      const from = anglesNow();
      const moved = Object.keys(from).some((id) => Math.abs(from[id] - track.first[id]) > 0.05);
      lead = moved ? { from, to: track.first, time: moveTime(from, track.first, anim()), t: 0 } : null;
    }
    playing = true;
    renderTransport(shownStep);
  }

  function pause() {
    if (!playing) return;
    playing = false;
    lead = null;
    renderTransport(shownStep);
    store.emit('params'); // everything else catches up once the motion stops
  }

  function stop() {
    playing = false;
    lead = null;
    t = 0;
    if (track?.first) apply(track.first);
    renderTransport(track?.first ? sample(track, 0, false).step : -1);
    store.emit('params');
  }

  viewer.onFrame((dt) => {
    if (!playing || !track) return;
    const a = anim();
    if (lead) {
      lead.t = Math.min(lead.time, lead.t + dt);
      const u = lead.t / lead.time, k = u * u * (3 - 2 * u);
      apply(Object.fromEntries(Object.keys(lead.to).map((id) => [id, lead.from[id] + (lead.to[id] - lead.from[id]) * k])));
      if (u >= 1) lead = null;
      renderTransport(-1);
      return;
    }
    t += dt;
    let done = false;
    if (t >= track.total) {
      if (a.loop && track.total > 0) t %= track.total;
      else { t = track.total; done = true; }
    }
    const s = sample(track, t, a.pause);
    apply(s.q);
    if (done) { playing = false; store.emit('params'); }
    renderTransport(s.step);
  });

  // ---- Events

  host.addEventListener('click', (e) => {
    if (!active()) return;
    const b = e.target.closest('button');
    if (!b) return;
    const a = anim();
    const d = b.dataset;
    const swap = (i, j) => { [a.steps[i], a.steps[j]] = [a.steps[j], a.steps[i]]; edited(); };
    if (d.anim === 'play') playing ? pause() : play();
    else if (d.anim === 'stop') stop();
    else if (d.anim === 'loop') { a.loop = !a.loop; edited(); }
    else if (d.anim === 'new' || d.anim === 'copy') { naming = d.anim; render(); }
    else if (d.anim === 'cancel') { naming = null; render(); }
    else if (d.anim === 'delete') {
      pause();
      state.animations.splice(current, 1);
      current = Math.max(0, Math.min(current, state.animations.length - 1));
      t = 0;
      edited();
    } else if (d.addPose != null) { a.steps.push({ pose: d.addPose, hold: HOLD }); edited(); }
    else if (d.stepUp != null) swap(Number(d.stepUp), Number(d.stepUp) - 1);
    else if (d.stepDown != null) swap(Number(d.stepDown), Number(d.stepDown) + 1);
    else if (d.stepDel != null) { a.steps.splice(Number(d.stepDel), 1); edited(); }
  });

  host.addEventListener('submit', (e) => {
    if (!active()) return;
    e.preventDefault();
    const name = $('#anim-new-name').value.trim();
    if (!name) return;
    const a = anim();
    const copy = naming === 'copy' && a ? { ...structuredClone(a), name } : blank(name);
    delete copy.walk; // the robot keeps one walk
    state.animations.push(copy);
    current = state.animations.length - 1;
    naming = null;
    pause();
    t = 0;
    edited();
  });

  host.addEventListener('change', (e) => {
    if (!active()) return;
    const el = e.target, a = anim();
    if (el.id === 'anim-pick') { pause(); current = Number(el.value); t = 0; edited(); }
    else if (el.id === 'anim-name') { a.name = el.value.trim() || a.name; render(); }
    else if (el.id === 'anim-speed') { a.speed = Number(el.value); edited(); }
    else if (el.id === 'anim-pause') { a.pause = el.checked; edited(); }
    else if (el.dataset.stepPose != null) { a.steps[el.dataset.stepPose].pose = el.value; edited(); }
    else if (el.dataset.stepHold != null) { a.steps[el.dataset.stepHold].hold = Math.min(60, Math.max(0, Number(el.value) || 0)); edited(); }
    else if (el.id === 'anim-scrub') store.emit('params');
  });

  host.addEventListener('input', (e) => {
    if (!active() || e.target.id !== 'anim-scrub') return;
    playing = false;
    lead = null;
    t = Number(e.target.value);
    const s = sample(track, t, anim().pause);
    apply(s.q);
    renderTransport(s.step);
  });

  host.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && naming) { e.stopPropagation(); naming = null; render(); }
  });

  // Reorder by dragging a step's grip onto another step.
  host.addEventListener('pointerdown', (e) => { grabbed = !!e.target.closest('.anim-grip'); });
  host.addEventListener('dragstart', (e) => {
    const li = e.target.closest?.('.anim-step');
    if (!li || !grabbed) { e.preventDefault(); return; }
    dragFrom = Number(li.dataset.step);
    e.dataTransfer.effectAllowed = 'move';
    e.dataTransfer.setData('text/plain', String(dragFrom));
    li.classList.add('is-dragging');
  });
  host.addEventListener('dragover', (e) => {
    const li = e.target.closest('.anim-step');
    if (dragFrom == null || !li) return;
    e.preventDefault();
    $$('.anim-step').forEach((x) => x.classList.toggle('drop-target', x === li));
  });
  host.addEventListener('drop', (e) => {
    const li = e.target.closest('.anim-step');
    if (dragFrom == null || !li) return;
    e.preventDefault();
    const steps = anim().steps;
    const [s] = steps.splice(dragFrom, 1);
    steps.splice(Number(li.dataset.step), 0, s);
    dragFrom = null;
    edited();
  });
  host.addEventListener('dragend', () => {
    dragFrom = null;
    grabbed = false;
    $$('.anim-step').forEach((x) => x.classList.remove('is-dragging', 'drop-target'));
  });

  // Walk: switch to Animate and play the robot's walk from the start; again to stop.
  $('#btn-walk').addEventListener('click', () => {
    if (walking()) { pause(); return; }
    const w = walkIndex();
    if (w < 0) return;
    if (!active()) store.setView('anim');
    playing = false;
    lead = null;
    current = w;
    naming = null;
    t = 0;
    edited();
    play();
  });

  // Moving a joint by hand, or going to a pose in the Poses list, pauses the animation.
  document.addEventListener('input', (e) => { if (e.target.dataset?.q) pause(); });
  $('#poses').addEventListener('click', (e) => { if (e.target.closest('button')) pause(); });

  store.subscribe((kind) => {
    if (kind === 'structure') {
      // A new robot starts on its first animation; a changed one keeps its place.
      playing = false;
      lead = null;
      t = 0;
      start = null;
      naming = null;
      if (state.robot !== robot || !state.animations[current]) current = 0;
      robot = state.robot;
      rebuild();
      render();
      renderWalk();
    } else if (kind === 'view') {
      // 'view' also fires for other display changes; only redraw on switching to Animate.
      if (active() === shown) return;
      shown = active();
      if (!shown) { pause(); return; }
      rebuild();
      render();
    } else if (kind === 'poses' || kind === 'animations') edited();
  });
}
