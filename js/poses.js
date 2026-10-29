// Saved poses: a named set of joint angles, listed above the Assembly list (whose actuator rows hold the joint sliders). Clicking one moves
// every joint there together, each no faster than its speed limit; Save pose stores the current
// angles. Poses are saved in the robot's JSON file as `poses: [{ name, q: { actuatorId: deg } }]`.
import * as store from './robot.js';
import { state, node } from './robot.js';
import { forwardKinematics, solveLinks } from './kinematics.js';
import { jointSpeed } from './simulate.js';

const MIN_TIME = 0.6; // s: even a small move takes this long, so it reads as motion
const $ = (s) => document.querySelector(s);
const $$ = (s) => [...document.querySelectorAll(s)];
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));

const actuators = () => store.buildOrder().map(node).filter((n) => n.type === 'actuator');

/** True when every joint the pose names is at its angle. */
const isAt = (p) => Object.entries(p.q).every(([id, v]) => !node(id) || Math.abs(node(id).q - v) < 0.05);

export function initPoses(viewer) {
  const host = $('#poses');
  let naming = false;
  let tween = null; // { joints: [{ n, from, to }], t, time }

  function render() {
    const acts = actuators().length;
    if (!acts) { host.innerHTML = ''; return; }
    const list = state.poses.length
      ? `<div class="pose-list">${state.poses.map((p, i) => `
          <span class="pose-chip">
            <button type="button" class="btn btn-small pose-go" data-pose="${i}" aria-pressed="${isAt(p)}" title="Move to ${esc(p.name)}">${esc(p.name)}</button>
            <button type="button" class="pose-del" data-pose-del="${i}" aria-label="Delete ${esc(p.name)}" title="Delete this pose">&times;</button>
          </span>`).join('')}</div>`
      : '<p class="hint">Set the joints, then Save pose to keep the angles.</p>';
    const form = naming
      ? `<form class="pose-form">
          <input type="text" id="pose-name" aria-label="Pose name" value="${esc(`Pose ${state.poses.length + 1}`)}" maxlength="40">
          <button type="submit" class="btn btn-small btn-primary">Save</button>
          <button type="button" class="btn btn-small" data-pose-cancel>Cancel</button>
        </form>`
      : '';
    host.innerHTML = `
      <div class="poses-head"><span class="label">Poses</span>
        <span class="poses-tools">
          <button type="button" class="btn btn-ghost btn-small" data-pose-zero title="Put every joint back to 0°">Zero all</button>
          ${naming ? '' : '<button type="button" class="btn btn-ghost btn-small" data-pose-save title="Save the current joint angles as a pose">Save pose</button>'}
        </span>
      </div>${form}${list}`;
    if (naming) { const input = $('#pose-name'); input.focus(); input.select(); }
  }

  const markCurrent = () => $$('[data-pose]').forEach((b) => b.setAttribute('aria-pressed', String(isAt(state.poses[b.dataset.pose]))));

  function goTo(p) {
    const joints = actuators().filter((n) => n.id in p.q).map((n) => ({ n, from: n.q, to: p.q[n.id] }));
    const time = Math.max(MIN_TIME, ...joints.map((j) => Math.abs(j.to - j.from) / jointSpeed(j.n)));
    tween = { joints, t: 0, time };
  }

  viewer.onFrame((dt) => {
    if (!tween) return;
    tween.t = Math.min(tween.time, tween.t + dt);
    const u = tween.t / tween.time, s = u * u * (3 - 2 * u); // ease in and out
    for (const { n, from, to } of tween.joints) {
      n.q = Math.round((from + (to - from) * s) * 10) / 10;
      $$(`[data-q="${n.id}"]`).forEach((el) => { el.value = n.q; });
      $$(`[data-q-out="${n.id}"]`).forEach((el) => { el.textContent = `${Math.round(n.q)}°`; });
    }
    if (u < 1) { viewer.pose(forwardKinematics(state.robot, solveLinks(state.robot))); return; }
    for (const { n, to } of tween.joints) n.q = to;
    tween = null;
    store.emit('params'); // everything else catches up once the move ends
  });

  host.addEventListener('click', (e) => {
    const t = e.target.closest('button');
    if (!t) return;
    if (t.dataset.pose != null) goTo(state.poses[t.dataset.pose]);
    else if ('poseZero' in t.dataset) goTo({ q: Object.fromEntries(actuators().map((n) => [n.id, 0])) });
    else if (t.dataset.poseDel != null) store.deletePose(Number(t.dataset.poseDel));
    else if ('poseSave' in t.dataset) { naming = true; render(); }
    else if ('poseCancel' in t.dataset) { naming = false; render(); }
  });
  host.addEventListener('submit', (e) => {
    e.preventDefault();
    const name = $('#pose-name').value.trim();
    if (!name) return;
    naming = false;
    store.savePose(name);
  });
  host.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && naming) { e.stopPropagation(); naming = false; render(); }
  });
  // Moving a slider by hand stops a move in progress.
  document.addEventListener('input', (e) => { if (e.target.dataset?.q) tween = null; });

  store.subscribe((kind) => {
    if (kind === 'structure') { tween = null; naming = false; render(); }
    else if (kind === 'poses') render();
    else if (kind === 'params') markCurrent();
  });
}
