// Timeline, under the 3D view: the current animation laid out in time. A track for every joint the
// animation moves draws that joint's angle as a curve, with a diamond at each pose; the playhead
// shows where playback is. Click or drag across the tracks to scrub. The controls along the top play,
// stop and loop it and set its speed, the same as the Animate panel (which edits the steps).
import * as store from './robot.js';
import { state, node } from './robot.js';
import { sample } from './animate.js';

const $ = (s, root = document) => root.querySelector(s);
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const SPEEDS = [10, 25, 50, 100];
const SAMPLES = 160; // points per curve
const MOVES = 0.5; // degrees: a joint that moves less than this gets no track
const COLLAPSE_KEY = 'rbpro.timeline.collapsed';

export function initTimeline(viewer, animator) {
  const host = $('#timeline');
  let collapsed = false;
  try { collapsed = localStorage.getItem(COLLAPSE_KEY) === '1'; } catch { /* storage blocked */ }
  let scrubbing = false;

  const actuators = () => store.buildOrder().map(node).filter((n) => n.type === 'actuator');
  const secs = (t) => `${t.toFixed(2)}`;

  /** Each moving joint's curve: points across the timeline and its keyframes. */
  function tracks() {
    const track = animator.track, a = animator.anim;
    if (!track?.first || !track.total) return [];
    const ts = Array.from({ length: SAMPLES + 1 }, (_, i) => track.total * i / SAMPLES);
    const at = ts.map((t) => sample(track, t, a.pause).q);
    const keys = track.segs.filter((s) => !s.move).map((s) => s.t0);
    if (a.loop) keys.push(track.total);
    return actuators().map((n) => {
      const vals = at.map((q) => q[n.id]);
      const lo = Math.min(...vals), hi = Math.max(...vals);
      if (!(hi - lo >= MOVES)) return null;
      const pad = (hi - lo) * 0.12;
      const y = (v) => 3 + (1 - (v - lo + pad) / (hi - lo + 2 * pad)) * 20; // in a 26-unit lane
      return {
        n, lo, hi,
        points: vals.map((val, i) => `${(i / SAMPLES * 1000).toFixed(1)},${y(val).toFixed(2)}`).join(' '),
        keys: keys.map((t) => ({ x: t / track.total * 100, y: y(sample(track, Math.min(t, track.total - 1e-6), a.pause).q[n.id]) })),
      };
    }).filter(Boolean);
  }

  let shown = null; // what was last drawn, so playing and scrubbing only move the playhead
  function render(force = false) {
    const list = state.animations, a = animator.anim, track = animator.track;
    const sig = JSON.stringify([animator.index, a, track?.total, collapsed, list.length, state.poses]);
    if (!force && sig === shown && host.firstChild) return renderClock();
    shown = sig;
    host.classList.toggle('is-collapsed', collapsed);
    if (!list.length) {
      host.innerHTML = `<div class="tl-head"><span class="tl-title">Timeline</span>
        <span class="hint">No animations yet. Save some poses, then build one in Animate.</span>
        <span class="tl-spacer"></span><button type="button" class="btn btn-small" data-cmd="view:anim">Open Animate</button></div>`;
      return;
    }
    const total = track?.total ?? 0;
    const rows = collapsed ? [] : tracks();
    const steps = a.steps.map((s) => s.pose);
    const ticks = total ? [0, 0.25, 0.5, 0.75, 1] : [];
    host.innerHTML = `
      <div class="tl-head">
        <button type="button" class="tl-btn tl-fold" data-tl="collapse" aria-expanded="${!collapsed}" aria-label="${collapsed ? 'Show' : 'Hide'} the tracks">
          <svg viewBox="0 0 12 12" aria-hidden="true"><path d="${collapsed ? 'M4 2l4 4-4 4' : 'M2 4l4 4 4-4'}"></path></svg></button>
        <span class="tl-title">Timeline</span>
        <label class="sr-only" for="tl-anim">Animation</label>
        <select id="tl-anim" class="tl-select">${list.map((x, i) => `<option value="${i}"${i === animator.index ? ' selected' : ''}>${esc(x.name)}</option>`).join('')}</select>
        <div class="tl-transport" role="group" aria-label="Transport">
          <button type="button" class="tl-btn" data-tl="stop" aria-label="Back to the first pose" title="Back to the first pose"${track?.first ? '' : ' disabled'}>
            <svg viewBox="0 0 12 12" aria-hidden="true"><path d="M2 2v8M10 2L4 6l6 4z"></path></svg></button>
          <button type="button" class="tl-btn tl-play" data-tl="play" aria-label="Play" title="Play / pause (Space in Animate)"${track?.first ? '' : ' disabled'}></button>
          <button type="button" class="tl-btn" data-tl="loop" aria-pressed="${a.loop}" aria-label="Loop" title="Loop">
            <svg viewBox="0 0 14 12" aria-hidden="true"><path d="M3 4h7a2.5 2.5 0 0 1 0 5H4"></path><path d="M5 2L3 4l2 2"></path></svg></button>
        </div>
        <output class="tl-time mono" id="tl-time">${secs(animator.t)} / ${secs(total)} s</output>
        <label class="tl-field">Speed
          <select id="tl-speed" class="tl-select">${SPEEDS.concat(SPEEDS.includes(a.speed) ? [] : [a.speed]).sort((x, y) => x - y)
            .map((v) => `<option value="${v}"${v === a.speed ? ' selected' : ''}>${v}%</option>`).join('')}</select></label>
        <label class="tl-field"><input type="checkbox" id="tl-ease"${a.pause ? ' checked' : ''}> Ease at poses</label>
        <span class="tl-spacer"></span>
        <span class="tl-steps" title="Steps, in order">${steps.length ? steps.map(esc).join(' · ') : 'No steps'}</span>
        <button type="button" class="btn btn-small" data-cmd="view:anim" title="Edit the steps in the Animate panel">Edit steps</button>
      </div>
      ${collapsed ? '' : `<div class="tl-body">
        <div class="tl-row tl-ruler"><span class="tl-label">Joint</span>
          <div class="tl-lane">${ticks.map((f) => `<span class="tl-tick" style="left:${f * 100}%">${secs(f * total)}</span>`).join('')}</div></div>
        <div class="tl-tracks">${rows.length ? rows.map((r, k) => `<div class="tl-row" style="--c: var(--tl-${k % 6})">
            <span class="tl-label" title="${esc(r.n.name)}: ${r.lo.toFixed(1)}° to ${r.hi.toFixed(1)}°">${esc(r.n.name || 'Joint')}</span>
            <div class="tl-lane">
              <svg viewBox="0 0 1000 26" preserveAspectRatio="none" aria-hidden="true"><polyline points="${r.points}"></polyline></svg>
              ${r.keys.map((p) => `<span class="tl-key" style="left:${p.x}%;top:${(p.y / 26 * 100).toFixed(1)}%"></span>`).join('')}
            </div></div>`).join('')
          : `<p class="hint tl-empty">${track?.first ? 'Nothing moves: every step is the same pose.' : 'Add poses as steps in Animate to see the curves.'}</p>`}
          ${rows.length ? '<div class="tl-playhead-wrap" aria-hidden="true"><div class="tl-playhead" id="tl-playhead"></div></div>' : ''}
        </div>
      </div>`}`;
    renderClock();
  }

  /** What changes as it plays: the time, the playhead and the play button. */
  function renderClock() {
    const total = animator.track?.total ?? 0;
    const out = $('#tl-time');
    if (out) out.textContent = `${secs(animator.t)} / ${secs(total)} s`;
    const ph = $('#tl-playhead');
    if (ph) ph.style.left = `${total ? animator.t / total * 100 : 0}%`;
    const play = $('.tl-play');
    if (play) {
      const on = animator.playing;
      if (play.dataset.on !== String(on)) {
        play.dataset.on = String(on);
        play.setAttribute('aria-label', on ? 'Pause' : 'Play');
        play.innerHTML = on ? '<svg viewBox="0 0 12 12" aria-hidden="true"><path d="M3 2v8M9 2v8"></path></svg>'
          : '<svg viewBox="0 0 12 12" aria-hidden="true"><path d="M3 1.5l7 4.5-7 4.5z" class="fill"></path></svg>';
      }
    }
  }
  viewer.onFrame(renderClock);

  host.addEventListener('click', (e) => {
    const b = e.target.closest('[data-tl]');
    if (!b || b.disabled) return;
    const act = b.dataset.tl;
    if (act === 'play') animator.toggle();
    else if (act === 'stop') animator.stop();
    else if (act === 'loop') animator.set({ loop: !animator.anim.loop });
    else if (act === 'collapse') {
      collapsed = !collapsed;
      try { localStorage.setItem(COLLAPSE_KEY, collapsed ? '1' : '0'); } catch { /* storage blocked */ }
      render(true);
    }
  });
  host.addEventListener('change', (e) => {
    if (e.target.id === 'tl-anim') animator.select(Number(e.target.value));
    else if (e.target.id === 'tl-speed') animator.set({ speed: Number(e.target.value) });
    else if (e.target.id === 'tl-ease') animator.set({ pause: e.target.checked });
  });

  // Scrub: press on the tracks (or the ruler) and drag.
  const seekAt = (e) => {
    const lane = host.querySelector('.tl-tracks .tl-lane, .tl-ruler .tl-lane');
    const total = animator.track?.total;
    if (!lane || !total) return;
    const r = lane.getBoundingClientRect();
    animator.seek(Math.min(1, Math.max(0, (e.clientX - r.left) / r.width)) * total);
  };
  host.addEventListener('pointerdown', (e) => {
    if (!e.target.closest('.tl-lane') || e.button !== 0) return;
    scrubbing = true;
    host.setPointerCapture(e.pointerId);
    seekAt(e);
  });
  host.addEventListener('pointermove', (e) => { if (scrubbing) seekAt(e); });
  host.addEventListener('pointerup', () => { if (scrubbing) { scrubbing = false; store.emit('params'); } });

  animator.onChange(render);
  store.subscribe((kind) => { if (kind === 'structure' || kind === 'poses') render(true); });
  render();
}
