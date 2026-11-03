// Start screen: a gallery of the example robots, in sections by category (legged robots, arms,
// simple mechanisms). It opens after the tutorial closes when the site first opens, and from the
// Examples button. Picking a card replaces the robot; New in the top bar starts a blank one.
import * as store from './robot.js';
import { state } from './robot.js';
import { EXAMPLES, CATEGORIES } from './examples.js';

const $ = (s) => document.querySelector(s);
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const dof = (data) => data.nodes.filter((n) => n.type === 'actuator').length;

export function initStart(viewer) {
  const dlg = $('#start-dialog');
  const card = (id, name, note, pic) => `<button type="button" class="start-card" data-start="${id}">
      <span class="start-pic">${pic}</span>
      <span class="start-name">${esc(name)}</span><span class="start-note">${esc(note)}</span></button>`;
  $('#start-grid').innerHTML = CATEGORIES.map(([key, heading]) => `
    <section class="start-section" aria-labelledby="start-${key}">
      <h3 class="start-heading" id="start-${key}">${esc(heading)}</h3>
      <div class="start-cards">${EXAMPLES.filter((ex) => ex.category === key).map((ex) => card(ex.id, ex.label, `${ex.blurb} · ${dof(ex.data)} DoF`,
        `<img src="img/examples/${ex.id}.png" alt="" loading="lazy" onerror="this.remove()">`)).join('')}</div>
    </section>`).join('');

  $('#start-grid').addEventListener('click', (e) => {
    const id = e.target.closest('[data-start]')?.dataset.start;
    if (!id) return;
    store.load(EXAMPLES.find((x) => x.id === id).data);
    viewer.fit();
    dlg.close();
  });
  $('#start-close').addEventListener('click', () => dlg.close());

  const open = () => {
    const parts = Object.keys(state.robot.nodes).length;
    $('#start-note').textContent = parts > 1 ? `Picking one replaces “${state.name}”. Close this window to keep it.` : '';
    dlg.showModal();
  };
  $('#btn-examples').addEventListener('click', open);
  return { open };
}
