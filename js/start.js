// Start screen: a gallery of the example robots plus a blank one. It opens after the tutorial
// closes when the site first opens, from the Examples button and from File > New. Picking a card
// replaces the robot.
import * as store from './robot.js';
import { state } from './robot.js';
import { EXAMPLES } from './examples.js';

const $ = (s) => document.querySelector(s);
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const dof = (data) => data.nodes.filter((n) => n.type === 'actuator').length;

export function initStart(viewer) {
  const dlg = $('#start-dialog');
  const card = (id, name, note, pic) => `<button type="button" class="start-card" data-start="${id}">
      <span class="start-pic">${pic}</span>
      <span class="start-name">${esc(name)}</span><span class="start-note">${esc(note)}</span></button>`;
  $('#start-grid').innerHTML = EXAMPLES.map((ex) => card(ex.id, ex.label, `${ex.blurb} · ${dof(ex.data)} DoF`,
    `<img src="img/examples/${ex.id}.png" alt="" loading="lazy" onerror="this.remove()">`)).join('')
    + card('blank', 'Blank', 'Just the base: build from scratch', `<svg viewBox="0 0 60 60" aria-hidden="true">
        <rect x="16" y="38" width="28" height="8" class="start-blank-base"></rect><circle cx="30" cy="30" r="6" class="start-blank-port"></circle></svg>`);

  $('#start-grid').addEventListener('click', (e) => {
    const id = e.target.closest('[data-start]')?.dataset.start;
    if (!id) return;
    if (id === 'blank') store.newRobot();
    else store.load(EXAMPLES.find((x) => x.id === id).data);
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
  $('#btn-new').addEventListener('click', open);
  return { open };
}
