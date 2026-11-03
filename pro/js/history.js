// Undo and redo. After each change settles (a slider drag, an edit, a new part) the whole project is
// saved as JSON on a stack; Undo loads the one before it, Redo the one after. Joint angles count, so
// moving a slider can be undone, but an animation playing doesn't add steps. Also knows whether the
// project has changed since it was opened or saved (the dot on the document tab).
import * as store from './robot.js';
import { state, node } from './robot.js';

const LIMIT = 100; // steps kept
const SETTLE = 450; // ms with no further change before a step is saved

export function initHistory(animator) {
  const stack = [store.serialize()];
  let at = 0;
  let clean = stack[0]; // as it was opened or last saved
  let timer = null;
  let restoring = false;
  const listeners = new Set();
  const notify = () => listeners.forEach((fn) => fn());

  function commit() {
    timer = null;
    if (animator.playing) return schedule(); // wait for the motion to stop
    const s = store.serialize();
    if (s === stack[at]) return notify();
    stack.splice(at + 1);
    stack.push(s);
    if (stack.length > LIMIT) stack.shift();
    at = stack.length - 1;
    notify();
  }
  function schedule() {
    clearTimeout(timer);
    timer = setTimeout(commit, SETTLE);
  }

  function restore(i) {
    clearTimeout(timer);
    timer = null;
    at = i;
    const sel = state.selection, view = state.view;
    restoring = true;
    store.load(stack[at]);
    state.fresh = false;
    state.born = {}; // no growing in: the parts are simply there
    // Keep the selection where it still exists.
    if (sel && node(sel.nodeId) && (sel.kind === 'node' || node(sel.nodeId).children.length > sel.index)) store.select(sel);
    if (state.view !== view) store.setView(view);
    restoring = false;
    notify();
  }

  store.subscribe((kind) => {
    if (restoring) return;
    if (kind === 'structure' && state.fresh) {
      // A robot was just opened (an example, New, a file): it starts its own history.
      state.fresh = false;
      clearTimeout(timer);
      timer = null;
      stack.length = 0;
      stack.push(store.serialize());
      at = 0;
      clean = stack[0];
      notify();
      return;
    }
    if (kind === 'structure' || kind === 'params' || kind === 'rename' || kind === 'poses') schedule();
  });
  document.addEventListener('rb:animations', () => { if (!restoring) schedule(); });
  // The project name is typed straight into the state.
  document.getElementById('project-name').addEventListener('input', schedule);
  // Saving: copying or downloading the JSON counts as saved.
  for (const id of ['file-copy', 'file-download']) {
    document.getElementById(id).addEventListener('click', () => { commit(); clean = stack[at]; notify(); });
  }

  return {
    undo() { if (timer) commit(); if (at > 0) restore(at - 1); },
    redo() { if (timer) commit(); if (at < stack.length - 1) restore(at + 1); },
    get canUndo() { return at > 0 || !!timer; },
    get canRedo() { return at < stack.length - 1; },
    get dirty() { return (timer ? store.serialize() : stack[at]) !== clean; },
    onChange(fn) { listeners.add(fn); },
  };
}
