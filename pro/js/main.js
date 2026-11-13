import { createViewer, setCadListener } from './viewer.js';
import * as store from './robot.js';
import { initUI } from './ui.js';
import { EXAMPLES } from './examples.js';
import { initTutorial } from './tutorial.js';
import { initStyles } from './styles.js';
import { initPoses } from './poses.js';
import { initAnimate } from './animate.js';
import { initChrome } from './chrome.js';
import { initTimeline } from './timeline.js';
import { initHistory } from './history.js';
import { initMenus } from './menus.js';
import { initStart } from './start.js';
import './linkfill.js'; // trial: filled links in the CAD tab and their STEP solids (delete this line to remove)

const viewer = createViewer(document.getElementById('viewport'), {
  onPick: (pick) => store.select(pick),
});
initUI(viewer);
// Saved poses, listed above the Assembly list.
initPoses(viewer);
// Animations: the Animate view's panel plays a series of saved poses.
const animator = initAnimate(viewer);
// Menu bar menus and the Examples window.
initMenus();
const start = initStart(viewer);
// The module STEP files load when first needed; redraw (and refresh the legend) as each arrives.
setCadListener(() => { viewer.redraw(); store.emit('view'); });
// Open on the robot handed over by the other layout, if you just switched, or else the first example.
const handed = store.takeHandoff();
let switched = false;
if (handed) try { store.load(handed); switched = true; } catch { /* not a robot after all: start fresh */ }
if (!switched) store.load(EXAMPLES[0].data);
viewer.fit();
// Pro workspace: undo / redo, then the commands, view cube, labels, dock tabs and the timeline.
const undo = initHistory(animator);
initChrome(viewer, { animator, history: undo });
initTimeline(viewer, animator);
// Themes in View > Theme: Dark (default) and Light. After the first robot loads, so it can redraw.
initStyles(viewer);
// Guided tour: opens every time the site opens, and from Help > Tutorial. When it closes the first
// time, the start screen offers the examples.
let first = true;
initTutorial(viewer, { autoOpen: !switched, onClose: () => { if (first) { first = false; start.open(); } } });
// Help > Open the classic layout takes the robot with it.
document.querySelector('#menu-help a[href="index.html"]')?.addEventListener('click', () => store.handOff());
