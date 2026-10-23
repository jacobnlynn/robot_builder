import { createViewer, loadCadModels } from './viewer.js';
import * as store from './robot.js';
import { initUI } from './ui.js';
import { EXAMPLES } from './examples.js';
import { initTutorial } from './tutorial.js';
import { initStyles } from './styles.js';
import './linkfill.js'; // trial: filled links in the CAD tab + their STL (delete this line to remove)

const viewer = createViewer(document.getElementById('viewport'), {
  onPick: (pick) => store.select(pick),
});
initUI(viewer);
// The CAD tab's actuator shells; redraw (and refresh the legend) as each one loads or is found missing.
loadCadModels(() => { viewer.redraw(); store.emit('view'); });
// Open on the first example.
store.load(EXAMPLES[0].data);
viewer.fit();
// Style menu: Retro (default), Normal (Datasheet), Risograph. After the first robot loads, so it can redraw.
initStyles(viewer);
// Guided tour: opens every time the site opens, and from the Tutorial button.
initTutorial(viewer);
