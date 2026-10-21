import { createViewer, loadCadModels } from './viewer.js';
import * as store from './robot.js';
import { initUI } from './ui.js';
import { EXAMPLES } from './examples.js';
import { initTutorial } from './tutorial.js';
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
// Guided tour: opens every time the site opens, and from the Tutorial button.
initTutorial(viewer);
