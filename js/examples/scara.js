// Example: SCARA arm. A column carries three joints with vertical axes: the shoulder and elbow swing
// the arm in a horizontal plane, and the wrist spins the tool, which points straight down and ends
// in a ball. A real SCARA also slides its tool up and down; these modules only turn, so this one
// keeps the tool at one height.
import { builder, v3, UP, DOWN, BACK, at } from './build.js';

const { motor, link, robot } = builder();
const H = 420; // height of the arm plane (mm)

motor('shoulder', 'shoulder', at('column'), ['cycloidal_18', 'cycloidal_18'], v3(0, 0, H), UP, BACK, 'upper_arm');
motor('elbow', 'elbow', at('upper_arm'), ['cycloidal_18'], v3(300, 0, H), UP, BACK, 'forearm');
motor('wrist', 'wrist', at('forearm'), ['planetary_5'], v3(560, 0, H - 120), DOWN, BACK, 'tool');

link('column', 'column', at('base'), ['shoulder']);
link('upper_arm', 'upper_arm', at('shoulder'), ['elbow']);
link('forearm', 'forearm', at('elbow'), ['wrist']);
link('tool', 'tool', at('wrist'), [{ sphere: 20, at: v3(560, 0, H - 220) }]);

// Saved poses (the Poses list above the Assembly list). Elbow left and Elbow right bend the
// shoulder and elbow opposite ways, reaching the same point with the elbow on either side.
const scara = (name, shoulder, elbow, wrist = 0) => ({ name, q: { shoulder, elbow, wrist } });
export const SCARA = robot('SCARA', 'column', [
  scara('Straight', 0, 0),
  scara('Elbow left', 40, -80, 40),
  scara('Elbow right', -40, 80, -40),
  scara('Folded', 0, 150, -150),
  scara('Side reach', 90, 0, 90),
]);
