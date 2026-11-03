// Example: 6-DoF industrial arm, the classic layout: base yaw, shoulder and elbow pitch, then a
// wrist of roll, pitch and roll whose three axes meet near one point, ending in a ball tool. With
// every joint at 0° the upper arm stands straight up and the forearm points forward.
import { builder, animation, v3, UP, FWD, BACK, LEFT, at } from './build.js';

const { motor, link, robot } = builder();
const Y = 100; // the shoulder, elbow and wrist motors sit this far to the side of the base axis

motor('j1', 'J1 base', at('pedestal'), ['cycloidal_18', 'cycloidal_18'], v3(0, 0, 220), UP, BACK, 'turret');
motor('j2', 'J2 shoulder', at('turret'), ['cycloidal_18', 'cycloidal_18'], v3(0, Y, 330), LEFT, UP, 'upper_arm');
motor('j3', 'J3 elbow', at('upper_arm'), ['cycloidal_18'], v3(0, Y, 650), LEFT, UP, 'forearm');
motor('j4', 'J4 wrist roll', at('forearm'), ['planetary_9'], v3(300, Y, 650), FWD, UP, 'wrist1');
motor('j5', 'J5 wrist pitch', at('wrist1'), ['planetary_9'], v3(420, Y, 650), LEFT, UP, 'wrist2');
motor('j6', 'J6 tool roll', at('wrist2'), ['planetary_5'], v3(575, Y, 650), FWD, UP, 'tool');

link('pedestal', 'pedestal', at('base'), ['j1']);
link('turret', 'turret', at('j1'), ['j2']);
link('upper_arm', 'upper_arm', at('j2'), ['j3']);
link('forearm', 'forearm', at('j3'), ['j4']);
link('wrist1', 'wrist1', at('j4'), ['j5']);
link('wrist2', 'wrist2', at('j5'), ['j6']);
link('tool', 'tool', at('j6'), [{ sphere: 22, at: v3(655, Y, 650) }]);

// Saved poses (the Poses list above the Assembly list).
const arm = (name, ...q) => ({ name, q: Object.fromEntries(q.map((v, i) => [`j${i + 1}`, v])) });
export const ARM_6DOF = robot('6DOF arm', 'pedestal', [
  arm('Ready', 0, 0, 0, 0, 0, 0),
  arm('Upright', 0, 0, -90, 0, 0, 0),
  arm('Reach', 0, 40, -30, 0, -10, 0),
  arm('Pick', 30, 20, 40, 0, 30, 0),
  arm('Wrist twist', -45, 10, 10, 90, 60, -90),
  arm('Stow', 0, -30, 125, 0, 60, 0),
], [], [
  // Animations (the Animate view).
  animation('Pick and place', [['Ready', 0.3], 'Reach', ['Pick', 0.5], 'Upright']),
  animation('Wrist twist', [['Ready', 0.3], ['Wrist twist', 0.5]], { speed: 25 }),
  animation('Stretch', [['Ready', 0.3], ['Upright', 0.5], ['Reach', 0.3]], { speed: 25 }),
]);
