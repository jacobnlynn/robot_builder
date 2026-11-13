// Example: six-wheeled rover on a rocker-bogie suspension, the layout of NASA's Mars rovers. Each
// side of the body has a rocker on a free hinge: its front end carries the front wheel, and its back
// end carries a bogie, on a second free hinge, with the middle and rear wheels. The hinges let the
// six wheels follow uneven ground and keep all of them on it. Every wheel has its own drive motor;
// a mast on top carries a pan-tilt camera. The axles are at z = 0, so the ground is one wheel radius
// below. (On a real rover a differential bar across the body links the two rockers; here each
// rocker swings on its own.)
import { builder, animation, v3, UP, DOWN, BACK, LEFT, RIGHT, at } from './build.js';

const { motor, link, robot } = builder();
const WHEEL = { radius: 65, width: 45 };
const SIDES = [['left', 1], ['right', -1]];
const TRACK = 270; // wheel centres from the middle
const LINKS_Y = 215; // the rockers and bogies run this far out, inside the wheels
// Along x: the front, middle and rear wheels; the rocker's hinge on the body, and the bogie's hinge.
const FRONT = 340, MIDDLE = -20, REAR = -290;
const ROCKER = v3(60, 0, 190), BOGIE = v3(-155, 0, 95);

for (const [side, s] of SIDES) {
  const id = (p) => `${side}_${p}`;
  const out = s > 0 ? LEFT : RIGHT;
  const y = (k) => s * k;
  // Drive motors hang below their mounting faces, just inside each wheel.
  for (const [w, x] of [['front', FRONT], ['middle', MIDDLE], ['rear', REAR]]) {
    const parent = w === 'front' ? at(id('rocker'), 0) : at(id('bogie'), w === 'middle' ? 0 : 1);
    motor(id(`${w}_drive`), id(`${w}_drive`), parent, ['planetary_9'], v3(x, y(TRACK - 45), 0), out, DOWN, id(`${w}_hub`));
    link(id(`${w}_hub`), id(`${w}_hub`), at(id(`${w}_drive`)), [{ wheel: WHEEL, at: v3(x, y(TRACK), 0), axis: out }]);
  }
  // Rocker: on a hinge on the body; the front wheel's motor, and the bogie's hinge at its back end.
  link(id('rocker'), id('rocker'), at('body', side === 'left' ? 0 : 1), [id('front_drive'),
    { joint: 'hinge', at: v3(BOGIE.x, y(LINKS_Y), BOGIE.z), axis: out, child: id('bogie') }]);
  // Bogie: the middle and rear wheels' motors.
  link(id('bogie'), id('bogie'), at(id('rocker'), 1), [id('middle_drive'), id('rear_drive')]);
}
// The mast's pan and tilt.
motor('pan', 'pan', at('body', 2), ['planetary_9'], v3(220, 0, 330), UP, BACK, 'mast');
motor('tilt', 'tilt', at('mast'), ['planetary_9'], v3(220, 60, 520), LEFT, UP, 'camera');
link('mast', 'mast', at('pan'), ['tilt']);
link('camera', 'camera', at('tilt'), [{ sphere: 45, at: v3(330, 60, 520) }]);
// Body: the two rocker hinges and the mast.
link('body', 'body', at('base'), [
  ...SIDES.map(([, s]) => ({ joint: 'hinge', at: v3(ROCKER.x, s * LINKS_Y, ROCKER.z), axis: s > 0 ? LEFT : RIGHT, child: `${s > 0 ? 'left' : 'right'}_rocker` })),
  'pan',
]);

// Saved poses. Each wheel's axle points out from its side, so rolling forward turns the left wheels
// positive and the right wheels negative (the same way, seen from the side).
const WHEELS = SIDES.flatMap(([side, s]) => ['front', 'middle', 'rear'].map((w) => [`${side}_${w}_drive`, s]));
function pose(name, { roll = 0, turn = 0, pan = 0, tilt = 0 } = {}) {
  const q = { pan, tilt };
  for (const [k, s] of WHEELS) q[k] = s > 0 ? roll - turn : -(roll + turn);
  return { name, q };
}
export const ROCKER_BOGIE = robot('rocker-bogie rover', 'body', [
  pose('Park'),
  pose('Roll 1', { roll: 90 }),
  pose('Roll 2', { roll: 180 }),
  pose('Turn 1', { turn: 90 }),
  pose('Turn 2', { turn: 180 }),
  pose('Look left', { pan: 70 }),
  pose('Look right', { pan: -70 }),
  pose('Look down', { tilt: 30 }),
], [], [
  // Animations (the Animate view).
  animation('Drive forward', ['Park', 'Roll 1', 'Roll 2'], { loop: false, pause: false, speed: 5 }),
  animation('Turn on the spot', ['Park', 'Turn 1', 'Turn 2'], { loop: false, pause: false, speed: 5 }),
  animation('Look around', [['Look left', 0.4], ['Look right', 0.4], 'Look down', 'Park'], { speed: 25 }),
]);
