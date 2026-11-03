// Example: five-bar SCARA, a parallel arm that works in a flat, horizontal plane. Two motors stand
// side by side on a column with their shafts pointing up; each turns a 200 mm proximal arm, a 280 mm
// distal arm hangs off each on a passive hinge, and the two distal arms meet at the wrist, where
// their hinges are joined in a closed loop (see fiveBar in build.js). The tool is a ball at the wrist.
// Both motors stay on the column, so the arms carry no motor weight: why parallel SCARAs are fast.
// Turning both motors the same way swings the wrist side to side; turning them opposite ways moves
// it in and out.
import { builder, fiveBar, animation, v3, UP, FWD, at } from './build.js';

const b = builder();
const seats = [];
const seat = (id) => at('column', seats.push(id) - 1);
const REACH = v3(0, 1, 0); // the arms reach along +y

const arm = fiveBar(b, seat, {
  centre: v3(0, 0, 400), axis: UP, along: FWD, reach: REACH, up: REACH,
  upper: 200, lower: 280, spread: 60, splay: 30, gears: ['cycloidal_18', 'cycloidal_18'],
  tip: { sphere: 20 }, // the tool, at the wrist
  names: { motorA: 'right_motor', motorB: 'left_motor', upperA: 'right_proximal', upperB: 'left_proximal', lowerA: 'right_distal', lowerB: 'left_distal' },
});
b.link('column', 'column', at('base'), seats);

const pose = (name, right, left) => ({ name, q: { right_motor: right, left_motor: left } });
export const SCARA_5BAR = b.robot('SCARA (5-bar)', 'column', [
  pose('Home', 0, 0),
  pose('Reach out', 25, -25),
  pose('Pull in', -35, 35),
  pose('Swing left', 35, 35),
  pose('Swing right', -35, -35),
], [arm.loop], [
  // Animations (the Animate view).
  animation('Trace a loop', ['Swing left', 'Reach out', 'Swing right', 'Pull in'], { pause: false }),
  animation('In and out', ['Reach out', 'Pull in']),
  animation('Side to side', [['Swing left', 0.3], ['Swing right', 0.3]]),
]);
