// Example: parallelogram linkage. A motor turns a 200 mm crank; a rocker of the same length swings
// about a pivot 300 mm away, and a 300 mm coupler joins their tips. Opposite bars are equal, so the
// coupler stays level as it moves: the platform on it travels round an arc without tilting, the
// trick behind desk lamps, excavator arms and parallel grippers. Every joint but the motor is a
// passive hinge; the rocker's far end is joined to the frame's pivot in a closed loop. The crank
// stays within ±70° of upright: lying flat, the bars line up and the linkage could fold the other way.
import { builder, animation, v3, LEFT, UP, at } from './build.js';

const { motor, link, robot } = builder();
const Y = 60; // the plane the bars move in; every hinge axis is along +y
const H = 300; // height of the two ground pivots
const A = v3(0, Y, H), D = v3(300, Y, H); // crank pivot (the motor's output) and rocker pivot
const B = v3(0, Y, H + 200), C = v3(300, Y, H + 200); // crank and rocker tips, with the crank upright

motor('crank_motor', 'crank_motor', at('frame'), ['cycloidal_18', 'cycloidal_18'], A, LEFT, UP, 'crank');
link('frame', 'frame', at('base'), ['crank_motor', { joint: 'hinge', at: D, axis: LEFT }]);
link('crank', 'crank', at('crank_motor'), [{ joint: 'hinge', at: B, axis: LEFT, child: 'coupler' }]);
link('coupler', 'coupler', at('crank', 0), [{ joint: 'hinge', at: C, axis: LEFT, child: 'rocker' }, { sphere: 30, at: v3(150, Y, H + 260) }]);
link('rocker', 'rocker', at('coupler', 0), [{ joint: 'hinge', at: D, axis: LEFT }]);

const crank = (name, deg) => ({ name, q: { crank_motor: deg } });
export const PARALLELOGRAM = robot('Parallelogram', 'frame', [
  crank('Upright', 0), crank('Lean forward', 40), crank('Lean back', -40), crank('Far forward', 70), crank('Far back', -70),
], [[['rocker', 0], ['frame', 1]]], [
  // Animations (the Animate view).
  animation('Swing', ['Far back', 'Far forward']),
  animation('Step through', [['Upright', 0.4], ['Lean forward', 0.4], ['Upright', 0.4], ['Lean back', 0.4]]),
]);
