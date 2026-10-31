// Example: pan-tilt head, the simplest useful mechanism: two actuators at right angles. The pan
// motor stands on the base with its shaft up and turns everything above it about the vertical; the
// tilt motor on its bracket nods the head (a ball for a camera) up and down.
import { builder, animation, v3, UP, BACK, LEFT, at } from './build.js';

const { motor, link, robot } = builder();
motor('pan', 'pan', at('stand'), ['cycloidal_18'], v3(0, 0, 200), UP, BACK, 'bracket');
motor('tilt', 'tilt', at('bracket'), ['planetary_9'], v3(0, 60, 380), LEFT, UP, 'head');
link('stand', 'stand', at('base'), ['pan']);
link('bracket', 'bracket', at('pan'), ['tilt']);
link('head', 'head', at('tilt'), [{ sphere: 40, at: v3(110, 60, 380) }]);

const look = (name, pan, tilt) => ({ name, q: { pan, tilt } });
export const PAN_TILT = robot('pan-tilt', 'stand', [
  look('Centre', 0, 0),
  look('Look left', 60, 0),
  look('Look right', -60, 0),
  look('Look up', 0, -40),
  look('Look down', 0, 40),
  look('Up left', 60, -40),
  look('Up right', -60, -40),
  look('Down right', -60, 40),
  look('Down left', 60, 40),
], [], [
  // Animations (the Animate view).
  animation('Scan', [['Look left', 0.4], ['Look right', 0.4]], { speed: 25 }),
  animation('Nod', ['Look up', 'Look down']),
  animation('Look around', ['Up left', 'Up right', 'Down right', 'Down left'], { pause: false, speed: 25 }),
]);
