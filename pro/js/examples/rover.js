// Example: two-wheeled rover. A chassis (on the base, at the world origin) carries two drive motors,
// one each side with its shaft pointing out, each turning a wheel; a caster ball at the front and
// back keeps it level; a pan-tilt head on top aims a camera (a ball). The axles are at z = 0, so the
// ground is one wheel radius below. Drive both wheels the same way to roll forward, opposite ways
// to turn on the spot.
import { builder, animation, v3, UP, DOWN, BACK, LEFT, RIGHT, at } from './build.js';

const { motor, link, robot } = builder();
const WHEEL = { radius: 70, width: 30 };
const TRACK = 200; // wheel centres from the middle, each side
const GROUND = -WHEEL.radius;
const CASTER = 25;

// Drive motors hang below their mounting faces, so the chassis sits on top of them.
motor('left_drive', 'left_drive', at('chassis', 0), ['planetary_9'], v3(0, 150, 0), LEFT, DOWN, 'left_hub');
motor('right_drive', 'right_drive', at('chassis', 1), ['planetary_9'], v3(0, -150, 0), RIGHT, DOWN, 'right_hub');
motor('pan', 'pan', at('chassis', 2), ['planetary_9'], v3(-60, 0, 140), UP, BACK, 'mast');
motor('tilt', 'tilt', at('mast'), ['planetary_9'], v3(-60, 60, 320), LEFT, UP, 'camera');

link('chassis', 'chassis', at('base'), ['left_drive', 'right_drive', 'pan',
  { sphere: CASTER, at: v3(170, 0, GROUND + CASTER) }, { sphere: CASTER, at: v3(-170, 0, GROUND + CASTER) }]);
link('left_hub', 'left_hub', at('left_drive'), [{ wheel: WHEEL, at: v3(0, TRACK, 0), axis: LEFT }]);
link('right_hub', 'right_hub', at('right_drive'), [{ wheel: WHEEL, at: v3(0, -TRACK, 0), axis: RIGHT }]);
link('mast', 'mast', at('pan'), ['tilt']);
link('camera', 'camera', at('tilt'), [{ sphere: 40, at: v3(50, 60, 320) }]);

// Saved poses. Each wheel's axle points out from its side, so rolling forward turns the left wheel
// positive and the right wheel negative (the same way, seen from the side).
const pose = (name, { left = 0, right = 0, pan = 0, tilt = 0 } = {}) => ({ name, q: { left_drive: left, right_drive: -right, pan, tilt } });
export const ROVER = robot('rover', 'chassis', [
  pose('Park'),
  pose('Roll 1', { left: 90, right: 90 }),
  pose('Roll 2', { left: 180, right: 180 }),
  pose('Turn 1', { left: 90, right: -90 }),
  pose('Turn 2', { left: 180, right: -180 }),
  pose('Look left', { pan: 60 }),
  pose('Look right', { pan: -60 }),
  pose('Look down', { tilt: 35 }),
], [], [
  // Animations (the Animate view): half a turn of the wheels forward, then on the spot.
  animation('Drive forward', ['Park', 'Roll 1', 'Roll 2'], { loop: false, pause: false, speed: 5 }),
  animation('Turn on the spot', ['Park', 'Turn 1', 'Turn 2'], { loop: false, pause: false, speed: 5 }),
  animation('Look around', [['Look left', 0.4], ['Look right', 0.4], 'Look down', 'Park'], { speed: 25 }),
]);
