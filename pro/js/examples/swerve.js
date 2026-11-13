// Example: swerve drive, as on an FRC robot. A square chassis (on the base, at the world origin)
// with a swerve module at each corner: a steering motor whose shaft points straight down turns the
// whole module about the vertical, and the module carries a drive motor that spins a 4 in wheel
// right below the steering axis. Each wheel can point any way and roll at any speed, so the robot
// can drive, strafe and spin at once. The axles are at z = 0, so the floor is one wheel radius below.
import { builder, animation, v3, DOWN, LEFT, at } from './build.js';

const { motor, link, robot } = builder();
const WHEEL = { radius: 51, width: 38 }; // 4 in × 1.5 in, the usual FRC swerve wheel
const HALF = 260; // steering axes from the middle, along and across (a 26 in square frame)
const MODULES = [['front_left', 1, 1], ['front_right', 1, -1], ['back_left', -1, 1], ['back_right', -1, -1]];
const STEER_Z = 130; // height of the steering motors' outputs

MODULES.forEach(([key, fx, fy], i) => {
  const x = fx * HALF, y = fy * HALF;
  const inward = v3(-fx, -fy, 0).normalize();
  // Steering motor: shaft down, its body standing in towards the middle of the frame.
  motor(`${key}_steer`, `${key}_steer`, at('chassis', i), ['cycloidal_18'], v3(x, y, STEER_Z), DOWN, inward, `${key}_fork`);
  // Drive motor: shaft along +y, hanging from its mounting face beside the wheel. Every module is
  // the same way round, as on a real robot.
  motor(`${key}_drive`, `${key}_drive`, at(`${key}_fork`), ['planetary_5'], v3(x, y - 45, 0), LEFT, DOWN, `${key}_hub`);
  link(`${key}_fork`, `${key}_fork`, at(`${key}_steer`), [`${key}_drive`]);
  link(`${key}_hub`, `${key}_hub`, at(`${key}_drive`), [{ wheel: WHEEL, at: v3(x, y, 0), axis: LEFT }]);
});
link('chassis', 'chassis', at('base'), MODULES.map(([k]) => `${k}_steer`));

// Saved poses. A steering motor's shaft points down, so a positive angle turns its module clockwise
// seen from above; with every joint at 0° the wheels roll along x (forward). `steer` is how far each
// wheel's rolling direction is turned anticlockwise from forward (degrees), `roll` how far it has
// turned on its axle.
function pose(name, steer, roll = 0) {
  const q = {};
  for (const [key, fx, fy] of MODULES) {
    q[`${key}_steer`] = -(typeof steer === 'function' ? steer(fx, fy) : steer);
    q[`${key}_drive`] = roll;
  }
  return { name, q };
}
// Spinning on the spot: each wheel square to the line from the middle (so all four roll round a circle).
const spin = (fx, fy) => { const a = (Math.atan2(fy, fx) * 180) / Math.PI + 90; return ((a + 90) % 180 + 180) % 180 - 90; };
export const SWERVE = robot('swerve drive', 'chassis', [
  pose('Forward', 0),
  pose('Diagonal', 45),
  pose('Strafe', 90),
  pose('Spin', spin),
  pose('Roll 1', 0, 90),
  pose('Roll 2', 0, 180),
  pose('Strafe roll 1', 90, 90),
  pose('Strafe roll 2', 90, 180),
], [], [
  // Animations (the Animate view).
  animation('Steer around', [['Forward', 0.3], ['Diagonal', 0.3], ['Strafe', 0.3], ['Spin', 0.5], 'Forward'], { speed: 30 }),
  animation('Drive forward', ['Forward', 'Roll 1', 'Roll 2'], { loop: false, pause: false, speed: 10 }),
  animation('Strafe', ['Strafe', 'Strafe roll 1', 'Strafe roll 2'], { loop: false, pause: false, speed: 10 }),
]);
