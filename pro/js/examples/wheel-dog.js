// Example: wheeled robot dog, a wheel-legged robot. A body (on the base, at the world origin) with
// four legs, each with a hip abduction joint (swinging the leg out to the side), a hip pitch and a
// knee, and a driven wheel where the foot would be, so it can roll on its wheels as well as step.
// With every joint at 0° it stands with its knees bent back and each wheel right under its hip; the
// axles are at z = -340, so the ground is one wheel radius below that.
import { builder, animation, v3, DOWN, FWD, at } from './build.js';

const { motor, link, robot } = builder();
const WHEEL = { radius: 55, width: 32 };
const LEGS = [['front_left', 1, 1], ['front_right', 1, -1], ['back_left', -1, 1], ['back_right', -1, -1]];
const HIP_X = 230; // the hips, from the middle along x
const ABDUCT_Y = 70, LEG_Y = 140; // the abduction motors, and the plane each leg moves in
const KNEE = v3(-40, 0, -175), AXLE_Z = -340; // knee from the hip at 0° (back and down), and the wheel axles

LEGS.forEach(([key, fx, s], i) => {
  const id = (p) => `${key}_${p}`;
  const OUT = v3(0, s, 0);
  const hx = fx * HIP_X;
  motor(id('abduct'), id('abduct'), at('body', i), ['cycloidal_18'], v3(hx, s * ABDUCT_Y, 0), FWD, DOWN, id('hip'));
  motor(id('hip_pitch'), id('hip_pitch'), at(id('hip')), ['cycloidal_18'], v3(hx + 50, s * LEG_Y, 0), OUT, DOWN, id('thigh'));
  motor(id('knee'), id('knee'), at(id('thigh')), ['cycloidal_18'], v3(hx + 50 + KNEE.x, s * LEG_Y, KNEE.z), OUT, DOWN, id('shin'));
  motor(id('wheel'), id('wheel'), at(id('shin')), ['planetary_5'], v3(hx + 50, s * LEG_Y, AXLE_Z), OUT, DOWN, id('hub'));
  link(id('hip'), id('hip'), at(id('abduct')), [id('hip_pitch')]);
  link(id('thigh'), id('thigh'), at(id('hip_pitch')), [id('knee')]);
  link(id('shin'), id('shin'), at(id('knee')), [id('wheel')]);
  link(id('hub'), id('hub'), at(id('wheel')), [{ wheel: WHEEL, at: v3(hx + 50, s * (LEG_Y + 45), AXLE_Z), axis: OUT }]);
});
link('body', 'body', at('base'), LEGS.map(([k]) => `${k}_abduct`));

// Saved poses, given as for a left leg: the right legs are mirror images, so their joints are
// negated. A positive hip pitch swings the thigh back; a negative knee folds the shin forward.
const JOINTS = ['abduct', 'hip_pitch', 'knee', 'wheel'];
function pose(name, angles) {
  const q = {};
  for (const [key, fx, s] of LEGS) {
    const a = typeof angles === 'function' ? angles(key, fx) : angles;
    JOINTS.forEach((j, k) => { q[`${key}_${j}`] = s * (a[k] ?? 0); });
  }
  return { name, q };
}
export const WHEEL_DOG = robot('wheeled robot dog', 'body', [
  pose('Stand', [0, 0, 0]),
  pose('Crouch', [0, 30, -60]),
  pose('Tall', [0, -15, 30]),
  pose('Wide', [-15, 0, 0]),
  pose('Wave', (key) => (key === 'front_left' ? [-10, -80, -20] : [0, 0, 0])),
  pose('Roll 1', [0, 0, 0, 90]),
  pose('Roll 2', [0, 0, 0, 180]),
], [], [
  // Animations (the Animate view).
  animation('Drive forward', ['Stand', 'Roll 1', 'Roll 2'], { loop: false, pause: false, speed: 5 }),
  animation('Crouch', [['Stand', 0.3], ['Crouch', 0.3], 'Tall']),
  animation('Wave', [['Stand', 0.3], ['Wave', 0.6]]),
]);
