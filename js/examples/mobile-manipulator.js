// Example: mobile manipulator. A four-wheeled, skid-steer base (a drive motor and wheel at each
// corner, shafts pointing out to the sides) with a four-joint arm on top: a base yaw, shoulder and
// elbow pitch, and a wrist pitch ending in a ball tool. The axles are at z = 0, so the ground is one
// wheel radius below. With every joint at 0° the upper arm stands straight up and the forearm
// points forward.
import { builder, animation, v3, UP, DOWN, BACK, LEFT, RIGHT, at } from './build.js';

const { motor, link, robot } = builder();
const WHEEL = { radius: 75, width: 34 };
const WHEELBASE = 170, TRACK = 230; // wheel centres from the middle: along and across
const CORNERS = [['front_left', 1, 1], ['front_right', 1, -1], ['back_left', -1, 1], ['back_right', -1, -1]];
const Y = 100; // the shoulder, elbow and wrist motors sit this far to the side of the yaw axis

CORNERS.forEach(([key, fx, side], i) => {
  const out = side > 0 ? LEFT : RIGHT;
  // Each drive motor hangs below its mounting face, so the chassis sits on top of them.
  motor(`${key}_drive`, `${key}_drive`, at('chassis', i), ['planetary_9'], v3(fx * WHEELBASE, side * 175, 0), out, DOWN, `${key}_hub`);
  link(`${key}_hub`, `${key}_hub`, at(`${key}_drive`), [{ wheel: WHEEL, at: v3(fx * WHEELBASE, side * TRACK, 0), axis: out }]);
});
motor('j1', 'J1 yaw', at('chassis', 4), ['cycloidal_18'], v3(-40, 0, 150), UP, BACK, 'turret');
motor('j2', 'J2 shoulder', at('turret'), ['cycloidal_18', 'cycloidal_18'], v3(-40, Y, 260), LEFT, UP, 'upper_arm');
motor('j3', 'J3 elbow', at('upper_arm'), ['cycloidal_18'], v3(-40, Y, 520), LEFT, UP, 'forearm');
motor('j4', 'J4 wrist', at('forearm'), ['planetary_9'], v3(200, Y, 520), LEFT, UP, 'gripper');

link('chassis', 'chassis', at('base'), [...CORNERS.map(([k]) => `${k}_drive`), 'j1']);
link('turret', 'turret', at('j1'), ['j2']);
link('upper_arm', 'upper_arm', at('j2'), ['j3']);
link('forearm', 'forearm', at('j3'), ['j4']);
link('gripper', 'gripper', at('j4'), [{ sphere: 22, at: v3(310, Y, 520) }]);

// Saved poses. Each wheel's axle points out from its side, so rolling forward turns the left
// wheels positive and the right wheels negative (the same way, seen from the side).
function pose(name, { drive = 0, turn = 0, arm = [0, 0, 0, 0] } = {}) {
  const q = Object.fromEntries(arm.map((v, i) => [`j${i + 1}`, v]));
  for (const [key, , side] of CORNERS) q[`${key}_drive`] = side > 0 ? drive - turn : -(drive + turn);
  return { name, q };
}
const STOW = [0, 0, 95, -90];
export const MOBILE_MANIPULATOR = robot('mobile manipulator', 'chassis', [
  pose('Ready'),
  pose('Stow', { arm: STOW }),
  pose('Reach forward', { arm: [0, 45, -10, 30] }),
  pose('Pick from floor', { arm: [0, 70, 30, 60] }),
  pose('Reach left', { arm: [90, 45, -10, 30] }),
  pose('Roll 1', { drive: 90, arm: STOW }),
  pose('Roll 2', { drive: 180, arm: STOW }),
  pose('Turn 1', { turn: 90, arm: STOW }),
  pose('Turn 2', { turn: 180, arm: STOW }),
], [], [
  // Animations (the Animate view).
  animation('Pick up', [['Ready', 0.3], 'Reach forward', ['Pick from floor', 0.5], 'Reach forward', 'Reach left', ['Ready', 0.3]]),
  animation('Drive forward', ['Stow', 'Roll 1', 'Roll 2'], { loop: false, pause: false, speed: 5 }),
  animation('Turn on the spot', ['Stow', 'Turn 1', 'Turn 2'], { loop: false, pause: false, speed: 5 }),
]);
