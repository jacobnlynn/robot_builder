// Example: biped, walking legs only. A pelvis on the base (at the world origin) carries two legs
// of six joints each: hip yaw, hip roll and hip pitch, knee, ankle pitch and ankle roll. Each foot
// is a link with a ball at the toe and one at the heel.
import { builder, animation, v3, DOWN, FWD, BACK, at } from './build.js';

const { motor, link, robot } = builder();
const SIDES = [['left', 1], ['right', -1]];

for (const [side, s] of SIDES) {
  const id = (p) => `${side}_${p}`;
  const OUT = v3(0, s, 0);
  motor(id('hip_yaw'), id('hip_yaw'), at('pelvis', side === 'left' ? 0 : 1), ['cycloidal_18'], v3(0, 110 * s, -40), DOWN, BACK, id('hip'));
  motor(id('hip_roll'), id('hip_roll'), at(id('hip')), ['cycloidal_18', 'cycloidal_18'], v3(100, 110 * s, -170), FWD, DOWN, id('hip_block'));
  motor(id('hip_pitch'), id('hip_pitch'), at(id('hip_block')), ['cycloidal_18', 'cycloidal_18'], v3(0, 230 * s, -240), OUT, DOWN, id('thigh'));
  motor(id('knee'), id('knee'), at(id('thigh')), ['harmonic_45_2'], v3(0, 230 * s, -500), OUT, DOWN, id('shin'));
  motor(id('ankle_pitch'), id('ankle_pitch'), at(id('shin')), ['cycloidal_18'], v3(0, 230 * s, -760), OUT, DOWN, id('ankle'));
  motor(id('ankle_roll'), id('ankle_roll'), at(id('ankle')), ['planetary_9'], v3(90, 175 * s, -870), FWD, DOWN, id('foot'));
}

link('pelvis', 'pelvis', at('base'), ['left_hip_yaw', 'right_hip_yaw']);
for (const [side, s] of SIDES) {
  const id = (p) => `${side}_${p}`;
  link(id('hip'), id('hip'), at(id('hip_yaw')), [id('hip_roll')]);
  link(id('hip_block'), id('hip_block'), at(id('hip_roll')), [id('hip_pitch')]);
  link(id('thigh'), id('thigh'), at(id('hip_pitch')), [id('knee')]);
  link(id('shin'), id('shin'), at(id('knee')), [id('ankle_pitch')]);
  link(id('ankle'), id('ankle'), at(id('ankle_pitch')), [id('ankle_roll')]);
  link(id('foot'), id('foot'), at(id('ankle_roll')), [{ sphere: 25, at: v3(170, 175 * s, -930) }, { sphere: 25, at: v3(-60, 175 * s, -930) }]);
}

// Saved poses (the Poses list above the Assembly list). Angles are given as for the left leg; the
// right leg is its mirror image, so every right joint is negated (Turn turns both feet the same way).
const LEG = ['hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ankle_pitch', 'ankle_roll'];
function legs(name, left = {}, right = left) {
  const q = {};
  for (const j of LEG) {
    q[`left_${j}`] = left[j] ?? 0;
    q[`right_${j}`] = -(right[j] ?? 0);
  }
  return { name, q };
}
export const BIPED = robot('Biped', 'pelvis', [
  legs('Stand'),
  legs('Crouch', { hip_pitch: -40, knee: 80, ankle_pitch: -40 }),
  legs('Step', { hip_pitch: -30, knee: 20, ankle_pitch: 10 }, { hip_pitch: 15, knee: 30, ankle_pitch: -45 }),
  legs('Kick', {}, { hip_pitch: -70, knee: 10, ankle_pitch: 30 }),
  legs('Wide stance', { hip_roll: 15, ankle_roll: -15 }),
  legs('Turn', { hip_yaw: 30 }, { hip_yaw: -30 }),
  // Walk: a gait cycle in four poses, worked out by inverse kinematics so feet on the ground stay level and
  // move back while the others lift and swing forward (the body is fixed, so it walks on the spot).
  legs('Walk 1', { hip_pitch: -30.5, knee: 42, ankle_pitch: -12 }, { hip_pitch: -12, knee: 42.5, ankle_pitch: -30 }),
  legs('Walk 2', { hip_pitch: -23, knee: 46, ankle_pitch: -23 }, { hip_pitch: -36, knee: 72.5, ankle_pitch: -36 }),
  legs('Walk 3', { hip_pitch: -12, knee: 42.5, ankle_pitch: -30 }, { hip_pitch: -30.5, knee: 42, ankle_pitch: -12 }),
  legs('Walk 4', { hip_pitch: -36, knee: 72.5, ankle_pitch: -36 }, { hip_pitch: -23, knee: 46, ankle_pitch: -23 }),
], [], [
  // Animations (the Animate view).
  animation('Walk', ['Walk 1', 'Walk 2', 'Walk 3', 'Walk 4'], { pause: false, speed: 100, walk: true }),
  animation('Squat', ['Stand', ['Crouch', 0.5]], { speed: 25 }),
  animation('Kick', [['Stand', 0.5], 'Kick']),
  animation('Look around', [['Stand', 0.3], ['Turn', 0.5], ['Wide stance', 0.3]], { speed: 25 }),
]);
