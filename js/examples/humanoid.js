// Example: humanoid, the full arrangement. A pelvis on the base (at the world origin) carries a
// waist joint and torso, with a head on a pan/tilt neck and two 7-joint arms (shoulder pitch, roll
// and yaw, elbow, wrist yaw, pitch and roll, and a ball for a hand), and below it two 6-joint legs
// (hip yaw, roll and pitch, knee, ankle pitch and roll, and a foot with a ball at the toe and heel).
// With every joint at 0° it stands straight with its arms hanging down.
// Every motor is placed in world coordinates (see build.js), so the links come out of the geometry.
import { builder, v3, UP, DOWN, FWD, BACK, at } from './build.js';

const { motor, link, robot } = builder();
const SIDES = [['left', 1], ['right', -1]];

// Waist: turns the whole torso about the vertical. Neck: pan about the vertical, then tilt (nod).
motor('waist', 'waist', at('pelvis', 0), ['harmonic_45_2', 'cycloidal_18'], v3(0, 0, 300), UP, BACK, 'torso');
motor('neck_pan', 'neck_pan', at('torso', 2), ['planetary_5'], v3(0, 0, 660), UP, BACK, 'neck');
motor('neck_tilt', 'neck_tilt', at('neck'), ['planetary_5'], v3(0, 40, 740), v3(0, 1, 0), UP, 'head');

for (const [side, s] of SIDES) {
  const id = (p) => `${side}_${p}`;
  const OUT = v3(0, s, 0);
  // Arm: the shoulder's three axes, then the elbow, then the wrist's three.
  motor(id('shoulder_pitch'), id('shoulder_pitch'), at('torso', side === 'left' ? 0 : 1), ['cycloidal_18', 'cycloidal_18'], v3(0, 190 * s, 560), OUT, UP, id('shoulder'));
  motor(id('shoulder_roll'), id('shoulder_roll'), at(id('shoulder')), ['cycloidal_18'], v3(70, 280 * s, 520), FWD, UP, id('shoulder_block'));
  motor(id('shoulder_yaw'), id('shoulder_yaw'), at(id('shoulder_block')), ['planetary_9'], v3(0, 300 * s, 360), DOWN, OUT, id('upper_arm'));
  motor(id('elbow'), id('elbow'), at(id('upper_arm')), ['planetary_9'], v3(0, 310 * s, 220), OUT, DOWN, id('elbow_block'));
  motor(id('wrist_yaw'), id('wrist_yaw'), at(id('elbow_block')), ['planetary_5'], v3(0, 310 * s, 60), DOWN, OUT, id('forearm'));
  motor(id('wrist_pitch'), id('wrist_pitch'), at(id('forearm')), ['planetary_5'], v3(0, 320 * s, -40), OUT, DOWN, id('wrist'));
  motor(id('wrist_roll'), id('wrist_roll'), at(id('wrist')), ['planetary_3'], v3(0, 320 * s, -190), DOWN, OUT, id('hand'));
  // Leg: the same six joints as the Biped example.
  motor(id('hip_yaw'), id('hip_yaw'), at('pelvis', side === 'left' ? 1 : 2), ['cycloidal_18'], v3(0, 110 * s, -40), DOWN, BACK, id('hip'));
  motor(id('hip_roll'), id('hip_roll'), at(id('hip')), ['cycloidal_18', 'cycloidal_18'], v3(100, 110 * s, -170), FWD, DOWN, id('hip_block'));
  motor(id('hip_pitch'), id('hip_pitch'), at(id('hip_block')), ['cycloidal_18', 'cycloidal_18'], v3(0, 230 * s, -240), OUT, DOWN, id('thigh'));
  motor(id('knee'), id('knee'), at(id('thigh')), ['harmonic_45_2'], v3(0, 230 * s, -500), OUT, DOWN, id('shin'));
  motor(id('ankle_pitch'), id('ankle_pitch'), at(id('shin')), ['cycloidal_18'], v3(0, 230 * s, -760), OUT, DOWN, id('ankle'));
  motor(id('ankle_roll'), id('ankle_roll'), at(id('ankle')), ['planetary_9'], v3(90, 175 * s, -870), FWD, DOWN, id('foot'));
}

link('pelvis', 'pelvis', at('base'), ['waist', 'left_hip_yaw', 'right_hip_yaw']);
link('torso', 'torso', at('waist'), ['left_shoulder_pitch', 'right_shoulder_pitch', 'neck_pan']);
link('neck', 'neck', at('neck_pan'), ['neck_tilt']);
link('head', 'head', at('neck_tilt'), [{ sphere: 70, at: v3(0, 0, 850) }]);
for (const [side, s] of SIDES) {
  const id = (p) => `${side}_${p}`;
  link(id('shoulder'), id('shoulder'), at(id('shoulder_pitch')), [id('shoulder_roll')]);
  link(id('shoulder_block'), id('shoulder_block'), at(id('shoulder_roll')), [id('shoulder_yaw')]);
  link(id('upper_arm'), id('upper_arm'), at(id('shoulder_yaw')), [id('elbow')]);
  link(id('elbow_block'), id('elbow_block'), at(id('elbow')), [id('wrist_yaw')]);
  link(id('forearm'), id('forearm'), at(id('wrist_yaw')), [id('wrist_pitch')]);
  link(id('wrist'), id('wrist'), at(id('wrist_pitch')), [id('wrist_roll')]);
  link(id('hand'), id('hand'), at(id('wrist_roll')), [{ sphere: 32, at: v3(0, 320 * s, -250) }]);
  link(id('hip'), id('hip'), at(id('hip_yaw')), [id('hip_roll')]);
  link(id('hip_block'), id('hip_block'), at(id('hip_roll')), [id('hip_pitch')]);
  link(id('thigh'), id('thigh'), at(id('hip_pitch')), [id('knee')]);
  link(id('shin'), id('shin'), at(id('knee')), [id('ankle_pitch')]);
  link(id('ankle'), id('ankle'), at(id('ankle_pitch')), [id('ankle_roll')]);
  link(id('foot'), id('foot'), at(id('ankle_roll')), [{ sphere: 25, at: v3(170, 175 * s, -930) }, { sphere: 25, at: v3(-60, 175 * s, -930) }]);
}

// Saved poses (the Poses list above the Assembly list). Angles are given as for the left side; the
// right side is its mirror image, so every right joint is negated.
const LIMB = ['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_yaw', 'wrist_pitch', 'wrist_roll',
  'hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ankle_pitch', 'ankle_roll'];
function bodyPose(name, { left = {}, right = {}, both = {}, ...centre } = {}) {
  const q = { waist: 0, neck_pan: 0, neck_tilt: 0, ...centre };
  for (const j of LIMB) {
    q[`left_${j}`] = left[j] ?? both[j] ?? 0;
    q[`right_${j}`] = -(right[j] ?? both[j] ?? 0);
  }
  return { name, q };
}
export const HUMANOID = robot('humanoid', 'pelvis', [
  bodyPose('Stand'),
  bodyPose('Arms forward', { both: { elbow: -90 } }),
  bodyPose('T-pose', { both: { shoulder_roll: 90 } }),
  // Wave: the upper arm out to the side, turned about its own axis so the elbow lifts the forearm up.
  bodyPose('Wave', { right: { shoulder_roll: 90, shoulder_yaw: -90, elbow: -90 }, neck_pan: -20 }),
  bodyPose('Walk', {
    left: { shoulder_pitch: 20, elbow: -20, hip_pitch: -30, knee: 20, ankle_pitch: 10 },
    right: { shoulder_pitch: -20, elbow: -20, hip_pitch: 15, knee: 30, ankle_pitch: -45 },
  }),
  bodyPose('Squat', { both: { shoulder_pitch: -80, hip_pitch: -60, knee: 120, ankle_pitch: -60 } }),
  bodyPose('Reach up', { both: { shoulder_pitch: -170 } }),
  bodyPose('Look around', { waist: 40, neck_pan: 30, neck_tilt: 15, both: { elbow: -30 } }),
]);
