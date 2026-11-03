// Example: humanoid torso, the Humanoid example's upper body on a fixed 300 mm pedestal: a waist joint,
// a head on a pan/tilt neck and two 7-joint arms (shoulder pitch, roll and yaw, elbow, wrist yaw,
// pitch and roll, and a ball for a hand). A two-armed workstation robot: a humanoid without legs.
// With every joint at 0° the arms hang straight down.
import { builder, animation, v3, UP, DOWN, FWD, BACK, at } from './build.js';

const { motor, link, robot } = builder();
const SIDES = [['left', 1], ['right', -1]];

motor('waist', 'waist', at('pedestal'), ['harmonic_45_2', 'cycloidal_18'], v3(0, 0, 300), UP, BACK, 'torso');
motor('neck_pan', 'neck_pan', at('torso', 2), ['planetary_5'], v3(0, 0, 660), UP, BACK, 'neck');
motor('neck_tilt', 'neck_tilt', at('neck'), ['planetary_5'], v3(0, 40, 740), v3(0, 1, 0), UP, 'head');
for (const [side, s] of SIDES) {
  const id = (p) => `${side}_${p}`;
  const OUT = v3(0, s, 0);
  motor(id('shoulder_pitch'), id('shoulder_pitch'), at('torso', side === 'left' ? 0 : 1), ['cycloidal_18', 'cycloidal_18'], v3(0, 190 * s, 560), OUT, UP, id('shoulder'));
  motor(id('shoulder_roll'), id('shoulder_roll'), at(id('shoulder')), ['cycloidal_18'], v3(70, 280 * s, 520), FWD, UP, id('shoulder_block'));
  motor(id('shoulder_yaw'), id('shoulder_yaw'), at(id('shoulder_block')), ['planetary_9'], v3(0, 300 * s, 360), DOWN, OUT, id('upper_arm'));
  motor(id('elbow'), id('elbow'), at(id('upper_arm')), ['planetary_9'], v3(0, 310 * s, 220), OUT, DOWN, id('elbow_block'));
  motor(id('wrist_yaw'), id('wrist_yaw'), at(id('elbow_block')), ['planetary_5'], v3(0, 310 * s, 60), DOWN, OUT, id('forearm'));
  motor(id('wrist_pitch'), id('wrist_pitch'), at(id('forearm')), ['planetary_5'], v3(0, 320 * s, -40), OUT, DOWN, id('wrist'));
  motor(id('wrist_roll'), id('wrist_roll'), at(id('wrist')), ['planetary_3'], v3(0, 320 * s, -190), DOWN, OUT, id('hand'));
}

link('pedestal', 'pedestal', at('base'), ['waist']);
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
}

// Saved poses. Angles are given as for the left arm; the right arm is its mirror image, so every
// right joint is negated.
const ARM = ['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_yaw', 'wrist_pitch', 'wrist_roll'];
function torsoPose(name, { left = {}, right = {}, both = {}, ...centre } = {}) {
  const q = { waist: 0, neck_pan: 0, neck_tilt: 0, ...centre };
  for (const j of ARM) {
    q[`left_${j}`] = left[j] ?? both[j] ?? 0;
    q[`right_${j}`] = -(right[j] ?? both[j] ?? 0);
  }
  return { name, q };
}
export const HUMANOID_TORSO = robot('humanoid torso', 'pedestal', [
  torsoPose('Arms down'),
  torsoPose('Arms forward', { both: { elbow: -90 } }),
  torsoPose('Reach forward', { both: { shoulder_pitch: -90 } }),
  torsoPose('Carry', { both: { shoulder_pitch: -30, elbow: -60 } }),
  torsoPose('T-pose', { both: { shoulder_roll: 90 } }),
  torsoPose('Reach up', { both: { shoulder_pitch: -170 } }),
  // Wave: the upper arm out to the side, turned about its own axis so the elbow lifts the forearm up.
  torsoPose('Wave', { right: { shoulder_roll: 90, shoulder_yaw: -90, elbow: -90 }, neck_pan: -20 }),
  torsoPose('Wave 2', { right: { shoulder_roll: 90, shoulder_yaw: -90, elbow: -55 }, neck_pan: -20 }),
  torsoPose('Turn left', { waist: 45, neck_pan: 20, both: { elbow: -90 } }),
  torsoPose('Turn right', { waist: -45, neck_pan: -20, both: { elbow: -90 } }),
], [], [
  // Animations (the Animate view).
  animation('Wave', [['Arms down', 0.3], 'Wave', 'Wave 2', 'Wave', 'Wave 2', ['Wave', 0.3]]),
  animation('Pick up', [['Arms down', 0.3], ['Reach forward', 0.3], ['Carry', 0.5], 'Turn left', ['Reach forward', 0.3]]),
  animation('Exercise', ['Arms down', 'T-pose', 'Reach up', 'T-pose']),
  animation('Look around', [['Turn left', 0.5], ['Turn right', 0.5]], { speed: 25 }),
]);
