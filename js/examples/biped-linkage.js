// Example: biped with linkage-driven ankles, the layout of Agility's Digit and many recent humanoids.
// Hips and knees are the Biped example's, but the ankle has no motors of its own: the foot sits on
// a passive universal joint under the shin (a pitch hinge, then a roll hinge, through a small cross
// link), and two motors high on the back of the shin (where their weight
// swings less) each turn a 50 mm crank. From each crank a rod with a ball joint at both ends runs
// down to the back of the foot, one each side of the heel, and its lower end is joined to the foot in
// a closed loop. Turning both ankle motors the same way pushes or pulls both rods, pitching the foot;
// turning them opposite ways rolls it.
import { builder, animation, v3, DOWN, FWD, BACK, at } from './build.js';

const { motor, link, robot } = builder();
const SIDES = [['left', 1], ['right', -1]];
const Y = 230; // the leg's plane, either side of the centre line
const ROD_Y = 35; // the rods run this far either side of the leg's plane

for (const [side, s] of SIDES) {
  const id = (p) => `${side}_${p}`;
  const OUT = v3(0, s, 0), IN = v3(0, -s, 0);
  const y = (d) => (Y + d) * s;
  motor(id('hip_yaw'), id('hip_yaw'), at('pelvis', side === 'left' ? 0 : 1), ['cycloidal_18'], v3(0, 110 * s, -40), DOWN, BACK, id('hip'));
  motor(id('hip_roll'), id('hip_roll'), at(id('hip')), ['cycloidal_18', 'cycloidal_18'], v3(100, 110 * s, -170), FWD, DOWN, id('hip_block'));
  motor(id('hip_pitch'), id('hip_pitch'), at(id('hip_block')), ['cycloidal_18', 'cycloidal_18'], v3(0, y(0), -240), OUT, DOWN, id('thigh'));
  motor(id('knee'), id('knee'), at(id('thigh')), ['harmonic_45_2'], v3(0, y(0), -500), OUT, DOWN, id('shin'));
  // Ankle motors on the back of the shin, one above the other, shafts pointing out and in.
  motor(id('ankle_a'), id('ankle_a'), at(id('shin'), 1), ['cycloidal_18'], v3(-70, y(ROD_Y), -580), OUT, BACK, id('crank_a'));
  motor(id('ankle_b'), id('ankle_b'), at(id('shin'), 2), ['cycloidal_18'], v3(-70, y(-ROD_Y), -660), IN, BACK, id('crank_b'));
}

link('pelvis', 'pelvis', at('base'), ['left_hip_yaw', 'right_hip_yaw']);
for (const [side, s] of SIDES) {
  const id = (p) => `${side}_${p}`;
  const y = (d) => (Y + d) * s;
  const heelA = v3(-90, y(ROD_Y), -830), heelB = v3(-90, y(-ROD_Y), -830);
  link(id('hip'), id('hip'), at(id('hip_yaw')), [id('hip_roll')]);
  link(id('hip_block'), id('hip_block'), at(id('hip_roll')), [id('hip_pitch')]);
  link(id('thigh'), id('thigh'), at(id('hip_pitch')), [id('knee')]);
  // Shin: the ankle's pitch hinge, then the two ankle motors' faces. The cross link turns on that
  // hinge and carries the roll hinge the foot hangs on: together a universal joint.
  const ankle = v3(0, y(0), -800);
  link(id('shin'), id('shin'), at(id('knee')), [{ joint: 'hinge', at: ankle, axis: v3(0, s, 0), child: id('ankle_cross') }, id('ankle_a'), id('ankle_b')]);
  link(id('ankle_cross'), id('ankle_cross'), at(id('shin'), 0), [{ joint: 'hinge', at: ankle, axis: FWD, child: id('foot') }]);
  link(id('crank_a'), id('crank_a'), at(id('ankle_a')), [{ joint: 'ball', at: v3(-120, y(ROD_Y), -580), axis: DOWN, child: id('rod_a') }]);
  link(id('crank_b'), id('crank_b'), at(id('ankle_b')), [{ joint: 'ball', at: v3(-120, y(-ROD_Y), -660), axis: DOWN, child: id('rod_b') }]);
  link(id('rod_a'), id('rod_a'), at(id('crank_a')), [{ joint: 'ball', at: heelA, axis: DOWN }]);
  link(id('rod_b'), id('rod_b'), at(id('crank_b')), [{ joint: 'ball', at: heelB, axis: DOWN }]);
  // Foot: the two rods' sockets at the back, then balls at the toe and heel.
  link(id('foot'), id('foot'), at(id('ankle_cross')), [
    { joint: 'ball', at: heelA, axis: DOWN }, { joint: 'ball', at: heelB, axis: DOWN },
    { sphere: 25, at: v3(150, y(0), -880) }, { sphere: 25, at: v3(-80, y(0), -880) },
  ]);
}

// Saved poses. Angles are given as for the left leg; the right leg is its mirror image, so every
// right joint is negated. `ankle: [pitch, roll]` sets the two ankle motors: pitch pushes both rods
// the same way (+ is toes down), roll pushes them opposite ways.
const LEG = ['hip_yaw', 'hip_roll', 'hip_pitch', 'knee', 'ankle_a', 'ankle_b'];
function legs(name, left = {}, right = left) {
  const q = {};
  const angles = (o) => {
    const [pitch = 0, roll = 0] = o.ankle ?? [];
    return { ...o, ankle_a: pitch + roll, ankle_b: -pitch + roll };
  };
  const L = angles(left), R = angles(right);
  for (const j of LEG) {
    q[`left_${j}`] = L[j] ?? 0;
    q[`right_${j}`] = -(R[j] ?? 0);
  }
  return { name, q };
}
export const BIPED_LINKAGE = robot('Biped (linkage ankles)', 'pelvis', [
  legs('Stand'),
  // About two degrees of ankle motor per degree of foot pitch, so a flat-footed crouch needs -80.
  legs('Crouch', { hip_pitch: -40, knee: 80, ankle: [-80, 0] }),
  legs('Step', { hip_pitch: -30, knee: 20, ankle: [-15, 0] }, { hip_pitch: 15, knee: 30, ankle: [-35, 0] }),
  legs('Toes up', { ankle: [-30, 0] }),
  legs('Toes down', { ankle: [30, 0] }),
  legs('Ankles roll', { ankle: [0, 20] }),
  legs('Ankles roll in', { ankle: [0, -20] }),
  // Walk: a gait cycle in four poses, worked out by inverse kinematics so feet on the ground stay level and
  // move back while the others lift and swing forward (the body is fixed, so it walks on the spot).
  legs('Walk 1', { hip_pitch: -30, knee: 42, ankle: [-24, 0] }, { hip_pitch: -15, knee: 42, ankle: [-51.5, 0] }),
  legs('Walk 2', { hip_pitch: -24, knee: 44.5, ankle: [-39.5, 0] }, { hip_pitch: -37.5, knee: 70, ankle: [-62.5, 0] }),
  legs('Walk 3', { hip_pitch: -15, knee: 42, ankle: [-51.5, 0] }, { hip_pitch: -30, knee: 42, ankle: [-24, 0] }),
  legs('Walk 4', { hip_pitch: -37.5, knee: 70, ankle: [-62.5, 0] }, { hip_pitch: -24, knee: 44.5, ankle: [-39.5, 0] }),
], [
  ...SIDES.flatMap(([side]) => [[[`${side}_rod_a`, 0], [`${side}_foot`, 0]], [[`${side}_rod_b`, 0], [`${side}_foot`, 1]]]),
], [
  // Animations (the Animate view).
  animation('Walk', ['Walk 1', 'Walk 2', 'Walk 3', 'Walk 4'], { pause: false, speed: 100, walk: true }),
  animation('Squat', ['Stand', ['Crouch', 0.5]], { speed: 25 }),
  animation('Ankle circles', ['Toes up', 'Ankles roll', 'Toes down', 'Ankles roll in'], { pause: false, speed: 25 }),
]);
