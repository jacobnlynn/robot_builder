// Example: hexapod with four joints per leg. A hexagonal mounting base carries six legs, two at the
// front, two at the sides and two at the back, each pointing straight out from the centre. Every leg
// has a hip yaw (a vertical axis, swinging the leg forward and back) and then three pitch joints
// whose axes are parallel, so the thigh, shin and foot all move in one vertical plane: the thigh
// motor at the hip, the knee and the ankle. With every joint at 0° each leg sticks straight out,
// level; Stand lifts the thigh and brings the shin and foot down so the foot stands just below z = 0.
// Every motor is placed in world coordinates (see build.js), so the links come out of the geometry.
import { builder, animation, v3, UP, at } from './build.js';

const { motor, link, robot } = builder();

// Legs at 30° steps round the body, front (+x) first, counterclockwise seen from above.
const LEGS = [['front_left', 30], ['middle_left', 90], ['back_left', 150], ['back_right', 210], ['middle_right', 270], ['front_right', 330]];
const RADIUS = 150; // hip axes, from the centre (mm)
const HIP_Z = 40; // height of the hip motors' outputs
// Along each leg (mm, at 0°): hip axis to thigh axis (and its rise), thigh, shin and foot.
const COXA = 90, COXA_RISE = 40, THIGH = 150, SHIN = 150, FOOT = 100;

for (const [key, deg] of LEGS) {
  const id = (p) => `${key}_${p}`;
  const a = deg * Math.PI / 180;
  const out = v3(Math.cos(a), Math.sin(a), 0); // straight out from the centre
  // The pitch axes lie across the leg, horizontal. The knee's points the other way, so its motor
  // sits on the other side of the leg from the thigh and ankle motors (they don't crowd each other):
  // a positive angle tips the leg down at the thigh and ankle, up at the knee.
  const across = v3(-Math.sin(a), Math.cos(a), 0);
  const along = (r, z) => out.clone().multiplyScalar(r).setZ(z);
  const pitchZ = HIP_Z + COXA_RISE;
  motor(id('hip'), id('hip'), at('body', LEGS.findIndex(([k]) => k === key)), ['cycloidal_18'], along(RADIUS, HIP_Z), UP, out, id('coxa'));
  motor(id('thigh'), id('thigh'), at(id('coxa')), ['cycloidal_18'], along(RADIUS + COXA, pitchZ), across, out, id('femur'));
  motor(id('knee'), id('knee'), at(id('femur')), ['cycloidal_18'], along(RADIUS + COXA + THIGH, pitchZ), across.clone().negate(), out, id('tibia'));
  motor(id('ankle'), id('ankle'), at(id('tibia')), ['planetary_9'], along(RADIUS + COXA + THIGH + SHIN, pitchZ), across, out, id('tarsus'));
  link(id('coxa'), id('coxa'), at(id('hip')), [id('thigh')]);
  link(id('femur'), id('femur'), at(id('thigh')), [id('knee')]);
  link(id('tibia'), id('tibia'), at(id('knee')), [id('ankle')]);
  link(id('tarsus'), id('tarsus'), at(id('ankle')), [{ sphere: 20, at: along(RADIUS + COXA + THIGH + SHIN + FOOT, pitchZ) }]);
}
link('body', 'Mounting Base', at('base'), LEGS.map(([k]) => `${k}_hip`));

// Leg angles [thigh, knee, ankle] that put the foot (straight down) at z = -10 with the thigh
// raised `lift` degrees: the shin's slope is solved so thigh, shin and foot reach that height.
function legAngles(lift) {
  const drop = (HIP_Z + COXA_RISE) + THIGH * Math.sin(lift * Math.PI / 180) - FOOT + 10;
  const shin = Math.asin(Math.min(1, drop / SHIN)) * 180 / Math.PI; // shin's angle below level
  return [-lift, -(lift + shin).toFixed(1), +(90 - shin).toFixed(1)];
}
const STAND = legAngles(30);
// A leg lifted for a step: the thigh up another 20°, the knee and ankle as when standing.
const LIFTED = [STAND[0] - 20, STAND[1], STAND[2]];

/** A pose from each leg's [hip, thigh, knee, ankle], given by a function of the leg's key and angle. */
const legsPose = (name, angles) => ({
  name,
  q: Object.fromEntries(LEGS.flatMap(([key, deg]) => {
    const [hip, thigh, knee, ankle] = angles(key, deg);
    return [[`${key}_hip`, hip], [`${key}_thigh`, thigh], [`${key}_knee`, knee], [`${key}_ankle`, ankle]];
  })),
});

// Walk: a tripod gait. Front left, back left and middle right step together, then the other three.
// A foot on the ground only turns about its hip's vertical axis, so it stays on the ground; the hip
// angle that swings a foot forward (+x) is negative on the left and positive on the right.
const TRIPOD = new Set(['front_left', 'back_left', 'middle_right']);
const SWING = 15;
const forward = (deg) => -Math.sign(Math.sin(deg * Math.PI / 180)) * SWING;
// Each step: the tripod `a` with its feet forward, in the middle or back, or lifted; the other opposite.
const walkPose = (name, phaseA, phaseB) => legsPose(name, (key, deg) => {
  const phase = TRIPOD.has(key) ? phaseA : phaseB;
  const f = forward(deg);
  if (phase === 'lift') return [0, ...LIFTED];
  return [phase === 'front' ? f : phase === 'back' ? -f : 0, ...STAND];
});

export const HEXAPOD_4DOF = robot('hexapod-4dof', 'body', [
  legsPose('Stand', () => [0, ...STAND]),
  legsPose('Tall', () => [0, ...legAngles(45)]),
  legsPose('Crouch', () => [0, ...legAngles(10)]),
  legsPose('Wave', (key) => (key === 'front_left' ? [0, -70, -40, 20] : [0, ...STAND])),
  walkPose('Walk 1', 'front', 'back'),
  walkPose('Walk 2', 'middle', 'lift'),
  walkPose('Walk 3', 'back', 'front'),
  walkPose('Walk 4', 'lift', 'middle'),
], [], [
  // Animations (the Animate view).
  animation('Walk', ['Walk 1', 'Walk 2', 'Walk 3', 'Walk 4'], { pause: false, speed: 100, walk: true }),
  animation('Wave', [['Stand', 0.3], ['Wave', 0.6]]),
  animation('Stretch', [['Stand', 0.3], ['Tall', 0.3], ['Crouch', 0.3]], { speed: 25 }),
]);
