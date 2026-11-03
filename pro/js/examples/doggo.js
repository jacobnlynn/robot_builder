// Example: Doggo, a quadruped with five-bar legs like Stanford Doggo's. Each leg has two motors
// side by side at the hip (Doggo's are coaxial; these right-angle modules can't share one axis, so
// they sit 80 mm apart along the body), each turning a 90 mm upper link. A 160 mm lower link hangs
// from each upper link on a passive hinge, and the two lower links meet at the foot, where their
// hinges are joined in a closed loop (see fiveBar in build.js). Turning both motors the same way
// swings the foot forward or back; turning them opposite ways pulls the foot up or pushes it down.
import { builder, fiveBar, animation, v3, DOWN, FWD, at } from './build.js';

const b = builder();
const seats = []; // motor ids, in the order of the body's outputs
const seat = (id) => at('body', seats.push(id) - 1);

// Four legs under a 400 × 220 mm body.
const LEGS = [['fl', 200, 1], ['fr', 200, -1], ['rl', -200, 1], ['rr', -200, -1]];
const legs = LEGS.map(([key, x, s]) => {
  const id = (part) => `${key}_${part}`;
  const leg = fiveBar(b, seat, {
    centre: v3(x, 110 * s, 0), axis: v3(0, s, 0), along: FWD, reach: DOWN, up: DOWN,
    upper: 90, lower: 160, spread: 40, splay: 30, gears: ['planetary_3'], // Doggo drives each link through a 3:1 belt
    tip: { sphere: 18 },
    names: { motorA: id('motor_a'), motorB: id('motor_b'), upperA: id('upper_a'), upperB: id('upper_b'), lowerA: id('lower_a'), lowerB: id('lower_b') },
  });
  return { key, s, ...leg };
});
b.link('body', 'body', at('base'), seats);

// Poses. For a left leg, turning both motors + swings the foot back; turning motor a − and motor b +
// pulls the foot up. The right legs' shafts point the other way, so their angles are negated.
function dogPose(name, perLeg) {
  const q = {};
  for (const { key, s } of legs) {
    const [qa, qb] = perLeg(key);
    q[`${key}_motor_a`] = s * qa;
    q[`${key}_motor_b`] = s * qb;
  }
  return { name, q };
}
const lift = (d) => [-d, d], swing = (d) => [d, d]; // lift: foot up by turning the motors apart; swing: foot back
const add = ([a, c], [d, e]) => [a + d, c + e];
// Walk: a trot in four poses (fl and rr together, then fr and rl), worked out by inverse kinematics so feet
// on the ground stay level and move back while the other pair lifts and swings forward.
const WALK = [
  { fl: [-11.5, -14.5], fr: [14.5, 11.5], rl: [14.5, 11.5], rr: [-11.5, -14.5] },
  { fl: [0, 0], fr: [-15.5, 15.5], rl: [-15.5, 15.5], rr: [0, 0] },
  { fl: [14.5, 11.5], fr: [-11.5, -14.5], rl: [-11.5, -14.5], rr: [14.5, 11.5] },
  { fl: [-15.5, 15.5], fr: [0, 0], rl: [0, 0], rr: [-15.5, 15.5] },
];
export const DOGGO = b.robot('Doggo', 'body', [
  dogPose('Stand', () => [0, 0]),
  dogPose('Crouch', () => lift(30)),
  dogPose('Tall', () => lift(-25)),
  dogPose('Lean forward', () => swing(25)),
  dogPose('Trot step', (k) => (k === 'fl' || k === 'rr' ? add(lift(30), swing(-20)) : swing(15))),
  dogPose('Sit', (k) => (k[0] === 'r' ? lift(40) : lift(-20))),
  ...WALK.map((w, i) => dogPose(`Walk ${i + 1}`, (k) => w[k])),
], legs.map((l) => l.loop), [
  // Animations (the Animate view).
  animation('Walk', ['Walk 1', 'Walk 2', 'Walk 3', 'Walk 4'], { pause: false, speed: 100, walk: true }),
  animation('Bounce', ['Crouch', 'Tall']),
  animation('Sit', [['Stand', 0.5], ['Sit', 1]], { speed: 25 }),
  animation('Lean', ['Stand', 'Lean forward', 'Stand', 'Crouch'], { speed: 25 }),
]);
