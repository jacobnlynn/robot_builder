// Example: humanoid. A pelvis on the base (at the world origin) carries a waist joint and torso
// above it, with two arms (shoulder pitch, shoulder roll, elbow, and a sphere for a hand) and a
// head on a pan/tilt neck, and two legs below it (hip pitch, hip roll, knee, ankle, and a sphere
// for a foot).
// Every motor is placed in world coordinates with every joint at 0°: `motor()` takes where its
// output should be, which way the output shaft points (its joint axis) and which way the motor
// stands up from its mounting face, and works out where that face goes. Each link output is
// then that face, measured from the world origin, so the links between motors come out of the
// geometry (solveLinks).
import * as THREE from 'three';
import { MOTORS } from '../constants.js';
import { matrixToPose, actuatorReach } from '../kinematics.js';

const ROD = { lon: 0, lat: 90 };
const zeroDh = () => ({ a: 0, alpha: 0, d: 0, theta: 0 });
const v3 = (x, y, z) => new THREE.Vector3(x, y, z);

const nodes = [];
const motors = {}; // id → mounting-face frame in world

/**
 * Add an actuator whose output (centre of the output face) is at `out` with the shaft along
 * `axis`, standing up from its mounting face along `up` (⊥ axis).
 */
function motor(id, name, parent, gearboxes, out, axis, up, child) {
  const rise = MOTORS.m1.rise, reach = actuatorReach({ motor: 'm1', gearboxes });
  // Mounting face frame: x = output direction, z = up, so output = face + reach·x + rise·z.
  const x = axis.clone().normalize(), z = up.clone().normalize();
  const face = out.clone().addScaledVector(x, -reach).addScaledVector(z, -rise);
  motors[id] = new THREE.Matrix4().makeBasis(x, new THREE.Vector3().crossVectors(z, x), z).setPosition(face);
  nodes.push({ id, type: 'actuator', name, motor: 'm1', gearboxes, mount: 0, q: 0, parent, children: [child] });
}

/** Add a link whose outputs are given motors' mounting faces, or sphere ends ({ sphere, at }). */
function link(id, name, parent, ends) {
  nodes.push({
    id, type: 'link', name, spec: 'pose', parent,
    outputs: ends.map((e) => {
      const m = typeof e === 'string' ? motors[e] : new THREE.Matrix4().setPosition(e.at);
      return {
        dh: zeroDh(), pose: { ...matrixToPose(m), ref: 'world' }, rod: { ...ROD },
        ...(typeof e === 'string' ? {} : { end: 'sphere', sphere: e.sphere }),
      };
    }),
    children: ends.map((e) => (typeof e === 'string' ? e : null)),
  });
}

const UP = v3(0, 0, 1), DOWN = v3(0, 0, -1), FWD = v3(1, 0, 0), BACK = v3(-1, 0, 0);
const at = (id, i = 0) => ({ nodeId: id, index: i });

// Motors first (their faces are needed by the links), then the links that join them.
// Waist: turns the whole torso about the vertical, on top of a 0.25 m pedestal.
motor('waist', 'waist', at('pedestal', 0), ['harmonic_45_2', 'cycloidal_18'], v3(0, 0, 300), UP, BACK, 'torso');
// Neck: pan about the vertical, then tilt (nod) about the side-to-side axis.
motor('neck_pan', 'neck_pan', at('torso', 2), ['planetary_5'], v3(0, 0, 640), UP, BACK, 'neck');
motor('neck_tilt', 'neck_tilt', at('neck'), ['planetary_5'], v3(0, 40, 720), v3(0, 1, 0), UP, 'head');

for (const [side, s] of [['left', 1], ['right', -1]]) {
  const OUT = v3(0, s, 0);
  const id = (p) => `${side}_${p}`;
  // Shoulder pitch swings the arm forward and back; it sits on the torso's shoulder plate.
  motor(id('shoulder_pitch'), id('shoulder_pitch'), at('torso', side === 'left' ? 0 : 1), ['cycloidal_18', 'cycloidal_18'],
    v3(0, 190 * s, 560), OUT, UP, id('shoulder'));
  // Shoulder roll lifts the arm out to the side.
  motor(id('shoulder_roll'), id('shoulder_roll'), at(id('shoulder')), ['cycloidal_18'],
    v3(60, 290 * s, 520), FWD, UP, id('upper_arm'));
  // Elbow bends the forearm; it hangs under the upper arm.
  motor(id('elbow'), id('elbow'), at(id('upper_arm')), ['planetary_9'],
    v3(0, 300 * s, 300), OUT, DOWN, id('forearm'));
}

for (const [side, s] of [['left', 1], ['right', -1]]) {
  const OUT = v3(0, s, 0);
  const id = (p) => `${side}_${p}`;
  // Hip pitch swings the leg forward and back; it hangs from the pelvis.
  motor(id('hip_pitch'), id('hip_pitch'), at('pedestal', side === 'left' ? 1 : 2), ['cycloidal_18', 'cycloidal_18'],
    v3(0, 120 * s, -60), OUT, DOWN, id('hip'));
  // Hip roll swings the leg out to the side.
  motor(id('hip_roll'), id('hip_roll'), at(id('hip')), ['cycloidal_18'],
    v3(60, 200 * s, -100), FWD, DOWN, id('thigh'));
  // Knee and ankle bend about the side-to-side axis, each hanging under the part above.
  motor(id('knee'), id('knee'), at(id('thigh')), ['harmonic_45_2'],
    v3(0, 210 * s, -330), OUT, DOWN, id('shin'));
  motor(id('ankle'), id('ankle'), at(id('shin')), ['planetary_9'],
    v3(0, 210 * s, -580), OUT, DOWN, id('foot'));
}

nodes.unshift(
  { id: 'base', type: 'link', name: 'Base', fixed: true, spec: 'dh', parent: null,
    outputs: [{ dh: zeroDh(), pose: { ...matrixToPose(new THREE.Matrix4()), ref: 'input' }, rod: { ...ROD } }], children: ['pedestal'] },
);
link('pedestal', 'pelvis', at('base'), ['waist', 'left_hip_pitch', 'right_hip_pitch']);
link('torso', 'torso', at('waist'), ['left_shoulder_pitch', 'right_shoulder_pitch', 'neck_pan']);
link('neck', 'neck', at('neck_pan'), ['neck_tilt']);
link('head', 'head', at('neck_tilt'), [{ sphere: 70, at: v3(0, 0, 830) }]);
for (const [side, s] of [['left', 1], ['right', -1]]) {
  const id = (p) => `${side}_${p}`;
  link(id('shoulder'), id('shoulder'), at(id('shoulder_pitch')), [id('shoulder_roll')]);
  link(id('upper_arm'), id('upper_arm'), at(id('shoulder_roll')), [id('elbow')]);
  link(id('forearm'), id('forearm'), at(id('elbow')), [{ sphere: 32, at: v3(230, 330 * s, 300) }]);
  link(id('hip'), id('hip'), at(id('hip_pitch')), [id('hip_roll')]);
  link(id('thigh'), id('thigh'), at(id('hip_roll')), [id('knee')]);
  link(id('shin'), id('shin'), at(id('knee')), [id('ankle')]);
  link(id('foot'), id('foot'), at(id('ankle')), [{ sphere: 35, at: v3(70, 230 * s, -650) }]);
}

export const HUMANOID = {
  format: 'modular-robot',
  version: 1,
  convention: 'standard',
  units: { length: 'mm', angle: 'deg' },
  name: 'humanoid',
  rootId: 'base',
  nodes,
};
