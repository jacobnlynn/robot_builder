// Helpers for examples built in world coordinates (the Biped, Humanoid, SCARA and 6DOF arms):
// every motor is placed with every joint at 0° by saying where its output should be, which way the
// shaft points (its joint axis) and which way the motor stands up from its mounting face. Each link output is then a motor's mounting face (or a sphere end)
// measured from the world origin, so the links between motors come out of the geometry (solveLinks).
import * as THREE from 'three';
import { MOTORS } from '../constants.js';
import { matrixToPose, actuatorReach } from '../kinematics.js';

const ROD = { lon: 0, lat: 90 };
const zeroDh = () => ({ a: 0, alpha: 0, d: 0, theta: 0 });

export const v3 = (x, y, z) => new THREE.Vector3(x, y, z);
export const UP = v3(0, 0, 1), DOWN = v3(0, 0, -1), FWD = v3(1, 0, 0), BACK = v3(-1, 0, 0), LEFT = v3(0, 1, 0), RIGHT = v3(0, -1, 0);
/**
 * An animation (the Animate view): `steps` are pose names, or [name, hold] to hold that pose for
 * `hold` seconds. speed is % of each joint's top speed; pause eases to a stop at each pose; walk makes
 * it the one the Walk button plays (legged robots).
 */
export const animation = (name, steps, { loop = true, speed = 50, pause = true, walk = false } = {}) => ({
  name, loop, speed, pause, ...(walk ? { walk } : {}), steps: steps.map((s) => (Array.isArray(s) ? { pose: s[0], hold: s[1] } : { pose: s, hold: 0 })),
});
/** Where a part attaches: output `i` of part `id`. */
export const at = (id, i = 0) => ({ nodeId: id, index: i });

export function builder() {
  const nodes = [{
    id: 'base', type: 'link', name: 'Base', fixed: true, spec: 'dh', parent: null,
    outputs: [{ dh: zeroDh(), pose: { ...matrixToPose(new THREE.Matrix4()), ref: 'input' }, rod: { ...ROD } }], children: [null],
  }];
  const faces = {}; // motor id → mounting-face frame in world

  /** An actuator whose output (centre of the output face) is at `out` with the shaft along `axis`, standing up from its face along `up` (⊥ axis). */
  function motor(id, name, parent, gearboxes, out, axis, up, child = null) {
    const rise = MOTORS.m1.rise, reach = actuatorReach({ motor: 'm1', gearboxes });
    // Mounting face frame: x = output direction, z = up, so output = face + reach·x + rise·z.
    const x = axis.clone().normalize(), z = up.clone().normalize();
    const face = out.clone().addScaledVector(x, -reach).addScaledVector(z, -rise);
    faces[id] = new THREE.Matrix4().makeBasis(x, new THREE.Vector3().crossVectors(z, x), z).setPosition(face);
    nodes.push({ id, type: 'actuator', name, motor: 'm1', gearboxes, mount: 0, q: 0, parent, children: [child] });
  }

  /**
   * A link whose outputs are the named motors' mounting faces, sphere ends ({ sphere, at }), or
   * passive joints ({ joint: 'hinge' | 'ball', at, axis, child }: a hinge turns about `axis`, and
   * `child` is the link on it, if any). Call after those motors.
   */
  function link(id, name, parent, ends) {
    nodes.push({
      id, type: 'link', name, spec: 'pose', parent,
      outputs: ends.map((e) => {
        if (typeof e === 'string') return { dh: zeroDh(), pose: { ...matrixToPose(faces[e]), ref: 'world' }, rod: { ...ROD } };
        if (e.sphere) return { dh: zeroDh(), pose: { ...matrixToPose(new THREE.Matrix4().setPosition(e.at)), ref: 'world' }, rod: { ...ROD }, end: 'sphere', sphere: e.sphere };
        // Joint frame: z along the axis, x any direction square to it.
        const z = e.axis.clone().normalize(), x = new THREE.Vector3().crossVectors(Math.abs(z.z) < 0.9 ? UP : FWD, z).normalize();
        const m = new THREE.Matrix4().makeBasis(x, new THREE.Vector3().crossVectors(z, x), z).setPosition(e.at);
        return { dh: zeroDh(), pose: { ...matrixToPose(m), ref: 'world' }, rod: { ...ROD }, end: e.joint, ...(e.joint === 'hinge' ? { q: 0 } : { ball: { x: 0, y: 0, z: 0 } }) };
      }),
      children: ends.map((e) => (typeof e === 'string' ? e : e.child ?? null)),
    });
  }

  /**
   * The finished project. `first` is the part on the base; `poses` are [{ name, q: { id: degrees } }];
   * `loops` join pairs of open joint outputs, as [[linkA, outputA], [linkB, outputB]];
   * `animations` are made with animation().
   */
  function robot(name, first, poses = [], loops = [], animations = []) {
    nodes[0].children = [first];
    return {
      format: 'modular-robot', version: 1, convention: 'standard', units: { length: 'mm', angle: 'deg' }, name, rootId: 'base', nodes, poses,
      loops: loops.map(([[an, ai], [bn, bi]]) => ({ a: { nodeId: an, index: ai }, b: { nodeId: bn, index: bi } })),
      animations,
    };
  }

  return { motor, link, robot };
}

/**
 * A five-bar linkage (Doggo's legs, the five-bar SCARA): two motors side by side, each turning an
 * upper link; a lower link on a passive hinge at each upper link's tip; the two lower links meet at
 * the tip, where their hinges are joined in a closed loop. Everything turns about `axis`.
 *   b: a builder; seat(motorId): the port each motor sits on.
 *   centre: midway between the two motor outputs; along: from motor b towards motor a;
 *   reach: the way the linkage points from the motors (⊥ along and axis); up: how the motors stand
 *   up from their mounting faces (⊥ axis).
 *   upper, lower, spread (each motor's distance from centre, mm), splay (degrees each upper link
 *   leans out from `reach` with both motors at 0°), gears, tip: { sphere, offset } for the ball at
 *   the tip (offset from the meeting point), names: ids of the parts.
 * Returns { motors, loop } (loop as robot() takes it). With both motors at 0° the tip is on the centre line.
 */
export function fiveBar(b, seat, { centre, axis, along, reach, up, upper, lower, spread, splay, gears, tip, names }) {
  const p = (a, r) => centre.clone().addScaledVector(along, a).addScaledVector(reach, r);
  const s = Math.sin(splay * Math.PI / 180), c = Math.cos(splay * Math.PI / 180);
  const kneeA = spread + upper * s, kneeR = upper * c;
  const tipR = kneeR + Math.sqrt(lower ** 2 - kneeA ** 2);
  const n = names;
  b.motor(n.motorA, n.motorA, seat(n.motorA), gears, p(spread, 0), axis, up, n.upperA);
  b.motor(n.motorB, n.motorB, seat(n.motorB), gears, p(-spread, 0), axis, up, n.upperB);
  b.link(n.upperA, n.upperA, at(n.motorA), [{ joint: 'hinge', at: p(kneeA, kneeR), axis, child: n.lowerA }]);
  b.link(n.upperB, n.upperB, at(n.motorB), [{ joint: 'hinge', at: p(-kneeA, kneeR), axis, child: n.lowerB }]);
  b.link(n.lowerA, n.lowerA, at(n.upperA), [{ joint: 'hinge', at: p(0, tipR), axis }, { sphere: tip.sphere, at: p(0, tipR).add(tip.offset ?? v3(0, 0, 0)) }]);
  b.link(n.lowerB, n.lowerB, at(n.upperB), [{ joint: 'hinge', at: p(0, tipR), axis }]);
  return { motors: [n.motorA, n.motorB], loop: [[n.lowerA, 0], [n.lowerB, 0]] };
}
