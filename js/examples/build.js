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

  /** A link whose outputs are the named motors' mounting faces, or sphere ends ({ sphere, at }). Call after those motors. */
  function link(id, name, parent, ends) {
    nodes.push({
      id, type: 'link', name, spec: 'pose', parent,
      outputs: ends.map((e) => {
        const m = typeof e === 'string' ? faces[e] : new THREE.Matrix4().setPosition(e.at);
        return {
          dh: zeroDh(), pose: { ...matrixToPose(m), ref: 'world' }, rod: { ...ROD },
          ...(typeof e === 'string' ? {} : { end: 'sphere', sphere: e.sphere }),
        };
      }),
      children: ends.map((e) => (typeof e === 'string' ? e : null)),
    });
  }

  /** The finished project. `first` is the part on the base; `poses` are [{ name, q: { id: degrees } }]. */
  function robot(name, first, poses = []) {
    nodes[0].children = [first];
    return { format: 'modular-robot', version: 1, convention: 'standard', units: { length: 'mm', angle: 'deg' }, name, rootId: 'base', nodes, poses };
  }

  return { motor, link, robot };
}
