import * as THREE from 'three';
import { state, node } from './robot.js';
import { JOINT_LIMIT, SIM_MOTOR_SPEED } from './constants.js';
import { forwardKinematics, solveLinks, stackRatio } from './kinematics.js';

// Simulate mode: grab any part in the 3D view and drag it. The joints between the base and that
// part turn to follow the pointer (damped least-squares inverse kinematics on the grabbed point),
// each no faster than its speed limit.

const DAMPING = 30; // mm. Keeps the solve calm near singular poses (arm straight, axes lined up).
const ITERATIONS = 3; // solver steps per animation frame
const DEG = Math.PI / 180;

/** Fastest a joint may turn, in degrees per second: the motor speed divided by its gear reduction. */
export function jointSpeed(n) {
  const [num, den] = stackRatio(n.gearboxes);
  return SIM_MOTOR_SPEED / (num / den);
}

/** Actuators that move a part: every one between the base and it, plus its own if `own`. */
function drivingJoints(id, own) {
  const out = [];
  for (let n = node(id); n; n = n.parent && node(n.parent.nodeId)) {
    if (n.type === 'actuator' && (own || n.id !== id)) out.unshift(n);
  }
  return out;
}

/**
 * @param viewer   from createViewer
 * @param onLive   (frames) => void, every frame the joints move during a drag
 * @param onEnd    () => void, once when a drag that moved something ends
 */
export function initSim(viewer, { onLive, onEnd }) {
  const host = viewer.canvas.parentElement;
  let drag = null;

  const framesNow = () => forwardKinematics(state.robot, drag.solved);
  const grabbedPoint = (frames) => {
    const f = frames[drag.id];
    return drag.local.clone().applyMatrix4(drag.onOutput ? f.outputs[0] : f.input);
  };

  // Capture phase, so a grab turns orbiting off before the orbit controls see the press.
  host.addEventListener('pointerdown', (e) => {
    if (state.view !== 'sim' || e.button !== 0) return;
    const hit = viewer.pickAt(e.clientX, e.clientY);
    if (!hit) return; // empty space still orbits
    const { nodeId, onOutput } = hit.pick;
    const joints = drivingJoints(nodeId, onOutput);
    if (!joints.length) return; // the base and parts fixed to it don't move
    const solved = solveLinks(state.robot);
    const frames = forwardKinematics(state.robot, solved);
    const f = frames[nodeId];
    const local = hit.point.clone().applyMatrix4((onOutput ? f.outputs[0] : f.input).clone().invert());
    const normal = viewer.camera.getWorldDirection(new THREE.Vector3());
    drag = {
      id: nodeId, onOutput, local, joints, solved, moved: false,
      plane: new THREE.Plane().setFromNormalAndCoplanarPoint(normal, hit.point),
      target: hit.point.clone(),
    };
    viewer.controls.enabled = false;
    try { viewer.canvas.setPointerCapture(e.pointerId); } catch { /* pointer already gone */ }
    host.classList.add('is-dragging');
    viewer.setDragMarker(hit.point, hit.point);
    e.preventDefault();
  }, { capture: true });

  host.addEventListener('pointermove', (e) => {
    if (state.view !== 'sim') return;
    if (!drag) {
      host.classList.toggle('can-grab', !!viewer.pickAt(e.clientX, e.clientY));
      return;
    }
    viewer.rayAt(e.clientX, e.clientY).ray.intersectPlane(drag.plane, drag.target);
  });

  const release = () => {
    if (!drag) return;
    const moved = drag.moved;
    drag = null;
    viewer.controls.enabled = true;
    host.classList.remove('is-dragging');
    viewer.setDragMarker(null);
    if (moved) {
      for (const n of Object.values(state.robot.nodes)) if (n.type === 'actuator') n.q = Math.round(n.q * 10) / 10;
      onEnd();
    }
  };
  host.addEventListener('pointerup', release);
  host.addEventListener('pointercancel', release);

  viewer.onFrame((dt) => {
    if (!drag) return;
    if (state.view !== 'sim') return release();
    // Each joint has this many degrees left to turn this frame.
    const budget = drag.joints.map((n) => jointSpeed(n) * dt);
    let frames = framesNow();
    let p = grabbedPoint(frames);
    for (let it = 0; it < ITERATIONS; it++) {
      const err = new THREE.Vector3().subVectors(drag.target, p);
      if (err.length() < 0.2) break;
      // Linear Jacobian of the grabbed point: z_j × (p − p_j) per joint, mm per rad.
      const cols = drag.joints.map((n) => {
        const m = frames[n.id].outputs[0].elements;
        const z = new THREE.Vector3(m[8], m[9], m[10]);
        return z.cross(new THREE.Vector3(p.x - m[12], p.y - m[13], p.z - m[14]));
      });
      // dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
      const A = new THREE.Matrix3().set(DAMPING ** 2, 0, 0, 0, DAMPING ** 2, 0, 0, 0, DAMPING ** 2);
      const a = A.elements; // column-major, symmetric here
      for (const c of cols) {
        const v = [c.x, c.y, c.z];
        for (let r = 0; r < 3; r++) for (let k = 0; k < 3; k++) a[k * 3 + r] += v[r] * v[k];
      }
      const y = err.clone().applyMatrix3(A.invert());
      let any = false;
      drag.joints.forEach((n, j) => {
        const want = cols[j].dot(y) / DEG;
        const step = Math.max(-budget[j], Math.min(budget[j], want));
        const q = Math.max(-JOINT_LIMIT, Math.min(JOINT_LIMIT, n.q + step));
        budget[j] -= Math.abs(q - n.q);
        if (q !== n.q) any = true;
        n.q = q;
      });
      if (!any) break;
      drag.moved = true;
      frames = framesNow();
      p = grabbedPoint(frames);
    }
    viewer.pose(frames);
    viewer.setDragMarker(p, drag.target);
    if (drag.moved) onLive(frames);
  });
}
