// Example robots offered on the start screen (the Examples button), grouped by category, in this
// order. The first one is loaded when the page opens.
// To add one: save the robot with Import/Export > Save JSON, save it as js/examples/<name>.js
// (`export const NAME = { ...the JSON... };`), import it here and add an entry with its category.
// Its card picture is img/examples/<id>.png (a robot with no picture gets a plain card).
import { ROBOT_DOG } from './examples/robot-dog.js';
import { HEXAPOD } from './examples/hexapod.js';
import { HEXAPOD_4DOF } from './examples/hexapod-4dof.js';
import { ARM_7DOF } from './examples/arm-7dof.js';
import { HUMANOID } from './examples/humanoid.js';
import { HUMANOID_TORSO } from './examples/humanoid-torso.js';
import { ROBOSIMIAN } from './examples/robosimian.js';
import { BIPED } from './examples/biped.js';
import { ARM_6DOF } from './examples/arm-6dof.js';
import { SCARA } from './examples/scara.js';
import { FOURBAR } from './examples/fourbar.js';
import { DOGGO } from './examples/doggo.js';
import { SCARA_5BAR } from './examples/scara-5bar.js';
import { BIPED_LINKAGE } from './examples/biped-linkage.js';
import { PAN_TILT } from './examples/pan-tilt.js';
import { PARALLELOGRAM } from './examples/parallelogram.js';
import { SWERVE } from './examples/swerve.js';
import { ROCKER_BOGIE } from './examples/rocker-bogie.js';
import { WHEEL_DOG } from './examples/wheel-dog.js';

/** Start screen sections: [key, heading]. */
export const CATEGORIES = [
  ['legged', 'Legged robots'],
  ['arms', 'Arms'],
  ['wheeled', 'Wheeled robots'],
  ['mechanisms', 'Simple mechanisms'],
];

export const EXAMPLES = [
  { id: 'robot-dog', category: 'legged', label: 'Robot Dog', blurb: 'Four legs with three motors each', data: ROBOT_DOG },
  { id: 'doggo', category: 'legged', label: 'Doggo', blurb: 'Four five-bar legs, based on Stanford Doggo', data: DOGGO },
  { id: 'hexapod', category: 'legged', label: 'Hexapod', blurb: 'Six legs with three motors each', data: HEXAPOD },
  { id: 'hexapod-4dof', category: 'legged', label: 'Hexapod (4DOF legs)', blurb: 'Six legs with four motors each', data: HEXAPOD_4DOF },
  { id: 'biped', category: 'legged', label: 'Biped', blurb: 'Two legs with six motors each', data: BIPED },
  { id: 'biped-linkage', category: 'legged', label: 'Biped (linkage ankles)', blurb: 'Biped with the ankle motors on the shins, driving the feet through rods', data: BIPED_LINKAGE },
  { id: 'humanoid', category: 'legged', label: 'Humanoid', blurb: 'Arms, legs, a waist and a head that pans and tilts', data: HUMANOID },
  { id: 'robosimian', category: 'legged', label: 'RoboSimian', blurb: "Four limbs with seven motors each, based on JPL's RoboSimian", data: ROBOSIMIAN },
  { id: 'humanoid-torso', category: 'arms', label: 'Humanoid torso', blurb: 'Two arms, a waist and a head on a fixed stand', data: HUMANOID_TORSO },
  { id: 'arm-7dof', category: 'arms', label: '7DOF Arm', blurb: 'Arm with seven motors', data: ARM_7DOF },
  { id: 'arm-6dof', category: 'arms', label: '6DOF Arm', blurb: 'Industrial-style arm with a three-motor wrist', data: ARM_6DOF },
  { id: 'swerve', category: 'wheeled', label: 'Swerve drive', blurb: 'FRC-style base where every wheel steers and drives', data: SWERVE },
  { id: 'rocker-bogie', category: 'wheeled', label: 'Six-wheeled rover', blurb: 'Six wheels on a rocker-bogie suspension', data: ROCKER_BOGIE },
  { id: 'wheel-dog', category: 'wheeled', label: 'Wheeled robot dog', blurb: 'Robot dog with a driven wheel on each leg', data: WHEEL_DOG },
  { id: 'pan-tilt', category: 'mechanisms', label: 'Pan-tilt', blurb: 'Two motors that point a camera', data: PAN_TILT },
  { id: 'fourbar', category: 'mechanisms', label: 'Four-bar', blurb: 'One motor driving a four-bar linkage', data: FOURBAR },
  { id: 'scara', category: 'mechanisms', label: 'SCARA', blurb: 'Flat pick-and-place arm', data: SCARA },
  { id: 'scara-5bar', category: 'mechanisms', label: 'SCARA (5-bar)', blurb: 'SCARA arm driven by two motors through a five-bar linkage', data: SCARA_5BAR },
  { id: 'parallelogram', category: 'mechanisms', label: 'Parallelogram', blurb: 'Linkage that keeps the top bar level as it swings', data: PARALLELOGRAM },
];
