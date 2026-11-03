// Example robots offered on the start screen (the Examples button), grouped by category, in this
// order. The first one is loaded when the page opens.
// To add one: save the robot with Import/Export > Save JSON, save it as js/examples/<name>.js
// (`export const NAME = { ...the JSON... };`), import it here and add an entry with its category.
// Its card picture is img/examples/<id>.png (a robot with no picture gets a plain card).
import { ROBOT_DOG } from './examples/robot-dog.js';
import { HEXAPOD } from './examples/hexapod.js';
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

/** Start screen sections: [key, heading]. */
export const CATEGORIES = [
  ['legged', 'Legged robots'],
  ['arms', 'Arms'],
  ['mechanisms', 'Simple mechanisms'],
];

export const EXAMPLES = [
  { id: 'robot-dog', category: 'legged', label: 'Robot Dog', blurb: 'Four legs, three joints each', data: ROBOT_DOG },
  { id: 'doggo', category: 'legged', label: 'Doggo', blurb: 'Stanford Doggo style: four five-bar legs, two motors each', data: DOGGO },
  { id: 'hexapod', category: 'legged', label: 'Hexapod', blurb: 'Six legs on a hexagonal body', data: HEXAPOD },
  { id: 'biped', category: 'legged', label: 'Biped', blurb: 'Two walking legs, six joints each', data: BIPED },
  { id: 'biped-linkage', category: 'legged', label: 'Biped (linkage ankles)', blurb: 'Ankle motors up the shin push the foot through two rods', data: BIPED_LINKAGE },
  { id: 'humanoid', category: 'legged', label: 'Humanoid', blurb: '7-joint arms, 6-joint legs, waist and a pan/tilt head', data: HUMANOID },
  { id: 'robosimian', category: 'legged', label: 'RoboSimian', blurb: "JPL's four-limbed robot, seven joints per limb", data: ROBOSIMIAN },
  { id: 'humanoid-torso', category: 'arms', label: 'Humanoid torso', blurb: 'Two 7-joint arms, a waist and a pan/tilt head on a pedestal', data: HUMANOID_TORSO },
  { id: 'arm-7dof', category: 'arms', label: '7DOF Arm', blurb: 'A seven-joint manipulator', data: ARM_7DOF },
  { id: 'arm-6dof', category: 'arms', label: '6DOF Arm', blurb: 'The classic industrial arm with a three-axis wrist', data: ARM_6DOF },
  { id: 'pan-tilt', category: 'mechanisms', label: 'Pan-tilt', blurb: 'Two motors at right angles aim a camera', data: PAN_TILT },
  { id: 'fourbar', category: 'mechanisms', label: 'Four-bar', blurb: 'A crank-rocker linkage: one motor, a closed loop of hinges', data: FOURBAR },
  { id: 'scara', category: 'mechanisms', label: 'SCARA', blurb: 'Three vertical axes: a flat pick-and-place arm', data: SCARA },
  { id: 'scara-5bar', category: 'mechanisms', label: 'SCARA (5-bar)', blurb: 'A parallel SCARA: two motors on the column, one closed loop', data: SCARA_5BAR },
  { id: 'parallelogram', category: 'mechanisms', label: 'Parallelogram', blurb: 'Equal opposite bars keep the platform level as it swings', data: PARALLELOGRAM },
];
