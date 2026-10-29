// Example robots offered on the start screen (File > New), in this order.
// The first one is loaded when the page opens.
// To add one: save the robot with File > Save JSON, save it as js/examples/<name>.js
// (`export const NAME = { ...the JSON... };`), import it here and add an entry. Its card picture is
// img/examples/<id>.png (a robot with no picture gets a plain card).
import { ROBOT_DOG } from './examples/robot-dog.js';
import { HEXAPOD } from './examples/hexapod.js';
import { ARM_7DOF } from './examples/arm-7dof.js';
import { HUMANOID } from './examples/humanoid.js';
import { ROBOSIMIAN } from './examples/robosimian.js';
import { BIPED } from './examples/biped.js';
import { ARM_6DOF } from './examples/arm-6dof.js';
import { SCARA } from './examples/scara.js';

export const EXAMPLES = [
  { id: 'robot-dog', label: 'Robot Dog', blurb: 'Four legs, three joints each', data: ROBOT_DOG },
  { id: 'hexapod', label: 'Hexapod', blurb: 'Six legs on a hexagonal body', data: HEXAPOD },
  { id: 'biped', label: 'Biped', blurb: 'Two walking legs, six joints each', data: BIPED },
  { id: 'humanoid', label: 'Humanoid', blurb: '7-joint arms, 6-joint legs, waist and a pan/tilt head', data: HUMANOID },
  { id: 'robosimian', label: 'RoboSimian', blurb: "JPL's four-limbed robot, seven joints per limb", data: ROBOSIMIAN },
  { id: 'arm-7dof', label: '7DOF Arm', blurb: 'A seven-joint manipulator', data: ARM_7DOF },
  { id: 'arm-6dof', label: '6DOF Arm', blurb: 'The classic industrial arm with a three-axis wrist', data: ARM_6DOF },
  { id: 'scara', label: 'SCARA', blurb: 'Three vertical axes: a flat pick-and-place arm', data: SCARA },
];
