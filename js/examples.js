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

export const EXAMPLES = [
  { id: 'robot-dog', label: 'Robot Dog', blurb: 'Four legs, three joints each', data: ROBOT_DOG },
  { id: 'hexapod', label: 'Hexapod', blurb: 'Six legs on a hexagonal body', data: HEXAPOD },
  { id: 'arm-7dof', label: '7DOF Arm', blurb: 'A seven-joint manipulator', data: ARM_7DOF },
  { id: 'humanoid', label: 'Humanoid', blurb: 'Arms, legs, waist and a pan/tilt head', data: HUMANOID },
  { id: 'robosimian', label: 'RoboSimian', blurb: "JPL's four-limbed robot, seven joints per limb", data: ROBOSIMIAN },
];
