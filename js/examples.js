// Example robots offered as buttons at the left of the tool bar, in this order.
// The first one is loaded when the page opens.
// To add one: export the robot with Export JSON, save it as js/examples/<name>.js
// (`export const NAME = { ...the JSON... };`), import it here and add an entry.
import { ROBOT_DOG } from './examples/robot-dog.js';
import { HEXAPOD } from './examples/hexapod.js';
import { ARM_7DOF } from './examples/arm-7dof.js';
import { HUMANOID } from './examples/humanoid.js';

export const EXAMPLES = [
  { id: 'robot-dog', label: 'Robot Dog', data: ROBOT_DOG },
  { id: 'hexapod', label: 'Hexapod', data: HEXAPOD },
  { id: 'arm-7dof', label: '7DOF Arm', data: ARM_7DOF },
  { id: 'humanoid', label: 'Humanoid', data: HUMANOID },
];
