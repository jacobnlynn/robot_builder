# Reconfigurable Robot Builder

Browser tool for assembling a robot from modular actuators (motor + gearbox stack) and links, defined by D-H parameters.

## Run it

No build step. ES modules need to be served over HTTP, so from this folder:

```
python -m http.server 8000
```

Then open http://localhost:8000. Three.js loads from the jsDelivr CDN, so you need an internet connection.

## Files

| File | What it holds |
| --- | --- |
| `js/constants.js` | Motor and gearbox library (offsets, ratios, sizes, colours), D-H convention for links. |
| `js/kinematics.js` | D-H → 4×4 matrix, per-part local transforms, forward kinematics tree walk, gear ratio maths |
| `js/robot.js` | Robot tree data model, add/remove/edit, save/load JSON |
| `js/viewer.js` | Three.js scene, CAD and simplified renderers, picking, grow-in animation, CAD tab module loading |
| `js/stepexport.js` | Export STEP: link parts rebuilt as solids with opencascade.js (loaded from jsDelivr on first use, about 50 MB), and the whole-robot STEP assembly |
| `js/poses.js` | Saved poses, listed above the Assembly list (with Zero all): click one to move every joint there (each at its top speed), or Save pose to keep the current angles. Saved in the project JSON as `poses: [{ name, q: { actuatorId: degrees } }]` |
| `js/simulate.js` | Simulate mode: drag a part and the joints before it follow (damped least-squares IK), each capped at its top speed |
| `js/workspace.js` | Reachable and dexterous workspaces for simulate mode: random joint samples binned into voxels, wrapped with marching cubes |
| `js/linkfill.js` | Trial: filled links. Each printed part (a link, or a run of links with no motor between them, which print as one piece) becomes one solid: input connector, straight round or square tubes (Tubes toggle in the CAD tab) with a 90° bend to each output, tapered collars fusing the pipe ends into the connectors, minus the motors around it. Uses manifold-3d (vendored in `vendor/manifold-3d-2.5.1/`, Apache-2.0). Shown in the CAD tab and exported as STEP from the link inspector (Export link STEP). Remove with its import line in main.js. |
| `js/tutorial.js` | Guided tour (opens every time the site opens; replay from ? > Tutorial; when it first closes, the start screen opens): views, building a small arm step by step, exports. Each step sets up its own state, so Back works. |
| `js/start.js` | Start screen: a card for each example (picture from `img/examples/<id>.png`) and a Blank one. Opens from File > New and after the tutorial first closes. |
| `js/menus.js` | The two top bar menus: File (New, Open JSON, Save JSON, Export STEP, Export URDF) and ? (Tutorial, Style). Opens and closes them; the items are ordinary buttons with their own handlers. |
| `js/styles.js`, `css/theme-*.css` | Style choices in the ? menu: Retro (default), Normal (the Datasheet theme) or Risograph. Each theme is a stylesheet loaded after style.css plus optional 3D colours; the choice is remembered in the browser. |
| `js/ui.js` | Assembly tree (with a joint slider under each actuator, the search box and Collapse all), inspector forms, status bar, project file dialog |
| `js/examples.js`, `js/examples/` | Example robots (Robot Dog, Hexapod, 7DOF Arm, Humanoid, RoboSimian) offered on the start screen, each with its saved poses; the first one loads when the page opens. To add one, save it with File > Save JSON, store it as `js/examples/<name>.js` (`export const NAME = { …JSON… };`) and add an entry to `EXAMPLES`. Its start screen picture is `img/examples/<id>.png` (360 × 270, transparent background). |

## Model

- The robot is a tree. Every part has one input (a parent port) and one or more outputs.
- Base: the root link, locked in place on the world origin (it can't be deleted). It starts with every value at zero and is edited like any other link.
- Actuator: right-angle. The input face points up its z axis; the output shaft is turned 90° to point along the input's +x.
  `T = Rz(φ) · Tx(h) · Tz(v) · Ry(90°) · Rz(q)`, where φ is the mount angle (how far the motor is turned about the port's z axis when bolted on), with v = 2.225 in and h = 1.472 in + 1.6 in per gearbox after the first. Every actuator has at least one gearbox; the first sits inside the 3 in motor housing, which is centred on the input.
- Link: each output has its own transform from the shared input, entered either as D-H parameters or as XYZ + angles (toggle per link).
  - XYZ + angles: position x, y, z and rotations α, β, γ about the reference frame's x, then y, then z axes (`T = Trans(x,y,z) · Rz(γ) · Ry(β) · Rx(α)`).
  - Rod direction: longitude (about the port's z axis, from +x toward +y) and latitude (up from the port's x-y plane; 90° is straight up z). The rod runs that way, bends at the point nearest the output and goes straight to it. Shape only; the output frame comes from the pose.
  - The reference frame defaults to the link's port, but can be the world origin or any other part's output. Since a link is rigid, a pose measured from another frame is solved with every joint at 0° and then stays fixed to the link (`solveLinks` in kinematics.js).
- Output: each link output is either another output (a port to keep building from) or a sphere end of adjustable radius, centred on the output, which ends the robot there. A sphere end is still an end effector in the Robot data window.
- D-H and XYZ are input formats; everything is stored and composed as 4×4 matrices.
- Robot data window (top right): for every end effector (open output) the geometric Jacobian in the base frame with rank, manipulability, condition number and singular values; a joint table; base → end-effector transforms; and motor-input → next-motor-input transforms through the links between them. All at the current joint angles. Each Jacobian also unfolds into its formula for any joint angles, T_e(q) = A₀·Rz(q₁)·A₁·…·Rz(qₖ)·Aₖ with the constant A matrices listed and J_i = [z_i × (p_e − p_i); z_i], plus the same formula as a copyable Python function.
- Joints: every actuator row in the Assembly list has its joint slider under it. Collapse all folds every limb (each branch of a multi-output link) to its first part; the search box lists just the parts whose names match.
- Poses (above the Assembly list): every example comes with a few (the Robot Dog has Stand, Crouch, Tall, Sit and Shake). Click one and the joints move there together; Save pose stores the current angles under a name (the same name replaces it), and × deletes one. Poses are saved with the robot in Export JSON.
- Simulate (left of Model): shows the Model view; drag any part and the joints between the base and it turn to follow the pointer. Gear ratios are reductions, so each joint's top speed is `SIM_MOTOR_SPEED` (constants.js, 4400 rpm = 26400 °/s) ÷ its total ratio. Nothing can be selected or added while simulating. The Reachable Workspace (red) and Dexterous Workspace (green) buttons under the speed table show where the end effectors can reach, and where they can reach while pointing (output z axis) in at least 24 of 26 directions. Both come from random sampling over the ±180° joint range and ignore collisions.
- CAD tab: drawn from the simplified STL shells in `models/` (`CAD_FILES` in viewer.js), which are small and quick to read: `motor.stl` (motor with its built-in first gearbox; origin at the centre of the input face, output along +x), `gearbox.stl` (one add-on stage; origin at its input face, shaft along +y; one shape for every gearbox type), and the link connectors `input.stl` and `output.stl`. Downloads use the full STEP files instead (`STEP_FILES`: `<family>_gearbox.step`, `<family>_extension.step`, `input.step`, `output.step`), which must share the STL files' frames. All are Fusion 360 exports (Y-up; STL in mm, STEP in inches), turned Z-up (`CAD_UP` in viewer.js). Each is read once when first needed; a placeholder shape is drawn until then. Every module is drawn in its family colour; the motor file's built-in gearbox (in front of `BUILT_IN_GEARBOX_START` in viewer.js) takes its type's colour. Links are their filled parts (js/linkfill.js), or just their connectors until those are ready. The base isn't drawn.
- Export STEP (File menu): the whole robot at its current pose as one STEP assembly (AP214, inches, Y-up, like the Fusion files), so it opens in Fusion 360 or any CAD tool as separate, positioned parts rather than one mesh. Each module file is included once and placed as an instance: one sub-assembly per actuator (its motor and extension stages), plus one solid per filled link part. See js/stepexport.js. A window shows progress and notes the design may still have small mechanical bugs. A single link part can also be exported from its inspector (Export link STEP).
- Export URDF (File menu): a .zip with robot.urdf and meshes/ built from the simplified STL shells and the filled link parts (OBJ meshes, mm). Each actuator is a revolute joint at its output frame (all joints at 0°), ±180° limits, velocity from the gear ratio, effort = 0.7 N·m motor torque × gear ratio. Inertial data (mass, centre of mass and inertia per body) assumes one uniform density (`DENSITY`, 1.24 g/cm³): actuator modules use values measured from their full models (`MODULES`), link parts are measured from their solids. See js/urdf.js.
- Units: stored in mm and degrees; the mm / in toggle only changes what is shown and typed. World z is up.
#   r o b o t _ b u i l d e r 
 
 