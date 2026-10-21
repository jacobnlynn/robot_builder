// Guided tour: a small window that walks through the site, with an arrow pointing at the button
// or panel it is talking about. It opens every time the site opens, and from the Tutorial button. Every step sets up everything it needs from scratch, so Back always works,
// including through the steps that build the example arm piece by piece.
import * as THREE from 'three';
import * as store from './robot.js';
import { state, defaultOutput } from './robot.js';
import { EXAMPLES } from './examples.js';
import { MOTORS } from './constants.js';
import { matrixToPose, actuatorReach } from './kinematics.js';

const $ = (s) => document.querySelector(s);

// ---- The example arm the tour builds: shoulder → upper arm → elbow → forearm with a ball end ----

const ARM = {
  shoulder: { name: 'Shoulder', gearboxes: ['cycloidal_18', 'cycloidal_18'] },
  elbow: { name: 'Elbow', gearboxes: ['planetary_9'], out: [110, 0, 300] }, // output centre, axis along +x
  hand: { at: [150, 0, 520], sphere: 25 },
};

/** World frame of a motor's mounting face, given where its output is (shaft along +x, standing up +z). */
function elbowFace() {
  const reach = actuatorReach({ motor: 'm1', gearboxes: ARM.elbow.gearboxes });
  const [x, y, z] = ARM.elbow.out;
  return new THREE.Matrix4().setPosition(x - reach, y, z - MOTORS.m1.rise);
}

const worldOutput = (m, extra = {}) => ({ ...defaultOutput(), pose: { ...matrixToPose(m), ref: 'world' }, ...extra });
const last = (type) => Object.values(state.robot.nodes).filter((n) => n.type === type).at(-1);

/** A fresh robot with the first `pieces` parts of the example arm (0–4). */
function buildArm(pieces) {
  store.newRobot('My first arm');
  if (pieces >= 1) store.addActuator({ nodeId: 'base', index: 0 }, { name: ARM.shoulder.name, motor: 'm1', gearboxes: ARM.shoulder.gearboxes });
  if (pieces >= 2) store.addLink({ nodeId: last('actuator').id, index: 0 }, { name: 'Upper arm', spec: 'pose', outputs: [worldOutput(elbowFace())] });
  if (pieces >= 3) store.addActuator({ nodeId: last('link').id, index: 0 }, { name: ARM.elbow.name, motor: 'm1', gearboxes: ARM.elbow.gearboxes });
  if (pieces >= 4) {
    store.addLink({ nodeId: last('actuator').id, index: 0 }, {
      name: 'Forearm', spec: 'pose',
      outputs: [worldOutput(new THREE.Matrix4().setPosition(...ARM.hand.at), { end: 'sphere', sphere: ARM.hand.sphere })],
    });
  }
}

// ---- Steps ----
// target: CSS selector (or function) of what the arrow points at, or null for a centred window.
// setup(ctx): puts the site in the state the step talks about.

const exampleDog = ({ viewer }) => {
  if (state.name !== EXAMPLES[0].data.name || Object.keys(state.robot.nodes).length < 5) { store.load(EXAMPLES[0].data); viewer.fit(); }
  store.select(null);
};
const view = (v) => (ctx) => { exampleDog(ctx); store.setView(v); };
const armStep = (pieces, select) => ({ viewer }) => {
  store.setView('cad');
  buildArm(pieces);
  for (const n of Object.values(state.robot.nodes)) if (n.type === 'actuator') n.q = 0;
  if (select) store.select(select());
  else store.select(null);
  store.emit('structure');
  viewer.fit();
};

const STEPS = [
  {
    title: 'Welcome',
    target: null,
    text: `<p>This is the Modular Robot Builder. You build a robot from two kinds of parts:
      actuators (a motor with a stack of gearboxes) and links (the rigid pieces between them).</p>
      <p>We've loaded the Robot Dog example. Click Next for a quick tour, or Skip to start building.</p>`,
    setup: (ctx) => { exampleDog(ctx); store.setView('cad'); },
  },
  {
    title: 'Examples',
    target: '#examples',
    text: '<p>Load a finished robot here any time to explore it: a dog, a hexapod, a 7-joint arm or a humanoid.</p>',
    setup: view('cad'),
  },
  {
    title: 'Model view',
    target: '[data-view="cad"]',
    text: `<p>Model is the everyday view: actuators as colour-coded cylinders (one colour per gearbox type)
      and links as rods. Drag to orbit, right-drag to pan, scroll to zoom.</p>`,
    setup: view('cad'),
  },
  {
    title: 'CAD view',
    target: '[data-view="cadfile"]',
    text: `<p>CAD shows the real designed shells of the motors and gearboxes, and fills each link in as a
      printable part. Switch the tubes between round and square with the Tubes toggle.</p>`,
    setup: view('cadfile'),
  },
  {
    title: 'Simplified view',
    target: '[data-view="simple"]',
    text: '<p>Simplified strips it down to the kinematics: a cylinder for each joint axis and thin lines for the links.</p>',
    setup: view('simple'),
  },
  {
    title: 'Simulate',
    target: '[data-view="sim"]',
    text: `<p>Simulate lets you grab any part and drag it: the joints follow, each no faster than its gearing allows.
      The side panel also shows the robot's reachable and dexterous workspaces.</p>`,
    setup: view('sim'),
  },
  {
    title: 'Start a new robot',
    target: '#btn-new',
    text: `<p>Now let's build one. New starts an empty robot: just the base, a locked link at the world origin.
      Its blue dot is an open port, where the next part attaches.</p>
      <p>Over the next four steps we'll build a simple arm, one piece at a time.</p>`,
    setup: armStep(0, () => ({ kind: 'port', nodeId: 'base', index: 0 })),
  },
  {
    title: '1 · Motors',
    target: '#inspector',
    text: `<p>We added a Shoulder actuator to the base. Every actuator is a right-angle motor: it bolts on with
      its input facing up and turns its output 90° to the side.</p>
      <p>Here you pick its gearbox stack (each one multiplies torque and divides speed), its mount angle
      on the port, and its joint angle. The i button shows the motor's offsets.</p>`,
    setup: armStep(1, () => ({ kind: 'node', nodeId: last('actuator').id })),
  },
  {
    title: '2 · Links',
    target: '#inspector',
    text: `<p>Next, an Upper arm link on the shoulder's output. A link is a rigid part: its outputs are placed
      with D-H parameters or XYZ + angles, measured from its port, the world or any other part.</p>
      <p>A link can have several outputs, and each one can be another port or a sphere end (a foot or a hand).</p>`,
    setup: armStep(2, () => ({ kind: 'node', nodeId: last('link').id })),
  },
  {
    title: '3 · Assembly',
    target: '.panel-left .panel-section:first-child',
    text: `<p>Then an Elbow actuator and a Forearm link ending in a sphere. The Assembly list shows the
      whole robot as a tree: click any part to edit it, or an open port to attach something.</p>
      <p>Use the − / + buttons to fold branches, or Tree view for a diagram.</p>`,
    setup: armStep(4, null),
  },
  {
    title: '4 · Joints',
    target: '.panel-left .panel-section:nth-child(2)',
    text: `<p>That's a two-joint arm. The Joints panel has a slider for every actuator. We've moved the shoulder
      and elbow so you can see it bend. Zero all puts every joint back to 0°.</p>`,
    setup: (ctx) => {
      armStep(4, null)(ctx);
      const [shoulder, elbow] = Object.values(state.robot.nodes).filter((n) => n.type === 'actuator');
      shoulder.q = 35;
      elbow.q = -60;
      store.emit('structure');
    },
  },
  {
    title: 'Robot data',
    target: '#data-body .data-title',
    // Opens the real window. It is modal, so the tour window moves inside it to stay clickable.
    opens: { button: '#btn-data', dialog: '#data-dialog' },
    text: `<p>This is the Robot data window (the Robot data button, top right). It holds the numbers behind your robot,
      updated live as joints move: the Jacobian of every end effector (unfold "Formula" for it at any joint angles, with
      Python to copy), its rank, manipulability and singular values, a table of every joint, the end-effector transforms,
      and the transform from each motor to the next.</p>`,
    setup: (ctx) => { if (!last('link') || last('link').name !== 'Forearm') armStep(4, null)(ctx); },
  },
  {
    title: 'Export STL',
    target: '#btn-export-stl',
    text: `<p>Export STL downloads the whole robot as one file: every actuator as its full model with all of its
      interior parts, and every link filled in as a printable part.</p>`,
    setup: (ctx) => { if (!last('link') || last('link').name !== 'Forearm') armStep(4, null)(ctx); store.setView('cadfile'); },
  },
  {
    title: 'From file to robot',
    target: '#btn-export-stl',
    text: `<p>That STL is meant to be a complete robot model: print and assemble the parts, and all that is left is to
      wire the motors and program it.</p>
      <p>(The full models may still have small mechanical bugs while the design is being refined.)</p>`,
    setup: (ctx) => { if (!last('link') || last('link').name !== 'Forearm') armStep(4, null)(ctx); store.setView('cadfile'); },
  },
  {
    title: 'Export URDF',
    target: '#btn-export-urdf',
    text: `<p>Export URDF saves your robot for simulation: a robot.urdf with its joints, limits, torques and masses,
      plus the meshes. Load it in MuJoCo (or Gazebo, PyBullet, Isaac Sim) to test it before you build it.</p>
      <p>That's the tour. You can replay it any time from the Tutorial button. Happy building!</p>`,
    setup: (ctx) => { if (!last('link') || last('link').name !== 'Forearm') armStep(4, null)(ctx); store.setView('cad'); },
  },
];

// ---- The window ----

export function initTutorial(viewer) {
  const ctx = { viewer };
  const box = document.createElement('div');
  box.className = 'tour';
  box.setAttribute('role', 'dialog');
  box.setAttribute('aria-modal', 'false');
  box.setAttribute('aria-labelledby', 'tour-title');
  box.hidden = true;
  box.innerHTML = `
    <div class="tour-arrow" aria-hidden="true"></div>
    <div class="tour-head"><h2 id="tour-title"></h2><span class="tour-count"></span>
      <button type="button" class="icon-btn" data-tour="skip" aria-label="Close the tutorial">&times;</button></div>
    <div class="tour-body"></div>
    <div class="tour-foot">
      <button type="button" class="btn btn-ghost btn-small" data-tour="skip">Skip tutorial</button>
      <span class="tour-spacer"></span>
      <button type="button" class="btn btn-small" data-tour="back">Back</button>
      <button type="button" class="btn btn-primary btn-small" data-tour="next">Next</button>
    </div>`;
  document.body.append(box);
  const arrow = box.querySelector('.tour-arrow');
  let index = -1, target = null, openDialog = null;

  /** Close a window the previous step opened, and bring the tour window back out of it. */
  function leaveStep() {
    if (openDialog) {
      document.body.append(box);
      if (openDialog.open) openDialog.close();
      openDialog = null;
    }
  }

  const targetOf = (s) => (typeof s.target === 'function' ? s.target() : s.target && $(s.target));

  function place() {
    if (box.hidden) return;
    target?.classList.remove('tour-target');
    target = targetOf(STEPS[index]);
    const vw = window.innerWidth, vh = window.innerHeight, gap = 14, pad = 10;
    const w = box.offsetWidth, h = box.offsetHeight;
    if (!target) {
      arrow.hidden = true;
      box.style.left = `${Math.max(pad, (vw - w) / 2)}px`;
      box.style.top = `${Math.max(pad, (vh - h) / 2)}px`;
      return;
    }
    target.classList.add('tour-target');
    const r = target.getBoundingClientRect();
    // Below the target if it fits, else above, else beside it (big panels).
    let side = r.bottom + gap + h <= vh - pad ? 'below' : r.top - gap - h >= pad ? 'above' : r.right + gap + w <= vw - pad ? 'right' : 'left';
    let left, top;
    if (side === 'below' || side === 'above') {
      left = Math.min(Math.max(pad, r.left + r.width / 2 - w / 2), vw - w - pad);
      top = side === 'below' ? r.bottom + gap : r.top - gap - h;
    } else {
      top = Math.min(Math.max(pad, r.top + r.height / 2 - h / 2), vh - h - pad);
      left = side === 'right' ? r.right + gap : Math.max(pad, r.left - gap - w);
    }
    box.style.left = `${left}px`;
    box.style.top = `${top}px`;
    arrow.hidden = false;
    arrow.dataset.side = side;
    // Point the arrow at the middle of the target, kept on the window's edge.
    if (side === 'below' || side === 'above') {
      arrow.style.left = `${Math.min(Math.max(16, r.left + r.width / 2 - left), w - 16)}px`;
      arrow.style.top = '';
    } else {
      arrow.style.top = `${Math.min(Math.max(16, r.top + r.height / 2 - top), h - 16)}px`;
      arrow.style.left = '';
    }
  }

  function show(i) {
    index = Math.max(0, Math.min(STEPS.length - 1, i));
    const s = STEPS[index];
    leaveStep();
    box.hidden = false;
    try { s.setup?.(ctx); } catch (e) { console.warn('Tutorial step setup failed:', e); }
    if (s.opens) {
      openDialog = $(s.opens.dialog);
      if (!openDialog.open) $(s.opens.button).click();
      openDialog.append(box);
      // Closed with its own × or Esc: bring the tour window back out so it stays visible.
      openDialog.addEventListener('close', () => {
        if (box.parentElement !== document.body) { document.body.append(box); openDialog = null; place(); }
      }, { once: true });
    }
    box.querySelector('#tour-title').textContent = s.title;
    box.querySelector('.tour-count').textContent = `${index + 1} of ${STEPS.length}`;
    box.querySelector('.tour-body').innerHTML = s.text;
    box.querySelector('[data-tour="back"]').disabled = index === 0;
    box.querySelector('[data-tour="next"]').textContent = index === STEPS.length - 1 ? 'Finish' : 'Next';
    // Let the site re-render (and the inspector fill in) before measuring where to point.
    requestAnimationFrame(() => requestAnimationFrame(place));
    box.querySelector('[data-tour="next"]').focus({ preventScroll: true });
  }

  function close() {
    leaveStep();
    box.hidden = true;
    target?.classList.remove('tour-target');
    target = null;
  }

  box.addEventListener('click', (e) => {
    const act = e.target.closest('[data-tour]')?.dataset.tour;
    if (act === 'skip') close();
    else if (act === 'back') show(index - 1);
    else if (act === 'next') (index === STEPS.length - 1 ? close() : show(index + 1));
  });
  document.addEventListener('keydown', (e) => {
    if (box.hidden) return;
    if (e.key === 'Escape') { close(); e.stopImmediatePropagation(); }
  }, true);
  window.addEventListener('resize', place);
  new ResizeObserver(() => place()).observe(document.body);

  $('#btn-tour')?.addEventListener('click', () => show(0));

  show(0); // every time the site opens
  return { show, close };
}

