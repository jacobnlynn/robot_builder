// Component library for the modular actuators and links.
// Units: millimetres and degrees everywhere.
//
// Fixed transforms are converted to 4x4 matrices at runtime (see kinematics.js).
// Motor and gearbox offsets are the real module dimensions; radii are still
// placeholders for the CAD-mode cylinders.

// 'standard' = Rz(θ) Tz(d) Tx(a) Rx(α)   'modified' (Craig) = Rx(α) Tx(a) Rz(θ) Tz(d)
export const DH_CONVENTION = 'standard';

export const MM_PER_IN = 25.4;

// Motor: the input face (bolted to the parent port) points up its z axis, and the
// output shaft is turned 90° from it, pointing along the input's +x axis.
// The housing is `length` long along the output axis, centred on the input, and holds
// the first gearbox. From the input frame, the output face (with that first gearbox)
// is `rise` up z and `reach` out x.
export const MOTORS = {
  m1: { label: 'Motor M1', rise: 2.225 * MM_PER_IN, reach: 1.472 * MM_PER_IN, length: 3 * MM_PER_IN, radius: 30 },
};

// Gearbox: the first one sits inside the motor housing; each one after it bolts onto
// the output face and extends it `length` further along the output axis. They stack
// in the order chosen; every actuator has at least one.
export const GEARBOX_LENGTH = 1.6 * MM_PER_IN;
export const GEARBOXES = {
  planetary_3:   { family: 'planetary', label: 'Planetary 3:1',  ratio: [3, 1],  length: GEARBOX_LENGTH, radius: 30 },
  planetary_5:   { family: 'planetary', label: 'Planetary 5:1',  ratio: [5, 1],  length: GEARBOX_LENGTH, radius: 30 },
  planetary_9:   { family: 'planetary', label: 'Planetary 9:1',  ratio: [9, 1],  length: GEARBOX_LENGTH, radius: 30 },
  cycloidal_18:  { family: 'cycloidal', label: 'Cycloidal 18:1', ratio: [18, 1], length: GEARBOX_LENGTH, radius: 34 },
  harmonic_45_2: { family: 'harmonic',  label: 'Harmonic 22.5:1', ratio: [45, 2], length: GEARBOX_LENGTH, radius: 32 },
};
export const MIN_GEARBOXES = 1;

export const FAMILY_LABELS = { planetary: 'Planetary', cycloidal: 'Cycloidal', harmonic: 'Harmonic' };

// Colour coding shared by the 3D view and the UI.
export const FAMILY_COLORS = {
  motor: 0x5b6573,
  planetary: 0x4f86c6,
  cycloidal: 0xd08a2e,
  harmonic: 0x3f9e7d,
  link: 0xb7c1cc,
};

export const MAX_GEARBOXES = 3;
export const JOINT_LIMIT = 180; // ± degrees on the joint sliders

// Simulate mode: motor shaft speed in degrees per second (4400 rpm × 6 = 26400 °/s). Gearboxes are
// reductions, so a joint's top speed is this divided by its total ratio (18:1 → 1467 °/s).
export const SIM_MOTOR_SPEED = 4400 * 6;

export const OUTPUT_FLANGE = { thickness: 6, inset: 4, color: 0x8b95a2 };
export const LINK = { radius: 13, flangeRadius: 26, flangeThickness: 6 };
// Passive joints between two links (a link output that is a ball or hinge joint): drawn much
// smaller than the motor connectors. A hinge turns about the output's z axis.
export const PASSIVE = { color: 0xd9a21b, ballRadius: 16, hingeRadius: 8, hingeLength: 40, limit: 180 };
export const DEFAULT_LINK_DH = { a: 0, alpha: 0, d: 80, theta: 0 };
