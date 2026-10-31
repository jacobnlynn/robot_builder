// Example: six-legged hexapod. A hexagonal mounting base at z = 0 with one leg on top of each
// corner, every leg facing straight out from the centre. Each leg is the robot dog's left leg
// (hip, upper and lower actuator joined by two links), rotated into place: the thigh rises
// 15° above horizontal and the shin comes down so the foot ends just below z = 0.

const ROD = { lon: 0, lat: 90 };
const zeroDh = () => ({ a: 0, alpha: 0, d: 0, theta: 0 });
const pose = (p) => ({ x: 0, y: 0, z: 0, alpha: 0, beta: 0, gamma: 0, ref: 'input', ...p });
// A link with nothing after it (a shin) ends the leg in a 20 mm foot sphere.
const FOOT = { end: 'sphere', sphere: 20 };
const link = (id, name, parent, dh, child) => ({
  id, type: 'link', name, spec: 'dh', parent,
  outputs: [{ dh, pose: pose({ z: 80 }), rod: { ...ROD }, ...(child ? {} : FOOT) }],
  children: [child],
});
const actuator = (id, name, parent, mount, q, child) => ({
  id, type: 'actuator', name, motor: 'm1', gearboxes: ['cycloidal_18'], mount, q, parent, children: [child],
});

// The dog's left leg from the hip on, with its own joint angles (solved from the geometry):
// upper -75° tilts the thigh 15° above horizontal; lower -67.2° points the shin down so the
// foot ends 10 mm below z = 0.
const UPPER_Q = -75;
const LOWER_Q = -67.2;
// Thigh 6 in and shin 8 in (the dog's are 5.5 in and 7 in).
const FEMUR = { a: 152.4, alpha: -90, d: 0, theta: -90 };
const TIBIA = { a: 203.2, alpha: 0, d: 0, theta: 90 };

// Hip mounts sit on a hexagon of this radius, facing up so the legs stand on top of the body.
// γ turns each one so its foot lands straight out from the centre: with these joint angles
// the foot is 102.0° off the port's outward axis, so turn it back by that much.
const RADIUS = 210;
const LEG_TWIST = 102.0;

const LEGS = ['front', 'front_left', 'back_left', 'back', 'back_right', 'front_right']
  .map((key, i) => ({ key, angle: i * 60 }));

const nodes = [
  {
    id: 'base', type: 'link', name: 'Base', fixed: true, spec: 'dh', parent: null,
    outputs: [{ dh: zeroDh(), pose: pose({}), rod: { ...ROD } }],
    children: ['body'],
  },
  {
    id: 'body', type: 'link', name: 'Mounting Base', spec: 'pose', parent: { nodeId: 'base', index: 0 },
    outputs: LEGS.map((l) => ({
      dh: zeroDh(),
      pose: pose({
        x: +(RADIUS * Math.cos(l.angle * Math.PI / 180)).toFixed(3),
        y: +(RADIUS * Math.sin(l.angle * Math.PI / 180)).toFixed(3),
        gamma: +(l.angle - 90 + LEG_TWIST).toFixed(3),
      }),
      rod: { ...ROD },
    })),
    children: LEGS.map((l) => `${l.key}_hip`),
  },
];

LEGS.forEach((l, i) => {
  const id = (part) => `${l.key}_${part}`;
  nodes.push(
    actuator(id('hip'), `${l.key}_hip`, { nodeId: 'body', index: i }, 0, 0, id('upper')),
    actuator(id('upper'), `${l.key}_upper`, { nodeId: id('hip'), index: 0 }, -90, UPPER_Q, id('femur')),
    link(id('femur'), `${l.key}_femur`, { nodeId: id('upper'), index: 0 }, { ...FEMUR }, id('lower')),
    actuator(id('lower'), `${l.key}_lower`, { nodeId: id('femur'), index: 0 }, -90, LOWER_Q, id('tibia')),
    link(id('tibia'), `${l.key}_tibia`, { nodeId: id('lower'), index: 0 }, { ...TIBIA }, null),
  );
});

// Saved poses (the Poses list above the joint sliders): [hip, upper, lower] for each leg.
// Tall and Wide push the feet further down or out; Tripod step lifts the front, back left and
// back right legs and swings them forward while the other three push back; Wave raises the front leg.
const TALL = [0, -95, -55];
const TRIPOD = new Set(['front', 'back_left', 'back_right']);
const legPose = (name, angles) => ({
  name,
  q: Object.fromEntries(LEGS.flatMap(({ key }) => {
    const [hip, upper, lower] = angles(key);
    return [[`${key}_hip`, hip], [`${key}_upper`, upper], [`${key}_lower`, lower]];
  })),
});
// Walk: a tripod gait in four poses (front, back left and back right together, then the other three),
// worked out by inverse kinematics so feet on the ground stay level and move back while the other
// tripod lifts and swings forward.
const WALK = [
  { front: [-6.5, -88, -44.5], front_left: [40, -62.5, -76], back_left: [-21.5, -80, -74.5], back: [-6.5, -88, -44.5], back_right: [40, -62.5, -76], front_right: [-21.5, -80, -74.5] },
  { front: [0, -75, -67], front_left: [-1.5, -64, -70], back_left: [0, -75, -67], back: [-1.5, -64, -70], back_right: [0, -75, -67], front_right: [-1.5, -64, -70] },
  { front: [7.5, -64, -86], front_left: [-26, -90.5, -54], back_left: [32, -74, -55.5], back: [7.5, -64, -86], back_right: [-26, -90.5, -54], front_right: [32, -74, -55.5] },
  { front: [-1.5, -64, -70], front_left: [0, -75, -67], back_left: [-1.5, -64, -70], back: [0, -75, -67], back_right: [-1.5, -64, -70], front_right: [0, -75, -67] },
];
const poses = [
  legPose('Stand', () => [0, UPPER_Q, LOWER_Q]),
  legPose('Tall', () => TALL),
  legPose('Wide', () => [0, -100, -35]),
  legPose('Tripod step', (key) => (TRIPOD.has(key) ? [15, -55, -75] : [-15, -100, -52])),
  legPose('Wave', (key) => (key === 'front' ? [0, -15, -65] : [0, UPPER_Q, LOWER_Q])),
  ...WALK.map((w, i) => legPose(`Walk ${i + 1}`, (key) => w[key])),
];

export const HEXAPOD = {
  format: 'modular-robot',
  version: 1,
  convention: 'standard',
  units: { length: 'mm', angle: 'deg' },
  name: 'hexapod',
  rootId: 'base',
  nodes,
  poses,
  // Animations (the Animate view): steps name poses; hold is in seconds, speed in % of top speed.
  animations: [
    { name: 'Walk', loop: true, speed: 100, pause: false, walk: true, steps: [1, 2, 3, 4].map((i) => ({ pose: `Walk ${i}`, hold: 0 })) },
    { name: 'Wave', loop: true, speed: 50, pause: true, steps: [{ pose: 'Stand', hold: 0.3 }, { pose: 'Wave', hold: 0.6 }] },
    { name: 'Stretch', loop: true, speed: 25, pause: true, steps: [{ pose: 'Stand', hold: 0.3 }, { pose: 'Tall', hold: 0.3 }, { pose: 'Wide', hold: 0.3 }] },
  ],
};
