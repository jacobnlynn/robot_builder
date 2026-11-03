// Example: four-bar linkage, the classic crank-rocker. A motor turns the crank; a coupler joins the
// crank's tip to a rocker that swings about a fixed pivot on the frame. Every joint but the motor is
// a passive hinge, and the rocker's far end is joined to the frame's pivot in a closed loop, so the
// coupler and rocker angles follow from the crank angle. Bars: crank 100, coupler 300, rocker 250,
// ground 300 mm. The crank is the shortest bar (100 + 300 ≤ 250 + 300), so it turns all the way round.
// A ball on the coupler traces the coupler curve.
import { builder, animation, v3, UP, LEFT, at } from './build.js';

const { motor, link, robot } = builder();
const Y = 60; // the plane the bars move in (y = 60 mm); every hinge axis is along +y
const H = 300; // height of the two ground pivots
const A = v3(0, Y, H), D = v3(300, Y, H); // crank pivot (the motor's output) and rocker pivot
const B = v3(100, Y, H); // crank tip, with the crank pointing along +x
const C = v3(268.75, Y, H + 248.04); // coupler-rocker hinge: 300 from B and 250 from D
const P = v3(118.4, Y, H + 169.0); // coupler point, off to one side of the coupler

motor('crank_motor', 'crank_motor', at('frame'), ['cycloidal_18', 'cycloidal_18'], A, LEFT, UP, 'crank');
link('frame', 'frame', at('base'), ['crank_motor', { joint: 'hinge', at: D, axis: LEFT }]);
link('crank', 'crank', at('crank_motor'), [{ joint: 'hinge', at: B, axis: LEFT, child: 'coupler' }]);
link('coupler', 'coupler', at('crank', 0), [{ joint: 'hinge', at: C, axis: LEFT, child: 'rocker' }, { sphere: 15, at: P }]);
link('rocker', 'rocker', at('coupler', 0), [{ joint: 'hinge', at: D, axis: LEFT }]);

// Saved poses: the crank at quarter turns.
const crank = (deg) => ({ name: `Crank ${deg}°`, q: { crank_motor: deg } });
// Full turn is Crank 0° one whole turn on, so Turn the crank keeps going round instead of turning back.
export const FOURBAR = robot('Four-bar', 'frame', [crank(0), crank(90), crank(180), crank(270), { name: 'Full turn', q: { crank_motor: 360 } }],
  [[['rocker', 0], ['frame', 1]]], [
    // Animations (the Animate view).
    animation('Turn the crank', ['Crank 0°', 'Crank 90°', 'Crank 180°', 'Crank 270°', 'Full turn'], { pause: false, speed: 100 }),
    animation('Rock', ['Crank 90°', 'Crank 270°'], { speed: 100 }),
  ]);
