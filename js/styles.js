// Themes (in the Tutorial & theme menu): switch the whole site between visual themes. A theme is a stylesheet loaded after
// style.css (it only overrides it) plus, optionally, colours for the 3D view. Retro is the default;
// the choice is remembered in this browser.
import * as store from './robot.js';
import { FAMILY_COLORS, OUTPUT_FLANGE } from './constants.js';

export const THEMES = {
  retro: {
    label: 'Retro',
    css: 'css/theme-retro.css',
  },
  datasheet: {
    label: 'Normal', // the Datasheet theme
    css: 'css/theme-datasheet.css',
    colors: {
      family: { motor: 0x23272d, planetary: 0x0a4bd6, cycloidal: 0x7d8590, harmonic: 0x9db8f5, link: 0xdfe2e6 },
      flange: 0xb9bec6,
      scene: { background: 0xf4f5f7, select: 0x0a4bd6, seam: 0x111316, simpleLine: 0x111316, simpleJoint: 0x0a4bd6 },
    },
  },
  risograph: {
    label: 'Risograph',
    css: 'css/theme-risograph.css',
    colors: {
      family: { motor: 0x1f4fd8, planetary: 0x8fa8ef, cycloidal: 0xff4f9a, harmonic: 0x7a3db8, link: 0xf6bdd4 },
      flange: 0xc9d2f5,
      scene: { background: 0xf4f1ea, select: 0xff4f9a, seam: 0x1f4fd8, simpleLine: 0x1f4fd8, simpleJoint: 0xff4f9a },
    },
  },
  pixel: {
    label: 'Pixel',
    css: 'css/theme-pixel.css',
    // The 3D view renders as usual, on the theme's navy, with a mustard selection.
    colors: {
      scene: { background: 0x1d2b53, select: 0xe8c547, seam: 0x1d2b53, simpleLine: 0xd4d7de, simpleJoint: 0x29adff },
    },
  },  cga: {
    label: 'CGA',
    css: 'css/theme-cga.css',
    // The 3D view renders as usual, on black, with a magenta selection.
    colors: {
      scene: { background: 0x000000, select: 0xff55ff, seam: 0x000000, simpleLine: 0x55ffff, simpleJoint: 0xff55ff },
    },
  },
};
export const DEFAULT_THEME = 'retro';
const KEY = 'mrb.style';

// The site's own 3D colours, restored before a theme applies its own.
const DEFAULTS = { family: { ...FAMILY_COLORS }, flange: OUTPUT_FLANGE.color };

let current = DEFAULT_THEME;

export function applyTheme(key, viewer) {
  const theme = THEMES[key] ?? THEMES[DEFAULT_THEME];
  current = THEMES[key] ? key : DEFAULT_THEME;
  document.getElementById('theme-css').href = theme.css;
  Object.assign(FAMILY_COLORS, DEFAULTS.family, theme.colors?.family);
  OUTPUT_FLANGE.color = theme.colors?.flange ?? DEFAULTS.flange;
  viewer.setSceneColors(theme.colors?.scene ?? {});
  if (store.state.robot) {
    viewer.redraw();
    store.emit('view'); // legend swatches and tree chips use the family colours
  }
  try { localStorage.setItem(KEY, current); } catch { /* storage blocked: just not remembered */ }
  renderMenu();
}

function renderMenu() {
  for (const item of document.querySelectorAll('[data-style]')) item.setAttribute('aria-checked', String(item.dataset.style === current));
}

/** Fills the Theme group of the Tutorial & theme menu; js/menus.js opens and closes the menu. */
export function initStyles(viewer) {
  const list = document.getElementById('style-list');
  list.innerHTML = Object.entries(THEMES).map(([k, t]) => `<button type="button" class="style-option" role="menuitemradio" data-style="${k}" aria-checked="false">
      <span class="style-name">${t.label}</span></button>`).join('');
  list.addEventListener('click', (e) => {
    const item = e.target.closest('[data-style]');
    if (item && item.dataset.style !== current) applyTheme(item.dataset.style, viewer);
  });

  let saved = null;
  try { saved = localStorage.getItem(KEY); } catch { /* storage blocked: use the default */ }
  applyTheme(saved && THEMES[saved] ? saved : DEFAULT_THEME, viewer);
}
