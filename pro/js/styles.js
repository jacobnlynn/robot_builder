// Theme choices (View > Theme): switch the whole site between visual themes. A theme is a stylesheet loaded after
// style.css (it only overrides it) plus, optionally, colours for the 3D view. Retro is the default;
// the choice is remembered in this browser. (Normal was called Light before; a saved 'light' still works.)
import * as store from './robot.js';
import { FAMILY_COLORS, OUTPUT_FLANGE } from './constants.js';

export const THEMES = {
  // The classic site's themes, in the pro layout. Their 3D view keeps the site's own colours, as in
  // the classic layout.
  retro: { label: 'Retro', css: 'pro/css/theme-retro.css' },
  normal: {
    label: 'Normal',
    css: 'pro/css/theme-light.css',
    colors: {
      scene: { background: 0xe9ecf0, select: 0xd9861a, seam: 0x353d47, simpleLine: 0x28313b, simpleJoint: 0x2f6fe4, grid: 0x000000 },
    },
  },
  dark: {
    label: 'Dark',
    css: 'pro/css/theme-dark.css',
    colors: {
      flange: 0x8d96a3,
      scene: { background: 0x2b2f36, select: 0xe39b3a, seam: 0x16191d, simpleLine: 0xc9ced6, simpleJoint: 0x8fb4e6, grid: 0xffffff },
    },
  },
  risograph: { label: 'Risograph', css: 'pro/css/theme-risograph.css' },
  pixel: { label: 'Pixel', css: 'pro/css/theme-pixel.css' },
  cga: { label: 'CGA', css: 'pro/css/theme-cga.css' },
};
export const DEFAULT_THEME = 'retro';
const KEY = 'mrb.pro-theme'; // (a new key, so everyone starts on the new default, Retro)

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

/** Fills the Theme group of the View menu; pro/js/menus.js opens and closes the menu. */
export function initStyles(viewer) {
  const list = document.getElementById('style-list');
  list.innerHTML = Object.entries(THEMES).map(([k, t]) => `<button type="button" class="menu-item" role="menuitemradio" data-style="${k}" aria-checked="false">
      <span>${t.label}</span></button>`).join('');
  list.addEventListener('click', (e) => {
    const item = e.target.closest('[data-style]');
    if (item && item.dataset.style !== current) applyTheme(item.dataset.style, viewer);
  });

  let saved = null;
  try { saved = localStorage.getItem(KEY); } catch { /* storage blocked: use the default */ }
  if (saved === 'light') saved = 'normal';
  applyTheme(saved && THEMES[saved] ? saved : DEFAULT_THEME, viewer);
}
