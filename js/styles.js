// Themes (in the Tutorial & theme menu): switch the whole site between visual themes. A theme is a stylesheet loaded after
// style.css (it only overrides it). The 3D view keeps the same colours in every theme (Retro's, the
// site's defaults); a theme may still give its own with a `colors` entry. Retro is the default;
// the choice is remembered in this browser. Pro layout switches to the main, pro page (index.html),
// taking the robot along (store.handOff).
import * as store from './robot.js';
import { FAMILY_COLORS, OUTPUT_FLANGE } from './constants.js';

export const THEMES = {
  retro: { label: 'Retro', css: 'css/theme-retro.css' },
  pro: { label: 'Pro layout', note: 'Switch to the pro version of the site', href: 'index.html' },
  risograph: { label: 'Risograph', css: 'css/theme-risograph.css' },
  pixel: { label: 'Pixel', css: 'css/theme-pixel.css' },
  cga: { label: 'CGA', css: 'css/theme-cga.css' },
};
export const DEFAULT_THEME = 'retro';
const KEY = 'mrb.style';

// The site's own 3D colours, restored before a theme applies its own.
const DEFAULTS = { family: { ...FAMILY_COLORS }, flange: OUTPUT_FLANGE.color };

let current = DEFAULT_THEME;

export function applyTheme(key, viewer) {
  // A theme that is another page (Normal) is never applied here; fall back to the default.
  const ok = Boolean(THEMES[key]?.css);
  const theme = ok ? THEMES[key] : THEMES[DEFAULT_THEME];
  current = ok ? key : DEFAULT_THEME;
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
      <span class="style-name">${t.label}</span>${t.note ? `<span class="style-note">${t.note}</span>` : ''}</button>`).join('');
  list.addEventListener('click', (e) => {
    const item = e.target.closest('[data-style]');
    if (!item || item.dataset.style === current) return;
    const t = THEMES[item.dataset.style];
    if (t.href) {
      store.handOff();
      try { localStorage.setItem('mrb.layout', 'pro'); } catch { /* storage blocked */ }
      // Another site (the online previews) opens in a new tab; the robot can't travel there.
      if (/^https?:/.test(t.href)) window.open(t.href, '_blank', 'noopener');
      else location.href = t.href;
      return;
    }
    applyTheme(item.dataset.style, viewer);
  });

  let saved = null;
  try { saved = localStorage.getItem(KEY); } catch { /* storage blocked: use the default */ }
  applyTheme(saved && THEMES[saved]?.css ? saved : DEFAULT_THEME, viewer);
}
