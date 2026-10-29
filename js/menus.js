// Top bar menus (File, ?): a button that opens a list of items. A click outside, Esc or picking an
// item closes it; the items themselves are ordinary buttons with their own handlers.
const MENUS = [['btn-menu-file', 'menu-file'], ['btn-menu-help', 'menu-help']];

export function initMenus() {
  const menus = MENUS.map(([b, l]) => ({ btn: document.getElementById(b), list: document.getElementById(l) }));
  const items = (m) => [...m.list.querySelectorAll('[role^="menuitem"]')];
  const open = (m, on) => { m.list.hidden = !on; m.btn.setAttribute('aria-expanded', String(on)); };
  const closeAll = () => menus.forEach((m) => open(m, false));

  for (const m of menus) {
    m.btn.addEventListener('click', (e) => {
      const on = m.list.hidden;
      closeAll();
      open(m, on);
      if (on && e.detail === 0) items(m)[0]?.focus(); // opened from the keyboard: start on the first item
    });
    // Picking an item closes the menu (after the item's own click handler has run).
    m.list.addEventListener('click', (e) => { if (e.target.closest('[role^="menuitem"]')) open(m, false); });
    m.list.addEventListener('keydown', (e) => {
      const all = items(m), i = all.indexOf(document.activeElement);
      if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
        e.preventDefault();
        all[(i + (e.key === 'ArrowDown' ? 1 : all.length - 1)) % all.length]?.focus();
      }
    });
  }
  document.addEventListener('click', (e) => { if (!e.target.closest('.menu')) closeAll(); });
  document.addEventListener('keydown', (e) => {
    const m = menus.find((x) => !x.list.hidden);
    if (e.key === 'Escape' && m) { open(m, false); m.btn.focus(); e.stopImmediatePropagation(); }
  });
}
