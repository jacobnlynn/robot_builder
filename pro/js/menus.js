// Menu bar menus (Project, Edit, View, Robot, Export, Help): each .menu holds a button and a list of
// items. Click (or Enter) opens one; while one is open, pointing at another menu's button opens that
// one instead, like a desktop menu bar. A click outside, Esc or picking an item closes it. The items
// are ordinary buttons with their own handlers (pro/js/chrome.js runs the data-cmd ones).
export function initMenus() {
  const menus = [...document.querySelectorAll('.menu')].map((el) => ({ btn: el.querySelector('[aria-haspopup="menu"]'), list: el.querySelector('.menu-list') }));
  const items = (m) => [...m.list.querySelectorAll('[role^="menuitem"]')];
  const open = (m, on) => { m.list.hidden = !on; m.btn.setAttribute('aria-expanded', String(on)); };
  const closeAll = () => menus.forEach((m) => open(m, false));
  const current = () => menus.find((x) => !x.list.hidden);

  menus.forEach((m, k) => {
    m.btn.addEventListener('click', (e) => {
      const on = m.list.hidden;
      closeAll();
      open(m, on);
      if (on && e.detail === 0) items(m)[0]?.focus(); // opened from the keyboard: start on the first item
    });
    m.btn.addEventListener('pointerenter', () => {
      const c = current();
      if (c && c !== m) { closeAll(); open(m, true); }
    });
    m.list.addEventListener('click', (e) => { if (e.target.closest('[role^="menuitem"]')) open(m, false); });
    m.list.addEventListener('keydown', (e) => {
      const all = items(m), i = all.indexOf(document.activeElement);
      if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
        e.preventDefault();
        all[(i + (e.key === 'ArrowDown' ? 1 : all.length - 1)) % all.length]?.focus();
      } else if (e.key === 'ArrowRight' || e.key === 'ArrowLeft') {
        e.preventDefault();
        const next = menus[(k + (e.key === 'ArrowRight' ? 1 : menus.length - 1)) % menus.length];
        closeAll();
        open(next, true);
        items(next)[0]?.focus();
      }
    });
  });
  document.addEventListener('click', (e) => { if (!e.target.closest('.menu')) closeAll(); });
  document.addEventListener('keydown', (e) => {
    const m = current();
    if (e.key === 'Escape' && m) { open(m, false); m.btn.focus(); e.stopImmediatePropagation(); }
  });
}
