# Robot Builder Pro

The builder's main layout, a desktop workstation: a menu bar, a ribbon of tools, three docks (assembly, 3D view, properties; the timeline shows under the 3D view in Animate) and a status bar, in any of the themes (View > Theme: Retro by default, Normal, Dark, Risograph, Pixel, CGA). The page is `index.html` at the site root; the classic layout is `classic.html`, and Help > Open the classic layout (or Pro layout in the classic theme menu) switches between them, taking the robot along. `pro/index.html` only forwards to `index.html`.

Run the site as usual (`python -m http.server 8000` in the folder above) and open http://localhost:8000/ (or http://localhost:8000/classic.html for the classic layout).

## How it is put together

`index.html` lives at the site root, so the pro version uses the root's `img/examples/`, `models/` and `vendor/` instead of copies. Its code is a copy of `js/` in `pro/js/`, so the two versions can change separately. The page keeps the element ids the shared code looks for (`#tree`, `#inspector`, `#status`, `#btn-new` and so on); only where they sit has changed.

What differs from `js/`:

| File | What changed |
| --- | --- |
| `chrome.js` (new) | Commands behind the menu bar, ribbon and keys; the view cube and axes that turn with the camera; the label on the selected part; the tags over the view; the ground grid; screenshots; the dock tabs and the Loops list; the unsaved-changes dot |
| `timeline.js` (new) | The timeline under the 3D view: the current animation's joint curves, keyframes and playhead, with transport, speed and easing controls; click or drag to scrub |
| `history.js` (new) | Undo and redo (the whole project as JSON after each change settles, up to 100 steps), and whether the project has changed since it was opened or saved |
| `animate.js` | Returns an API (play, pause, seek, select, set) that the timeline and the Walk command use, and tells listeners when the animation changes |
| `viewer.js` | Ground grid with a shadow-catching plane (`setGround`), grid colour per theme, `screenshot()` |
| `robot.js` | `state.fresh` marks a robot that was just opened, so undo starts a new history |
| `menus.js` | Every menu on the bar; while one is open, pointing at another opens it; arrow keys move between items and menus |
| `styles.js` | The Dark and Light themes (View > Theme) |
| `ui.js` | The New button keeps its icon while it asks "Discard?" |
| `tutorial.js` | Points at the pro layout's buttons and mentions the new tools |
| `linkfill.js` | Finds `vendor/` one folder further up |

`css/style.css` is a copy of the shared stylesheet; `css/pro.css` lays out the workstation over it, and `css/theme-dark.css` / `css/theme-light.css` hold the colours.

## New in the pro layout

- Menu bar: Project, Edit, View, Robot, Export and Help, each item with its shortcut. The document tab holds the project name, with a dot when there are unsaved changes.
- Ribbon: Project (Examples, New, Undo, Redo), Build (Attach: jump to the next open port; Tree; Delete), Display (Build, CAD, Simplified, Frames, Grid), Motion (Simulate, Animate, Walk), Analyze (Workspace, Robot data), Export (STEP, URDF).
- Undo and redo for every change, joint moves included.
- View cube: a real cube that turns with the camera; click a face to look from that side. Axes in the corner turn with it.
- Camera bar: fit, isometric, top, front, right, and a PNG screenshot of the view.
- Label on the selected part in the 3D view: its name and joint angle and ratio (or what kind of part or port it is).
- Tags over the view: projection, display mode, and the saved pose the joints are at (or the animation playing).
- Ground grid at the robot's lowest point, catching its shadow.
- Timeline: every joint the current animation moves as a curve over time, with a diamond at each pose and a playhead; scrub by dragging. Play, stop, loop, speed and easing from its header.
- Left dock tabs: Assembly, Poses (with a count) and Loops (every closed loop, with links to its two ends and Unjoin).
- Keyboard: 1 to 5 switch displays, 0 / 7 / 8 / 9 snap the camera, G grid, L label, T timeline, W walk, Space play / pause in Animate, Tab next open port, Alt+↑ parent part, Ctrl+Z / Ctrl+Y undo / redo, Ctrl+S / Ctrl+O save / open JSON, ? for the list.
- The grid, label and timeline settings and the theme are remembered in the browser.
