// Example: 7-DoF arm. Seven actuators, each with two cycloidal 18:1 gearboxes, joined by
// links. Exported from the builder as JSON (only the project name was changed).
// Saved poses at the end: Home (straight up, as loaded), Ready, Reach, Pick left, Twist and Folded.
export const ARM_7DOF = {
  "format": "modular-robot",
  "version": 1,
  "convention": "standard",
  "units": { "length": "mm", "angle": "deg" },
  "name": "7DOF arm",
  "rootId": "base",
  "nodes": [
    {
      "id": "base",
      "type": "link",
      "name": "Base",
      "fixed": true,
      "spec": "dh",
      "parent": null,
      "outputs": [
        {
          "dh": { "a": 0, "alpha": 0, "d": 0, "theta": 0 },
          "pose": { "x": 0, "y": 0, "z": 0, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        }
      ],
      "children": ["lmvge18wi0"]
    },
    {
      "type": "link",
      "name": "L1",
      "spec": "pose",
      "outputs": [
        {
          "dh": { "a": 0, "alpha": 0, "d": 80, "theta": 0 },
          "pose": { "x": 56.515, "y": 0, "z": 101.6, "alpha": 0, "beta": 270, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 0 }
        }
      ],
      "children": ["amvge2dwc1"],
      "id": "lmvge18wi0",
      "parent": { "nodeId": "base", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "J1",
      "motor": "m1",
      "gearboxes": ["cycloidal_18", "cycloidal_18"],
      "mount": 0,
      "q": 0,
      "children": ["amvge2kgn2"],
      "id": "amvge2dwc1",
      "parent": { "nodeId": "lmvge18wi0", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "J2",
      "motor": "m1",
      "gearboxes": ["cycloidal_18", "cycloidal_18"],
      "mount": 0,
      "q": 0,
      "children": ["lmvge3r2n3"],
      "id": "amvge2kgn2",
      "parent": { "nodeId": "amvge2dwc1", "index": 0 }
    },
    {
      "type": "link",
      "name": "L2",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "theta": 180, "d": 25.4, "a": 127, "alpha": 180 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        }
      ],
      "children": ["lmvge4jo94"],
      "id": "lmvge3r2n3",
      "parent": { "nodeId": "amvge2kgn2", "index": 0 }
    },
    {
      "type": "link",
      "name": "L3",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "a": 0, "alpha": 0, "d": 44.449999999999996, "theta": 0 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        }
      ],
      "children": ["amvge4qfp5"],
      "id": "lmvge4jo94",
      "parent": { "nodeId": "lmvge3r2n3", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "J3",
      "motor": "m1",
      "gearboxes": ["cycloidal_18", "cycloidal_18"],
      "mount": 0,
      "q": 0,
      "children": ["amvge6dyy6"],
      "id": "amvge4qfp5",
      "parent": { "nodeId": "lmvge4jo94", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "J4",
      "motor": "m1",
      "gearboxes": ["cycloidal_18", "cycloidal_18"],
      "mount": 0,
      "q": 0,
      "children": ["lmvge72bs7"],
      "id": "amvge6dyy6",
      "parent": { "nodeId": "amvge4qfp5", "index": 0 }
    },
    {
      "type": "link",
      "name": "L4",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "a": 127, "alpha": 180, "d": 25.4, "theta": 180 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        }
      ],
      "children": ["lmvge767e8"],
      "id": "lmvge72bs7",
      "parent": { "nodeId": "amvge6dyy6", "index": 0 }
    },
    {
      "type": "link",
      "name": "L5",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "a": 0, "alpha": 0, "d": 44.449999999999996, "theta": 0 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        }
      ],
      "children": ["amvge7ccc9"],
      "id": "lmvge767e8",
      "parent": { "nodeId": "lmvge72bs7", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "J5",
      "motor": "m1",
      "gearboxes": ["cycloidal_18", "cycloidal_18"],
      "mount": 0,
      "q": 0,
      "children": ["amvge7ne1a"],
      "id": "amvge7ccc9",
      "parent": { "nodeId": "lmvge767e8", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "J6",
      "motor": "m1",
      "gearboxes": ["cycloidal_18", "cycloidal_18"],
      "mount": 0,
      "q": 0,
      "children": ["lmvge80a0b"],
      "id": "amvge7ne1a",
      "parent": { "nodeId": "amvge7ccc9", "index": 0 }
    },
    {
      "type": "link",
      "name": "L6",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "theta": 180, "d": 25.4, "a": 127, "alpha": 180 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        }
      ],
      "children": ["lmvge85z4c"],
      "id": "lmvge80a0b",
      "parent": { "nodeId": "amvge7ne1a", "index": 0 }
    },
    {
      "type": "link",
      "name": "L7",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "a": 0, "alpha": 0, "d": 44.449999999999996, "theta": 0 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        }
      ],
      "children": ["amvge8j71d"],
      "id": "lmvge85z4c",
      "parent": { "nodeId": "lmvge80a0b", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "J7",
      "motor": "m1",
      "gearboxes": ["cycloidal_18", "cycloidal_18"],
      "mount": 0,
      "q": 0,
      "children": [null],
      "id": "amvge8j71d",
      "parent": { "nodeId": "lmvge85z4c", "index": 0 }
    }
  ],
  "poses": [
    { "name": "Home", "q": { "amvge2dwc1": 0, "amvge2kgn2": 0, "amvge4qfp5": 0, "amvge6dyy6": 0, "amvge7ccc9": 0, "amvge7ne1a": 0, "amvge8j71d": 0 } },
    { "name": "Ready", "q": { "amvge2dwc1": 0, "amvge2kgn2": 25, "amvge4qfp5": 0, "amvge6dyy6": 75, "amvge7ccc9": 0, "amvge7ne1a": 70, "amvge8j71d": 0 } },
    { "name": "Reach", "q": { "amvge2dwc1": 0, "amvge2kgn2": 70, "amvge4qfp5": 0, "amvge6dyy6": 20, "amvge7ccc9": 0, "amvge7ne1a": 0, "amvge8j71d": 0 } },
    { "name": "Pick left", "q": { "amvge2dwc1": 90, "amvge2kgn2": 45, "amvge4qfp5": 0, "amvge6dyy6": 70, "amvge7ccc9": 0, "amvge7ne1a": 60, "amvge8j71d": 0 } },
    { "name": "Twist", "q": { "amvge2dwc1": -40, "amvge2kgn2": 30, "amvge4qfp5": 60, "amvge6dyy6": 60, "amvge7ccc9": -60, "amvge7ne1a": 45, "amvge8j71d": 90 } },
    { "name": "Folded", "q": { "amvge2dwc1": 0, "amvge2kgn2": -10, "amvge4qfp5": 0, "amvge6dyy6": 120, "amvge7ccc9": 0, "amvge7ne1a": -110, "amvge8j71d": 0 } }
  ],
  "animations": [
    {"name": "Pick and place", "loop": true, "speed": 50, "pause": true, "steps": [{"pose": "Ready", "hold": 0.3}, {"pose": "Reach", "hold": 0}, {"pose": "Pick left", "hold": 0.5}, {"pose": "Ready", "hold": 0}]},
    {"name": "Wrist twist", "loop": true, "speed": 50, "pause": true, "steps": [{"pose": "Ready", "hold": 0.3}, {"pose": "Twist", "hold": 0.5}]},
    {"name": "Fold away", "loop": true, "speed": 50, "pause": true, "steps": [{"pose": "Home", "hold": 0.5}, {"pose": "Folded", "hold": 0.5}]}
  ]
};
