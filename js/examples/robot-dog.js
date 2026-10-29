// Example: four-legged robot dog. A mounting base with four outputs (one per leg), each
// leg a hip, upper and lower actuator joined by links. Exported from the builder as JSON.
// The thigh links (L2, L4, L6, L8) have d = 1.75 in so each knee motor sits fully outboard of its
// hip motor (5 mm gap) and can't swing into it.
// Saved poses at the end: Stand (as loaded), Crouch, Tall, Sit (back legs folded) and Shake
// (front right paw raised).
export const ROBOT_DOG = {
  "format": "modular-robot",
  "version": 1,
  "convention": "standard",
  "units": { "length": "mm", "angle": "deg" },
  "name": "robot dog",
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
      "children": ["lmvg1u9820"]
    },
    {
      "type": "link",
      "name": "Mounting Base",
      "spec": "pose",
      "outputs": [
        {
          "dh": { "a": 0, "alpha": 0, "d": 0, "theta": 0 },
          "pose": { "x": 127, "y": 127, "z": 0, "alpha": 180, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        },
        {
          "dh": { "a": 0, "alpha": 0, "d": 0, "theta": 0 },
          "pose": { "x": 127, "y": 0, "z": 0, "alpha": 180, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        },
        {
          "dh": { "a": 0, "alpha": 0, "d": 80, "theta": 0 },
          "pose": { "x": 0, "y": 0, "z": 0, "alpha": 180, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        },
        {
          "dh": { "a": 0, "alpha": 0, "d": 80, "theta": 0 },
          "pose": { "x": 0, "y": 127, "z": 0, "alpha": 180, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        }
      ],
      "children": ["amvg1udyg1", "amvg1ugoj2", "amvg1upky3", "amvg1uw1a4"],
      "id": "lmvg1u9820",
      "parent": { "nodeId": "base", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "top_left_hip",
      "motor": "m1",
      "gearboxes": ["planetary_9"],
      "mount": 0,
      "q": 0,
      "children": ["amvg1w8hj8"],
      "id": "amvg1udyg1",
      "parent": { "nodeId": "lmvg1u9820", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "top_left_upper",
      "motor": "m1",
      "gearboxes": ["planetary_9"],
      "mount": -90,
      "q": -61,
      "children": ["lmvg27lg9i"],
      "id": "amvg1w8hj8",
      "parent": { "nodeId": "amvg1udyg1", "index": 0 }
    },
    {
      "type": "link",
      "name": "L8",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "a": 139.7, "alpha": -90, "d": 44.45, "theta": -90 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        }
      ],
      "children": ["amvg281qfj"],
      "id": "lmvg27lg9i",
      "parent": { "nodeId": "amvg1w8hj8", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "top_left_lower",
      "motor": "m1",
      "gearboxes": ["planetary_9"],
      "mount": -90,
      "q": 115,
      "children": ["lmvg28bsuk"],
      "id": "amvg281qfj",
      "parent": { "nodeId": "lmvg27lg9i", "index": 0 }
    },
    {
      "type": "link",
      "name": "L9",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "a": 177.79999999999998, "alpha": 0, "d": 0, "theta": 90 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 },
          "end": "sphere",
          "sphere": 20
        }
      ],
      "children": [null],
      "id": "lmvg28bsuk",
      "parent": { "nodeId": "amvg281qfj", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "top_right_hip",
      "motor": "m1",
      "gearboxes": ["planetary_9"],
      "mount": 0,
      "q": 0,
      "children": ["amvg1w1317"],
      "id": "amvg1ugoj2",
      "parent": { "nodeId": "lmvg1u9820", "index": 1 }
    },
    {
      "type": "actuator",
      "name": "top_right_upper",
      "motor": "m1",
      "gearboxes": ["planetary_9"],
      "mount": 90,
      "q": 61,
      "children": ["lmvg25zugf"],
      "id": "amvg1w1317",
      "parent": { "nodeId": "amvg1ugoj2", "index": 0 }
    },
    {
      "type": "link",
      "name": "L6",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "a": 139.7, "alpha": 90, "d": 44.45, "theta": 90 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        }
      ],
      "children": ["amvg26757g"],
      "id": "lmvg25zugf",
      "parent": { "nodeId": "amvg1w1317", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "top_right_lower",
      "motor": "m1",
      "gearboxes": ["planetary_9"],
      "mount": 90,
      "q": -115,
      "children": ["lmvg26o36h"],
      "id": "amvg26757g",
      "parent": { "nodeId": "lmvg25zugf", "index": 0 }
    },
    {
      "type": "link",
      "name": "L7",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "a": 177.79999999999998, "alpha": 0, "d": 0, "theta": -90 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 },
          "end": "sphere",
          "sphere": 20
        }
      ],
      "children": [null],
      "id": "lmvg26o36h",
      "parent": { "nodeId": "amvg26757g", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "bottom_right_hip",
      "motor": "m1",
      "gearboxes": ["planetary_9"],
      "mount": 180,
      "q": 0,
      "children": ["amvg1vfcl6"],
      "id": "amvg1upky3",
      "parent": { "nodeId": "lmvg1u9820", "index": 2 }
    },
    {
      "type": "actuator",
      "name": "bottom_right_upper",
      "motor": "m1",
      "gearboxes": ["planetary_9"],
      "mount": -90,
      "q": 61,
      "children": ["lmvg20t8jc"],
      "id": "amvg1vfcl6",
      "parent": { "nodeId": "amvg1upky3", "index": 0 }
    },
    {
      "type": "link",
      "name": "L4",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "a": 139.7, "alpha": 90, "d": 44.45, "theta": -90 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        }
      ],
      "children": ["amvg211bjd"],
      "id": "lmvg20t8jc",
      "parent": { "nodeId": "amvg1vfcl6", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "bottom_right_lower",
      "motor": "m1",
      "gearboxes": ["planetary_9"],
      "mount": 90,
      "q": -115,
      "children": ["lmvg21edre"],
      "id": "amvg211bjd",
      "parent": { "nodeId": "lmvg20t8jc", "index": 0 }
    },
    {
      "type": "link",
      "name": "L5",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "a": 177.79999999999998, "alpha": 9, "d": 0, "theta": -90 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 },
          "end": "sphere",
          "sphere": 20
        }
      ],
      "children": [null],
      "id": "lmvg21edre",
      "parent": { "nodeId": "amvg211bjd", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "bottom_left_hip",
      "motor": "m1",
      "gearboxes": ["planetary_9"],
      "mount": 180,
      "q": 0,
      "children": ["amvg1v6xf5"],
      "id": "amvg1uw1a4",
      "parent": { "nodeId": "lmvg1u9820", "index": 3 }
    },
    {
      "type": "actuator",
      "name": "bottom_left_upper",
      "motor": "m1",
      "gearboxes": ["planetary_9"],
      "mount": 90,
      "q": -61,
      "children": ["lmvg1xv939"],
      "id": "amvg1v6xf5",
      "parent": { "nodeId": "amvg1uw1a4", "index": 0 }
    },
    {
      "type": "link",
      "name": "L2",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "a": 139.7, "alpha": -90, "d": 44.45, "theta": 90 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 }
        }
      ],
      "children": ["amvg1ydfta"],
      "id": "lmvg1xv939",
      "parent": { "nodeId": "amvg1v6xf5", "index": 0 }
    },
    {
      "type": "actuator",
      "name": "bottom_left_lower",
      "motor": "m1",
      "gearboxes": ["planetary_9"],
      "mount": -90,
      "q": 115,
      "children": ["lmvg1ynyib"],
      "id": "amvg1ydfta",
      "parent": { "nodeId": "lmvg1xv939", "index": 0 }
    },
    {
      "type": "link",
      "name": "L3",
      "spec": "dh",
      "outputs": [
        {
          "dh": { "a": 177.79999999999998, "alpha": 0, "d": 0, "theta": 90 },
          "pose": { "x": 0, "y": 0, "z": 80, "alpha": 0, "beta": 0, "gamma": 0, "ref": "input" },
          "rod": { "lon": 0, "lat": 90 },
          "end": "sphere",
          "sphere": 20
        }
      ],
      "children": [null],
      "id": "lmvg1ynyib",
      "parent": { "nodeId": "amvg1ydfta", "index": 0 }
    }
  ],
  "poses": [
    { "name": "Stand", "q": { "amvg1udyg1": 0, "amvg1w8hj8": -61, "amvg281qfj": 115, "amvg1ugoj2": 0, "amvg1w1317": 61, "amvg26757g": -115, "amvg1upky3": 0, "amvg1vfcl6": 61, "amvg211bjd": -115, "amvg1uw1a4": 0, "amvg1v6xf5": -61, "amvg1ydfta": 115 } },
    { "name": "Crouch", "q": { "amvg1udyg1": 0, "amvg1w8hj8": -75, "amvg281qfj": 150, "amvg1ugoj2": 0, "amvg1w1317": 75, "amvg26757g": -150, "amvg1upky3": 0, "amvg1vfcl6": 75, "amvg211bjd": -150, "amvg1uw1a4": 0, "amvg1v6xf5": -75, "amvg1ydfta": 150 } },
    { "name": "Tall", "q": { "amvg1udyg1": 0, "amvg1w8hj8": -45, "amvg281qfj": 80, "amvg1ugoj2": 0, "amvg1w1317": 45, "amvg26757g": -80, "amvg1upky3": 0, "amvg1vfcl6": 45, "amvg211bjd": -80, "amvg1uw1a4": 0, "amvg1v6xf5": -45, "amvg1ydfta": 80 } },
    { "name": "Sit", "q": { "amvg1udyg1": 0, "amvg1w8hj8": -45, "amvg281qfj": 80, "amvg1ugoj2": 0, "amvg1w1317": 45, "amvg26757g": -80, "amvg1upky3": 0, "amvg1vfcl6": 80, "amvg211bjd": -165, "amvg1uw1a4": 0, "amvg1v6xf5": -80, "amvg1ydfta": 165 } },
    { "name": "Shake", "q": { "amvg1udyg1": 0, "amvg1w8hj8": -61, "amvg281qfj": 115, "amvg1ugoj2": 0, "amvg1w1317": 125, "amvg26757g": -55, "amvg1upky3": 0, "amvg1vfcl6": 61, "amvg211bjd": -115, "amvg1uw1a4": 0, "amvg1v6xf5": -61, "amvg1ydfta": 115 } }
  ]
};
