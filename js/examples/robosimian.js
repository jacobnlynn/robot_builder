// Example: JPL's RoboSimian, a four-limbed robot. A rectangular chassis with a seven-joint limb at
// each corner (FL, FR, RR, RL), every limb the 7DOF arm's chain ending in a ball hand. The limbs are
// the same limb turned 90° from corner to corner, so one set of angles poses all four the same way.
// Saved poses at the end: Stand (as loaded), Tall, Low, Wide, Step (front left hand lifted) and
// Wave (front left hand raised high). J4 and J6 stay within ±120°, past which the motors on
// either side of them collide.
export const ROBOSIMIAN = {
  "format": "modular-robot",
  "version": 1,
  "convention": "standard",
  "units": {
    "length": "mm",
    "angle": "deg"
  },
  "name": "RoboSimian",
  "rootId": "base",
  "nodes": [
    {
      "id": "base",
      "type": "link",
      "name": "Base",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 0,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "chassis"
      ],
      "parent": null,
      "fixed": true
    },
    {
      "id": "chassis",
      "type": "link",
      "name": "Chassis",
      "spec": "pose",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 0,
            "theta": 0
          },
          "pose": {
            "x": 300,
            "y": 200,
            "z": 0,
            "alpha": 180,
            "beta": 0,
            "gamma": 45,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        },
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 0,
            "theta": 0
          },
          "pose": {
            "x": 300,
            "y": -200,
            "z": 0,
            "alpha": 180,
            "beta": 0,
            "gamma": -45,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        },
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 0,
            "theta": 0
          },
          "pose": {
            "x": -300,
            "y": -200,
            "z": 0,
            "alpha": 180,
            "beta": 0,
            "gamma": -135,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        },
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 0,
            "theta": 0
          },
          "pose": {
            "x": -300,
            "y": 200,
            "z": 0,
            "alpha": 180,
            "beta": 0,
            "gamma": 135,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "fl_j1",
        "fr_j1",
        "rr_j1",
        "rl_j1"
      ],
      "parent": {
        "nodeId": "base",
        "index": 0
      }
    },
    {
      "id": "fl_j1",
      "type": "actuator",
      "name": "FL_J1",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": -90,
      "children": [
        "fl_j2"
      ],
      "parent": {
        "nodeId": "chassis",
        "index": 0
      }
    },
    {
      "id": "fl_j2",
      "type": "actuator",
      "name": "FL_J2",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 45,
      "children": [
        "fl_l2a"
      ],
      "parent": {
        "nodeId": "fl_j1",
        "index": 0
      }
    },
    {
      "id": "fl_l2a",
      "type": "link",
      "name": "FL_L2a",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "theta": 180,
            "d": 25.4,
            "a": 127,
            "alpha": 180
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "fl_l2b"
      ],
      "parent": {
        "nodeId": "fl_j2",
        "index": 0
      }
    },
    {
      "id": "fl_l2b",
      "type": "link",
      "name": "FL_L2b",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 44.45,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "fl_j3"
      ],
      "parent": {
        "nodeId": "fl_l2a",
        "index": 0
      }
    },
    {
      "id": "fl_j3",
      "type": "actuator",
      "name": "FL_J3",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 0,
      "children": [
        "fl_j4"
      ],
      "parent": {
        "nodeId": "fl_l2b",
        "index": 0
      }
    },
    {
      "id": "fl_j4",
      "type": "actuator",
      "name": "FL_J4",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 90,
      "children": [
        "fl_l4a"
      ],
      "parent": {
        "nodeId": "fl_j3",
        "index": 0
      }
    },
    {
      "id": "fl_l4a",
      "type": "link",
      "name": "FL_L4a",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 127,
            "alpha": 180,
            "d": 25.4,
            "theta": 180
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "fl_l4b"
      ],
      "parent": {
        "nodeId": "fl_j4",
        "index": 0
      }
    },
    {
      "id": "fl_l4b",
      "type": "link",
      "name": "FL_L4b",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 44.45,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "fl_j5"
      ],
      "parent": {
        "nodeId": "fl_l4a",
        "index": 0
      }
    },
    {
      "id": "fl_j5",
      "type": "actuator",
      "name": "FL_J5",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 0,
      "children": [
        "fl_j6"
      ],
      "parent": {
        "nodeId": "fl_l4b",
        "index": 0
      }
    },
    {
      "id": "fl_j6",
      "type": "actuator",
      "name": "FL_J6",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": -120,
      "children": [
        "fl_l6a"
      ],
      "parent": {
        "nodeId": "fl_j5",
        "index": 0
      }
    },
    {
      "id": "fl_l6a",
      "type": "link",
      "name": "FL_L6a",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "theta": 180,
            "d": 25.4,
            "a": 127,
            "alpha": 180
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "fl_l6b"
      ],
      "parent": {
        "nodeId": "fl_j6",
        "index": 0
      }
    },
    {
      "id": "fl_l6b",
      "type": "link",
      "name": "FL_L6b",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 44.45,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "fl_j7"
      ],
      "parent": {
        "nodeId": "fl_l6a",
        "index": 0
      }
    },
    {
      "id": "fl_j7",
      "type": "actuator",
      "name": "FL_J7",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 0,
      "children": [
        "fl_hand"
      ],
      "parent": {
        "nodeId": "fl_l6b",
        "index": 0
      }
    },
    {
      "id": "fl_hand",
      "type": "link",
      "name": "FL_Hand",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 60,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          },
          "end": "sphere",
          "sphere": 20
        }
      ],
      "children": [
        null
      ],
      "parent": {
        "nodeId": "fl_j7",
        "index": 0
      }
    },
    {
      "id": "fr_j1",
      "type": "actuator",
      "name": "FR_J1",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": -90,
      "children": [
        "fr_j2"
      ],
      "parent": {
        "nodeId": "chassis",
        "index": 1
      }
    },
    {
      "id": "fr_j2",
      "type": "actuator",
      "name": "FR_J2",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 45,
      "children": [
        "fr_l2a"
      ],
      "parent": {
        "nodeId": "fr_j1",
        "index": 0
      }
    },
    {
      "id": "fr_l2a",
      "type": "link",
      "name": "FR_L2a",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "theta": 180,
            "d": 25.4,
            "a": 127,
            "alpha": 180
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "fr_l2b"
      ],
      "parent": {
        "nodeId": "fr_j2",
        "index": 0
      }
    },
    {
      "id": "fr_l2b",
      "type": "link",
      "name": "FR_L2b",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 44.45,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "fr_j3"
      ],
      "parent": {
        "nodeId": "fr_l2a",
        "index": 0
      }
    },
    {
      "id": "fr_j3",
      "type": "actuator",
      "name": "FR_J3",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 0,
      "children": [
        "fr_j4"
      ],
      "parent": {
        "nodeId": "fr_l2b",
        "index": 0
      }
    },
    {
      "id": "fr_j4",
      "type": "actuator",
      "name": "FR_J4",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 90,
      "children": [
        "fr_l4a"
      ],
      "parent": {
        "nodeId": "fr_j3",
        "index": 0
      }
    },
    {
      "id": "fr_l4a",
      "type": "link",
      "name": "FR_L4a",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 127,
            "alpha": 180,
            "d": 25.4,
            "theta": 180
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "fr_l4b"
      ],
      "parent": {
        "nodeId": "fr_j4",
        "index": 0
      }
    },
    {
      "id": "fr_l4b",
      "type": "link",
      "name": "FR_L4b",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 44.45,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "fr_j5"
      ],
      "parent": {
        "nodeId": "fr_l4a",
        "index": 0
      }
    },
    {
      "id": "fr_j5",
      "type": "actuator",
      "name": "FR_J5",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 0,
      "children": [
        "fr_j6"
      ],
      "parent": {
        "nodeId": "fr_l4b",
        "index": 0
      }
    },
    {
      "id": "fr_j6",
      "type": "actuator",
      "name": "FR_J6",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": -120,
      "children": [
        "fr_l6a"
      ],
      "parent": {
        "nodeId": "fr_j5",
        "index": 0
      }
    },
    {
      "id": "fr_l6a",
      "type": "link",
      "name": "FR_L6a",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "theta": 180,
            "d": 25.4,
            "a": 127,
            "alpha": 180
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "fr_l6b"
      ],
      "parent": {
        "nodeId": "fr_j6",
        "index": 0
      }
    },
    {
      "id": "fr_l6b",
      "type": "link",
      "name": "FR_L6b",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 44.45,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "fr_j7"
      ],
      "parent": {
        "nodeId": "fr_l6a",
        "index": 0
      }
    },
    {
      "id": "fr_j7",
      "type": "actuator",
      "name": "FR_J7",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 0,
      "children": [
        "fr_hand"
      ],
      "parent": {
        "nodeId": "fr_l6b",
        "index": 0
      }
    },
    {
      "id": "fr_hand",
      "type": "link",
      "name": "FR_Hand",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 60,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          },
          "end": "sphere",
          "sphere": 20
        }
      ],
      "children": [
        null
      ],
      "parent": {
        "nodeId": "fr_j7",
        "index": 0
      }
    },
    {
      "id": "rr_j1",
      "type": "actuator",
      "name": "RR_J1",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": -90,
      "children": [
        "rr_j2"
      ],
      "parent": {
        "nodeId": "chassis",
        "index": 2
      }
    },
    {
      "id": "rr_j2",
      "type": "actuator",
      "name": "RR_J2",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 45,
      "children": [
        "rr_l2a"
      ],
      "parent": {
        "nodeId": "rr_j1",
        "index": 0
      }
    },
    {
      "id": "rr_l2a",
      "type": "link",
      "name": "RR_L2a",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "theta": 180,
            "d": 25.4,
            "a": 127,
            "alpha": 180
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "rr_l2b"
      ],
      "parent": {
        "nodeId": "rr_j2",
        "index": 0
      }
    },
    {
      "id": "rr_l2b",
      "type": "link",
      "name": "RR_L2b",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 44.45,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "rr_j3"
      ],
      "parent": {
        "nodeId": "rr_l2a",
        "index": 0
      }
    },
    {
      "id": "rr_j3",
      "type": "actuator",
      "name": "RR_J3",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 0,
      "children": [
        "rr_j4"
      ],
      "parent": {
        "nodeId": "rr_l2b",
        "index": 0
      }
    },
    {
      "id": "rr_j4",
      "type": "actuator",
      "name": "RR_J4",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 90,
      "children": [
        "rr_l4a"
      ],
      "parent": {
        "nodeId": "rr_j3",
        "index": 0
      }
    },
    {
      "id": "rr_l4a",
      "type": "link",
      "name": "RR_L4a",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 127,
            "alpha": 180,
            "d": 25.4,
            "theta": 180
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "rr_l4b"
      ],
      "parent": {
        "nodeId": "rr_j4",
        "index": 0
      }
    },
    {
      "id": "rr_l4b",
      "type": "link",
      "name": "RR_L4b",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 44.45,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "rr_j5"
      ],
      "parent": {
        "nodeId": "rr_l4a",
        "index": 0
      }
    },
    {
      "id": "rr_j5",
      "type": "actuator",
      "name": "RR_J5",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 0,
      "children": [
        "rr_j6"
      ],
      "parent": {
        "nodeId": "rr_l4b",
        "index": 0
      }
    },
    {
      "id": "rr_j6",
      "type": "actuator",
      "name": "RR_J6",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": -120,
      "children": [
        "rr_l6a"
      ],
      "parent": {
        "nodeId": "rr_j5",
        "index": 0
      }
    },
    {
      "id": "rr_l6a",
      "type": "link",
      "name": "RR_L6a",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "theta": 180,
            "d": 25.4,
            "a": 127,
            "alpha": 180
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "rr_l6b"
      ],
      "parent": {
        "nodeId": "rr_j6",
        "index": 0
      }
    },
    {
      "id": "rr_l6b",
      "type": "link",
      "name": "RR_L6b",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 44.45,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "rr_j7"
      ],
      "parent": {
        "nodeId": "rr_l6a",
        "index": 0
      }
    },
    {
      "id": "rr_j7",
      "type": "actuator",
      "name": "RR_J7",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 0,
      "children": [
        "rr_hand"
      ],
      "parent": {
        "nodeId": "rr_l6b",
        "index": 0
      }
    },
    {
      "id": "rr_hand",
      "type": "link",
      "name": "RR_Hand",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 60,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          },
          "end": "sphere",
          "sphere": 20
        }
      ],
      "children": [
        null
      ],
      "parent": {
        "nodeId": "rr_j7",
        "index": 0
      }
    },
    {
      "id": "rl_j1",
      "type": "actuator",
      "name": "RL_J1",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": -90,
      "children": [
        "rl_j2"
      ],
      "parent": {
        "nodeId": "chassis",
        "index": 3
      }
    },
    {
      "id": "rl_j2",
      "type": "actuator",
      "name": "RL_J2",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 45,
      "children": [
        "rl_l2a"
      ],
      "parent": {
        "nodeId": "rl_j1",
        "index": 0
      }
    },
    {
      "id": "rl_l2a",
      "type": "link",
      "name": "RL_L2a",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "theta": 180,
            "d": 25.4,
            "a": 127,
            "alpha": 180
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "rl_l2b"
      ],
      "parent": {
        "nodeId": "rl_j2",
        "index": 0
      }
    },
    {
      "id": "rl_l2b",
      "type": "link",
      "name": "RL_L2b",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 44.45,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "rl_j3"
      ],
      "parent": {
        "nodeId": "rl_l2a",
        "index": 0
      }
    },
    {
      "id": "rl_j3",
      "type": "actuator",
      "name": "RL_J3",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 0,
      "children": [
        "rl_j4"
      ],
      "parent": {
        "nodeId": "rl_l2b",
        "index": 0
      }
    },
    {
      "id": "rl_j4",
      "type": "actuator",
      "name": "RL_J4",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 90,
      "children": [
        "rl_l4a"
      ],
      "parent": {
        "nodeId": "rl_j3",
        "index": 0
      }
    },
    {
      "id": "rl_l4a",
      "type": "link",
      "name": "RL_L4a",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 127,
            "alpha": 180,
            "d": 25.4,
            "theta": 180
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "rl_l4b"
      ],
      "parent": {
        "nodeId": "rl_j4",
        "index": 0
      }
    },
    {
      "id": "rl_l4b",
      "type": "link",
      "name": "RL_L4b",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 44.45,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "rl_j5"
      ],
      "parent": {
        "nodeId": "rl_l4a",
        "index": 0
      }
    },
    {
      "id": "rl_j5",
      "type": "actuator",
      "name": "RL_J5",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 0,
      "children": [
        "rl_j6"
      ],
      "parent": {
        "nodeId": "rl_l4b",
        "index": 0
      }
    },
    {
      "id": "rl_j6",
      "type": "actuator",
      "name": "RL_J6",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": -120,
      "children": [
        "rl_l6a"
      ],
      "parent": {
        "nodeId": "rl_j5",
        "index": 0
      }
    },
    {
      "id": "rl_l6a",
      "type": "link",
      "name": "RL_L6a",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "theta": 180,
            "d": 25.4,
            "a": 127,
            "alpha": 180
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "rl_l6b"
      ],
      "parent": {
        "nodeId": "rl_j6",
        "index": 0
      }
    },
    {
      "id": "rl_l6b",
      "type": "link",
      "name": "RL_L6b",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 44.45,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          }
        }
      ],
      "children": [
        "rl_j7"
      ],
      "parent": {
        "nodeId": "rl_l6a",
        "index": 0
      }
    },
    {
      "id": "rl_j7",
      "type": "actuator",
      "name": "RL_J7",
      "motor": "m1",
      "gearboxes": [
        "cycloidal_18",
        "cycloidal_18"
      ],
      "mount": 0,
      "q": 0,
      "children": [
        "rl_hand"
      ],
      "parent": {
        "nodeId": "rl_l6b",
        "index": 0
      }
    },
    {
      "id": "rl_hand",
      "type": "link",
      "name": "RL_Hand",
      "spec": "dh",
      "outputs": [
        {
          "dh": {
            "a": 0,
            "alpha": 0,
            "d": 60,
            "theta": 0
          },
          "pose": {
            "x": 0,
            "y": 0,
            "z": 0,
            "alpha": 0,
            "beta": 0,
            "gamma": 0,
            "ref": "input"
          },
          "rod": {
            "lon": 0,
            "lat": 90
          },
          "end": "sphere",
          "sphere": 20
        }
      ],
      "children": [
        null
      ],
      "parent": {
        "nodeId": "rl_j7",
        "index": 0
      }
    }
  ],
  "poses": [
    { "name": "Stand", "q": { "fl_j1": -90, "fl_j2": 45, "fl_j3": 0, "fl_j4": 90, "fl_j5": 0, "fl_j6": -120, "fl_j7": 0, "fr_j1": -90, "fr_j2": 45, "fr_j3": 0, "fr_j4": 90, "fr_j5": 0, "fr_j6": -120, "fr_j7": 0, "rr_j1": -90, "rr_j2": 45, "rr_j3": 0, "rr_j4": 90, "rr_j5": 0, "rr_j6": -120, "rr_j7": 0, "rl_j1": -90, "rl_j2": 45, "rl_j3": 0, "rl_j4": 90, "rl_j5": 0, "rl_j6": -120, "rl_j7": 0 } },
    { "name": "Tall", "q": { "fl_j1": -90, "fl_j2": 45, "fl_j3": 0, "fl_j4": 90, "fl_j5": 0, "fl_j6": -90, "fl_j7": 0, "fr_j1": -90, "fr_j2": 45, "fr_j3": 0, "fr_j4": 90, "fr_j5": 0, "fr_j6": -90, "fr_j7": 0, "rr_j1": -90, "rr_j2": 45, "rr_j3": 0, "rr_j4": 90, "rr_j5": 0, "rr_j6": -90, "rr_j7": 0, "rl_j1": -90, "rl_j2": 45, "rl_j3": 0, "rl_j4": 90, "rl_j5": 0, "rl_j6": -90, "rl_j7": 0 } },
    { "name": "Low", "q": { "fl_j1": -90, "fl_j2": 20, "fl_j3": 0, "fl_j4": 120, "fl_j5": 0, "fl_j6": -120, "fl_j7": 0, "fr_j1": -90, "fr_j2": 20, "fr_j3": 0, "fr_j4": 120, "fr_j5": 0, "fr_j6": -120, "fr_j7": 0, "rr_j1": -90, "rr_j2": 20, "rr_j3": 0, "rr_j4": 120, "rr_j5": 0, "rr_j6": -120, "rr_j7": 0, "rl_j1": -90, "rl_j2": 20, "rl_j3": 0, "rl_j4": 120, "rl_j5": 0, "rl_j6": -120, "rl_j7": 0 } },
    { "name": "Wide", "q": { "fl_j1": -90, "fl_j2": 20, "fl_j3": 0, "fl_j4": 90, "fl_j5": 0, "fl_j6": -110, "fl_j7": 0, "fr_j1": -90, "fr_j2": 20, "fr_j3": 0, "fr_j4": 90, "fr_j5": 0, "fr_j6": -110, "fr_j7": 0, "rr_j1": -90, "rr_j2": 20, "rr_j3": 0, "rr_j4": 90, "rr_j5": 0, "rr_j6": -110, "rr_j7": 0, "rl_j1": -90, "rl_j2": 20, "rl_j3": 0, "rl_j4": 90, "rl_j5": 0, "rl_j6": -110, "rl_j7": 0 } },
    { "name": "Step", "q": { "fl_j1": -90, "fl_j2": -5, "fl_j3": 0, "fl_j4": 120, "fl_j5": 0, "fl_j6": -120, "fl_j7": 0, "fr_j1": -90, "fr_j2": 45, "fr_j3": 0, "fr_j4": 90, "fr_j5": 0, "fr_j6": -120, "fr_j7": 0, "rr_j1": -90, "rr_j2": 45, "rr_j3": 0, "rr_j4": 90, "rr_j5": 0, "rr_j6": -120, "rr_j7": 0, "rl_j1": -90, "rl_j2": 45, "rl_j3": 0, "rl_j4": 90, "rl_j5": 0, "rl_j6": -120, "rl_j7": 0 } },
    { "name": "Wave", "q": { "fl_j1": -90, "fl_j2": -60, "fl_j3": 0, "fl_j4": 30, "fl_j5": 0, "fl_j6": -120, "fl_j7": 0, "fr_j1": -90, "fr_j2": 45, "fr_j3": 0, "fr_j4": 90, "fr_j5": 0, "fr_j6": -120, "fr_j7": 0, "rr_j1": -90, "rr_j2": 45, "rr_j3": 0, "rr_j4": 90, "rr_j5": 0, "rr_j6": -120, "rr_j7": 0, "rl_j1": -90, "rl_j2": 45, "rl_j3": 0, "rl_j4": 90, "rl_j5": 0, "rl_j6": -120, "rl_j7": 0 } }
  ]
};
