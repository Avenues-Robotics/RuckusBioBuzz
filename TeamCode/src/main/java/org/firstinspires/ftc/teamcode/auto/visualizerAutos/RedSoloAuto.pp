{
  "startPoint": {
    "x": 56.0,
    "y": 9.196,
    "heading": "constant",
    "degrees": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "l1",
      "name": "Start to the corner loop",
      "endPoint": {
        "x": 17.155,
        "y": 22.33,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 225,
        "headingCurve": 1.0
      },
      "controlPoints": [
        {
          "x": 36.0,
          "y": 24.0
        }
      ],
      "speed": 1,
      "stopAtEnd": false,
      "eventMarkers": [
        {
          "id": "ev-intake-path1",
          "name": "Intake On",
          "triggerType": "parametric",
          "position": 0,
          "triggerMs": 0,
          "durationMs": 0
        }
      ]
    },
    {
      "id": "l2",
      "name": "Corner loop, back",
      "endPoint": {
        "x": 10.222,
        "y": 10.299,
        "heading": "linear",
        "startDeg": 225,
        "endDeg": 270,
        "headingCurve": 1.0
      },
      "controlPoints": [
        {
          "x": 15.361,
          "y": 22.12
        },
        {
          "x": 11.204,
          "y": 17.548
        }
      ],
      "speed": 1
    },
    {
      "id": "l3",
      "name": "Up the field to shoot",
      "endPoint": {
        "x": 46.48,
        "y": 125.857,
        "heading": "linear",
        "startDeg": 270,
        "endDeg": 90,
        "headingCurve": 0.7
      },
      "controlPoints": [
        {
          "x": 20.787,
          "y": 41.881
        },
        {
          "x": 24.377,
          "y": 92.484
        },
        {
          "x": 48.724,
          "y": 111.429
        }
      ],
      "speed": 1,
      "eventMarkers": [
        {
          "id": "ev-1",
          "name": "Scoopula Down",
          "triggerType": "parametric",
          "position": 0.38,
          "triggerMs": 0,
          "durationMs": 300
        },
        {
          "id": "ev-intake-flower-1",
          "name": "Intake On",
          "triggerType": "parametric",
          "position": 0.38,
          "triggerMs": 0,
          "durationMs": 0
        }
      ]
    },
    {
      "id": "l4",
      "name": "Back off",
      "endPoint": {
        "x": 58.812,
        "y": 115.566,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 270,
        "headingCurve": 2.0
      },
      "controlPoints": [],
      "speed": 1,
      "eventMarkers": [
        {
          "id": "ev-2",
          "name": "Scoopula Up",
          "triggerType": "parametric",
          "position": 0.5,
          "triggerMs": 0,
          "durationMs": 300
        }
      ]
    },
    {
      "id": "l5",
      "name": "Down the field to shoot",
      "endPoint": {
        "x": 57.943,
        "y": 10.419,
        "heading": "linear",
        "startDeg": 270,
        "endDeg": 270,
        "headingCurve": 1.0
      },
      "controlPoints": [
        {
          "x": 61.964,
          "y": 56.009
        }
      ],
      "speed": 1,
      "eventMarkers": [
        {
          "id": "ev-intake-on",
          "name": "Intake On",
          "triggerType": "parametric",
          "position": 0,
          "triggerMs": 0,
          "durationMs": 0
        }
      ],
      "cameraPick": {
        "enabled": true,
        "alliance": "RED",
        "pickY": 59,
        "wallY": 9,
        "xMin": 8.5,
        "xMax": 58.0,
        "previewX": 57.943,
        "hfovDeg": 43,
        "cameraForward": 9
      }
    },
    {
      "id": "l6",
      "name": "To the flower",
      "endPoint": {
        "x": 15.442,
        "y": 47.215,
        "heading": "linear",
        "startDeg": 270,
        "endDeg": 180,
        "headingCurve": 1.0
      },
      "controlPoints": [
        {
          "x": 49.769,
          "y": 42.297
        }
      ],
      "speed": 1,
      "stopAtEnd": true,
      "eventMarkers": [
        {
          "id": "ev-3",
          "name": "Scoopula Down",
          "triggerType": "parametric",
          "position": 0.28,
          "triggerMs": 0,
          "durationMs": 300
        },
        {
          "id": "ev-intake-flower-2",
          "name": "Intake On",
          "triggerType": "parametric",
          "position": 0.28,
          "triggerMs": 0,
          "durationMs": 0
        }
      ],
      "turnFirst": true
    },
    {
      "id": "l7",
      "name": "To shoot",
      "endPoint": {
        "x": 45.308,
        "y": 11.717,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 180,
        "headingCurve": 1.0
      },
      "controlPoints": [],
      "speed": 1,
      "eventMarkers": [
        {
          "id": "ev-4",
          "name": "Scoopula Up",
          "triggerType": "parametric",
          "position": 0.37,
          "triggerMs": 0,
          "durationMs": 300
        }
      ]
    },
    {
      "id": "l8",
      "name": "Park",
      "endPoint": {
        "x": 10.466,
        "y": 101.157,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 180,
        "headingCurve": 1.0
      },
      "controlPoints": [
        {
          "x": 11.313,
          "y": 54.838
        }
      ],
      "speed": 1
    }
  ],
  "sequence": [
    {
      "kind": "event",
      "id": "shoot-1",
      "name": "Shoot",
      "durationMs": 1500
    },
    {
      "kind": "path",
      "lineId": "l1"
    },
    {
      "kind": "path",
      "lineId": "l2"
    },
    {
      "kind": "wait",
      "id": "wait-1",
      "name": "Wait",
      "durationMs": 500
    },
    {
      "kind": "path",
      "lineId": "l3"
    },
    {
      "kind": "event",
      "id": "shoot-2",
      "name": "Shoot",
      "durationMs": 1500
    },
    {
      "kind": "path",
      "lineId": "l4"
    },
    {
      "kind": "wait",
      "id": "wait-2",
      "name": "Wait",
      "durationMs": 2000
    },
    {
      "kind": "path",
      "lineId": "l5"
    },
    {
      "kind": "event",
      "id": "shoot-3",
      "name": "Shoot",
      "durationMs": 1500
    },
    {
      "kind": "path",
      "lineId": "l6"
    },
    {
      "kind": "path",
      "lineId": "l7"
    },
    {
      "kind": "event",
      "id": "shoot-4",
      "name": "Shoot",
      "durationMs": 1500
    },
    {
      "kind": "path",
      "lineId": "l8"
    }
  ],
  "shapes": [],
  "variables": [],
  "settings": {
    "rWidth": 18,
    "rHeight": 15,
    "maxVelocity": 52,
    "maxAcceleration": 30,
    "maxDeceleration": 197.1,
    "aVelocity": 3.141592653589793,
    "safetyMargin": 1,
    "fieldMap": "biobuzz.webp",
    "autoMidline": "red"
  },
  "version": "1.3.0"
}