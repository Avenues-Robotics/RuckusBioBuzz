{
  "startPoint": {
    "x": 85.5,
    "y": 132.304,
    "heading": "constant",
    "degrees": 0,
    "locked": false
  },
  "lines": [
    {
      "id": "l1",
      "name": "Start to corner",
      "endPoint": {
        "x": 131.383,
        "y": 131.96,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 0,
        "headingCurve": 1
      },
      "controlPoints": [],
      "speed": 1,
      "stopAtEnd": true,
      "eventMarkers": []
    },
    {
      "id": "l2",
      "name": "Corner loop, back",
      "endPoint": {
        "x": 131.278,
        "y": 131.201,
        "heading": "linear",
        "startDeg": 45,
        "endDeg": 90,
        "headingCurve": 1
      },
      "controlPoints": [
        {
          "x": 126.139,
          "y": 119.38
        },
        {
          "x": 130.296,
          "y": 123.952
        }
      ],
      "speed": 1,
      "eventMarkers": []
    },
    {
      "id": "l3",
      "name": "Up the field to shoot",
      "endPoint": {
        "x": 95.02000000000001,
        "y": 15.643,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 180,
        "headingCurve": 1.0
      },
      "controlPoints": [
        {
          "x": 120.713,
          "y": 99.619
        },
        {
          "x": 122.511,
          "y": 44.825
        },
        {
          "x": 92.77600000000001,
          "y": 30.070999999999998
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
        "x": 82.688,
        "y": 25.933999999999997,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 90,
        "headingCurve": 1.35
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
        "x": 83.557,
        "y": 131.081,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 90,
        "headingCurve": 1.0
      },
      "controlPoints": [
        {
          "x": 79.536,
          "y": 85.491
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
        "alliance": "BLUE",
        "pickY": 82.5,
        "wallY": 132.5,
        "xMin": 60.5,
        "xMax": 133,
        "previewX": 83.557,
        "hfovDeg": 43,
        "cameraForward": 9
      }
    },
    {
      "id": "l6",
      "name": "To the flower",
      "endPoint": {
        "x": 126.05799999999999,
        "y": 94.285,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 0,
        "headingCurve": 1.0
      },
      "controlPoints": [
        {
          "x": 91.731,
          "y": 99.203
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
        "x": 96.19200000000001,
        "y": 129.783,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 0,
        "headingCurve": 1
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
        "x": 131.034,
        "y": 40.343,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 0,
        "headingCurve": 1
      },
      "controlPoints": [
        {
          "x": 130.187,
          "y": 86.662
        }
      ],
      "speed": 1,
      "eventMarkers": []
    },
    {
      "id": "l2a",
      "name": "Corner loop, out",
      "endPoint": {
        "x": 124.345,
        "y": 119.17,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 45,
        "headingCurve": 1
      },
      "controlPoints": [
        {
          "x": 123.121,
          "y": 123.116
        },
        {
          "x": 122.551,
          "y": 118.961
        }
      ],
      "speed": 1,
      "eventMarkers": [
        {
          "id": "ev-intake-corner",
          "name": "Intake On",
          "triggerType": "parametric",
          "position": 0,
          "triggerMs": 0,
          "durationMs": 0
        }
      ]
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
      "lineId": "l2a"
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
    "autoMidline": "blue"
  },
  "version": "1.3.0"
}