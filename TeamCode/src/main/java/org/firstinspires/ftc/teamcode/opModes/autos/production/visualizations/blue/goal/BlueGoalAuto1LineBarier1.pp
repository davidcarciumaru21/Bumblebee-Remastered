{
  "startPoint": {
    "x": 31.176,
    "y": 132.122,
    "heading": "linear",
    "startDeg": 270,
    "endDeg": 137,
    "locked": false
  },
  "lines": [
    {
      "id": "bluegoalauto1linebarier1-path-01",
      "name": "Path 1",
      "endPoint": {
        "x": 42.538,
        "y": 98.08,
        "heading": "linear",
        "startDeg": 270,
        "endDeg": 137
      },
      "controlPoints": [],
      "color": "#9CBD95",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "bluegoalauto1linebarier1-path-02",
      "name": "Path 2",
      "endPoint": {
        "x": 40.093,
        "y": 82.5,
        "heading": "linear",
        "startDeg": 137,
        "endDeg": 180,
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 48.03,
          "y": 83.032
        }
      ],
      "color": "#9CBD95",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "bluegoalauto1linebarier1-path-03",
      "name": "Path 3",
      "endPoint": {
        "x": 21.0,
        "y": 82.5,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 180,
        "reverse": false
      },
      "controlPoints": [],
      "color": "#9CBD95",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "bluegoalauto1linebarier1-path-04",
      "name": "Path 4",
      "endPoint": {
        "x": 16.951,
        "y": 70.363,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 90,
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 22.73,
          "y": 68.411
        }
      ],
      "color": "#9CBD95",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "bluegoalauto1linebarier1-path-05",
      "name": "Path 5",
      "endPoint": {
        "x": 55.4,
        "y": 82.5,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 137,
        "reverse": false
      },
      "controlPoints": [],
      "color": "#9CBD95",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 141.5,
          "y": 70
        },
        {
          "x": 141.5,
          "y": 141.5
        },
        {
          "x": 120,
          "y": 141.5
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "bluegoalauto1linebarier1-path-01"
    },
    {
      "kind": "path",
      "lineId": "bluegoalauto1linebarier1-path-02"
    },
    {
      "kind": "path",
      "lineId": "bluegoalauto1linebarier1-path-03"
    },
    {
      "kind": "path",
      "lineId": "bluegoalauto1linebarier1-path-04"
    },
    {
      "kind": "path",
      "lineId": "bluegoalauto1linebarier1-path-05"
    }
  ],
  "pathChains": [
    {
      "id": "chain-bluegoalauto1linebarier1",
      "name": "Main Chain",
      "color": "#9CBD95",
      "lineIds": [
        "bluegoalauto1linebarier1-path-01",
        "bluegoalauto1linebarier1-path-02",
        "bluegoalauto1linebarier1-path-03",
        "bluegoalauto1linebarier1-path-04",
        "bluegoalauto1linebarier1-path-05"
      ]
    }
  ],
  "settings": {
    "xVelocity": 30,
    "yVelocity": 30,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.4,
    "rWidth": 17.2,
    "rHeight": 14.33,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": true,
    "onionLayerSpacing": 6,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false,
    "showHeadingArrow": false,
    "headingArrowLength": 50,
    "headingArrowColor": "#ffffff",
    "headingArrowThickness": 2,
    "pathOpacity": 1
  },
  "version": "1.2.1",
  "timestamp": "2026-07-12T11:52:49.619Z"
}
