{
  "startPoint": {
    "x": 112.824,
    "y": 132.122,
    "heading": "linear",
    "startDeg": 270,
    "endDeg": 43,
    "locked": false
  },
  "lines": [
    {
      "id": "redgoalauto1line-path-01",
      "name": "Path 1",
      "endPoint": {
        "x": 101.462,
        "y": 98.08,
        "heading": "linear",
        "startDeg": 270,
        "endDeg": 43
      },
      "controlPoints": [],
      "color": "#dc2626",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "redgoalauto1line-path-02",
      "name": "Path 2",
      "endPoint": {
        "x": 103.907,
        "y": 82.5,
        "heading": "linear",
        "startDeg": 43,
        "endDeg": 0,
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 95.97,
          "y": 83.032
        }
      ],
      "color": "#dc2626",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "redgoalauto1line-path-03",
      "name": "Path 3",
      "endPoint": {
        "x": 123.0,
        "y": 82.5,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 0,
        "reverse": false
      },
      "controlPoints": [],
      "color": "#dc2626",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "redgoalauto1line-path-04",
      "name": "Path 4",
      "endPoint": {
        "x": 88.6,
        "y": 82.5,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 43,
        "reverse": false
      },
      "controlPoints": [],
      "color": "#dc2626",
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
      "lineId": "redgoalauto1line-path-01"
    },
    {
      "kind": "path",
      "lineId": "redgoalauto1line-path-02"
    },
    {
      "kind": "path",
      "lineId": "redgoalauto1line-path-03"
    },
    {
      "kind": "path",
      "lineId": "redgoalauto1line-path-04"
    }
  ],
  "pathChains": [
    {
      "id": "chain-redgoalauto1line",
      "name": "Main Chain",
      "color": "#dc2626",
      "lineIds": [
        "redgoalauto1line-path-01",
        "redgoalauto1line-path-02",
        "redgoalauto1line-path-03",
        "redgoalauto1line-path-04"
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
  "timestamp": "2026-07-12T13:54:27.942Z"
}
