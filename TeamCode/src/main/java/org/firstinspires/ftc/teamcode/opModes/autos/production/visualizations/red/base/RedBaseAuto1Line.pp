{
  "startPoint": {
    "x": 89.4,
    "y": 8.9,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 0,
    "locked": false
  },
  "lines": [
    {
      "id": "redbaseauto1line-path-01",
      "name": "Path 1",
      "endPoint": {
        "x": 89.4,
        "y": 35.32398753894079,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 0
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
      "id": "redbaseauto1line-path-02",
      "name": "Path 2",
      "endPoint": {
        "x": 130,
        "y": 35.149,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#dc2626",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "redbaseauto1line-path-03",
      "name": "Path 3",
      "endPoint": {
        "x": 91.60404984423676,
        "y": 11.985669781931465,
        "heading": "linear",
        "reverse": false,
        "degrees": 180,
        "startDeg": 0,
        "endDeg": 60
      },
      "controlPoints": [],
      "color": "#dc2626",
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
      "lineId": "line-cxr9c2yvy19"
    },
    {
      "kind": "path",
      "lineId": "mrhu4dps-uu3ycy"
    },
    {
      "kind": "path",
      "lineId": "mrhu6jlp-u28dvz"
    }
  ],
  "pathChains": [
    {
      "id": "chain-mrhtv8d0-60pmck",
      "name": "Main Chain",
      "color": "#6A67BD",
      "lineIds": [
        "line-cxr9c2yvy19",
        "mrhu4dps-uu3ycy",
        "mrhu6jlp-u28dvz"
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
  "timestamp": "2026-07-12T13:37:26.479Z"
}
