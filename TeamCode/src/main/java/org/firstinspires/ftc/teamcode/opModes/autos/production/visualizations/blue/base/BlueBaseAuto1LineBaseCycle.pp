{
  "startPoint": {
    "x": 54.6,
    "y": 8.9,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-9kb926utysd",
      "name": "Path 1",
      "endPoint": {
        "x": 54.6,
        "y": 35.32398753894079,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#B6CDAA",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mrjk5rxl-d6f82g",
      "name": "Path 2",
      "endPoint": {
        "x": 14,
        "y": 35.149,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#B6CDAA",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mrjk766d-5vw4ys",
      "name": "Path 3",
      "endPoint": {
        "x": 52.395950155763245,
        "y": 13.985669781931465,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 120
      },
      "controlPoints": [],
      "color": "#B6CDAA",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mrkw11h2-7h1h8x",
      "name": "Path 4",
      "endPoint": {
        "x": 8.584230097901107,
        "y": 8.511736565620435,
        "heading": "linear",
        "reverse": false,
        "startDeg": 120,
        "endDeg": 180
      },
      "controlPoints": [
        {
          "x": 44.91015169194866,
          "y": 10.897316219369895
        }
      ],
      "color": "#B6CDAA",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mrkw1b7n-8krw7b",
      "name": "Path 5",
      "endPoint": {
        "x": 51.946432226092426,
        "y": 13.714458892448398,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 120
      },
      "controlPoints": [],
      "color": "#B6CDAA",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mrkw1zlf-402gew",
      "name": "Path 6",
      "endPoint": {
        "x": 7.8649555060355185,
        "y": 27.963659952504113,
        "heading": "linear",
        "reverse": false,
        "startDeg": 120,
        "endDeg": 180
      },
      "controlPoints": [
        {
          "x": 44.68311510293678,
          "y": 28.306387310457577
        }
      ],
      "color": "#B6CDAA",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mrkw31xt-fp1irh",
      "name": "Path 7",
      "endPoint": {
        "x": 52.14488358513371,
        "y": 13.636139089611241,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 120
      },
      "controlPoints": [],
      "color": "#B6CDAA",
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
      "lineId": "line-9kb926utysd"
    },
    {
      "kind": "path",
      "lineId": "mrjk5rxl-d6f82g"
    },
    {
      "kind": "path",
      "lineId": "mrjk766d-5vw4ys"
    },
    {
      "kind": "path",
      "lineId": "mrkw11h2-7h1h8x"
    },
    {
      "kind": "path",
      "lineId": "mrkw1b7n-8krw7b"
    },
    {
      "kind": "path",
      "lineId": "mrkw1zlf-402gew"
    },
    {
      "kind": "path",
      "lineId": "mrkw31xt-fp1irh"
    }
  ],
  "pathChains": [
    {
      "id": "chain-mrjk495m-bb0hae",
      "name": "Main Chain",
      "color": "#B6CDAA",
      "lineIds": [
        "line-9kb926utysd",
        "mrjk5rxl-d6f82g",
        "mrjk766d-5vw4ys",
        "mrkw11h2-7h1h8x",
        "mrkw1b7n-8krw7b",
        "mrkw1zlf-402gew",
        "mrkw31xt-fp1irh"
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
  "timestamp": "2026-07-14T17:40:05.605Z"
}