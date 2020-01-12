/*eslint-disable block-scoped-var, id-length, no-control-regex, no-magic-numbers, no-prototype-builtins, no-redeclare, no-shadow, no-var, sort-vars*/
"use strict";

var $protobuf = require("protobufjs/light");

var $root = ($protobuf.roots["default"] || ($protobuf.roots["default"] = new $protobuf.Root()))
.addJSON({
  NautilusVisionConfig: {
    nested: {
      SerialPort: {
        fields: {
          portName: {
            type: "string",
            id: 1
          },
          baudrate: {
            type: "int32",
            id: 2
          },
          parity: {
            type: "Parity",
            id: 3
          },
          dataBit: {
            type: "int32",
            id: 4
          },
          stopBit: {
            type: "StopBit",
            id: 5
          },
          synchronize: {
            type: "bool",
            id: 6
          },
          syncInterval: {
            type: "int32",
            id: 7
          }
        },
        nested: {
          Parity: {
            values: {
              NoParity: 0,
              OddParity: 1,
              EvenParity: 2,
              MarkParity: 3
            }
          },
          StopBit: {
            values: {
              OneStopBit: 0,
              One5StopBits: 1,
              TwoStopBits: 2
            }
          }
        }
      },
      Camera: {
        fields: {
          deviceIndex: {
            type: "int32",
            id: 1
          },
          frameWidth: {
            type: "int32",
            id: 2
          },
          frameHeight: {
            type: "int32",
            id: 3
          }
        }
      },
      TestVideoSource: {
        fields: {
          filename: {
            type: "string",
            id: 1
          }
        }
      },
      Aimbot: {
        fields: {
          maxShootRadius: {
            type: "float",
            id: 1
          },
          enemyColor: {
            type: "EnemyColor",
            id: 2
          },
          brightness: {
            type: "int32",
            id: 3
          },
          thresColor: {
            type: "int32",
            id: 4
          },
          lightBarMinArea: {
            type: "float",
            id: 5
          },
          lightBarContourMinSolidity: {
            type: "float",
            id: 6
          },
          lightBarEllipseMinAspectRatio: {
            type: "float",
            id: 7
          },
          armorMaxAngleDiff: {
            type: "float",
            id: 8
          },
          armorMaxHeightDiffRatio: {
            type: "float",
            id: 9
          },
          armorMaxYDiffRatio: {
            type: "float",
            id: 10
          },
          armorMinXDiffRatio: {
            type: "float",
            id: 11
          },
          armorBigArmorRatio: {
            type: "float",
            id: 12
          },
          armorSmallArmorRatio: {
            type: "float",
            id: 13
          },
          armorMinAspectRatio: {
            type: "float",
            id: 14
          },
          armorMaxAspectRatio: {
            type: "float",
            id: 15
          },
          areaNormalizedBase: {
            type: "float",
            id: 16
          },
          sightOffsetNormalizedBase: {
            type: "float",
            id: 17
          }
        },
        nested: {
          EnemyColor: {
            values: {
              Red: 0,
              Blue: 1
            }
          }
        }
      },
      PoseSolver: {
        fields: {
          offsetX: {
            type: "float",
            id: 1
          },
          offsetZPitch: {
            type: "float",
            id: 2
          },
          offsetYPitch: {
            type: "float",
            id: 3
          },
          offsetYYaw: {
            type: "float",
            id: 4
          },
          offsetZYaw: {
            type: "float",
            id: 5
          },
          offsetYaw: {
            type: "float",
            id: 6
          },
          offsetPitch: {
            type: "float",
            id: 7
          },
          initV: {
            type: "float",
            id: 8
          },
          initK: {
            type: "float",
            id: 9
          },
          gravity: {
            type: "float",
            id: 10
          }
        }
      },
      Configuration: {
        fields: {
          serialPort: {
            type: "SerialPort",
            id: 1
          },
          camera: {
            type: "Camera",
            id: 2
          },
          testVideoSource: {
            type: "TestVideoSource",
            id: 3
          },
          aimbot: {
            type: "Aimbot",
            id: 4
          },
          poseSolver: {
            type: "PoseSolver",
            id: 5
          }
        }
      }
    }
  }
});

module.exports = $root;
