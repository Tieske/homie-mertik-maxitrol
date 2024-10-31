# homie-mertik-maxitrol
Homie interface to a Mertik Maxitrol controlled fireplace using an ESP32/ESP8266 device

## Get going

copy `mertik-maxitrol.h.template` to `mertik-maxitrol.h` and modify it with
the appropiate values for the secrets.

Read the instructions in `mertik-maxitrol.ino`.

## Homie Device

### Node: pilot

#### Property: value

- type: bool
- format: "off,on"
- settable: true
- retained: true

### Node:
#### Property: level

- type: float
- format: "0:100:1"
- settable: true
- retained: false

{
  "name": "Smart Fireplace",
  "homie": "5.0",
  "nodes": {
    "pilot": {
      "name": "Pilot control",
      "type": "switch",
      "properties": {
        "control": {
          "name": "Pilot flame control",
          "datatype": "boolean",
          "format": "off,on",
          "settable": true,
          "retained": true
        },
        "status": {
          "name": "Pilot flame status",
          "datatype": "enum",
          "format": "off,igniting,on,extinguishing",
          "settable": false,
          "retained": true
        }
      }
    },
    "flames": {
      "properties": {
        "value": {
          "name": "Flame Level",
          "datatype": "float",
          "format": "0:100:1",
          "settable": true,
          "retained": true,
          "unit": "%"
        }
      }
    }
  }
}
