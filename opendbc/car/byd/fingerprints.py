""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.byd.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.BYD_SHARK_6_PHEV: {
    (Ecu.fwdRadar, 0x6b6, None): [
      b'212053276',
    ],
  },
}
