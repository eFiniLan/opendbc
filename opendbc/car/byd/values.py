from dataclasses import dataclass, field

from opendbc.car.lateral import AngleSteeringLimits, ISO_LATERAL_ACCEL
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, structs
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.structs import CarParams
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

GEAR_MAP = {
  0: structs.CarState.GearShifter.unknown,
  1: structs.CarState.GearShifter.park,
  2: structs.CarState.GearShifter.reverse,
  4: structs.CarState.GearShifter.drive,
}

# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll lowers lateral acceleration

class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    220, # deg ECU_FAULT_MAX_ANGLES([0., 1., 8.], [360, 290, 220])
    ([0., 5., 15.], [4., 3., 2.]),
    ([0., 5., 15.], [6., 4., 3.]),
  )


@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.byd]))


@dataclass
class BydDMOPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'byd_dmo_platform'})


class CAR(Platforms):
  BYD_SHARK_6_PHEV = BydDMOPlatformConfig(
    [BydCarDocs("BYD Shark 6 PHEV 2025")],
    CarSpecs(mass=2710., wheelbase=3.26, steerRatio=15.2),
  )

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
      logging=True,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.SUPPLIER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.SUPPLIER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
      logging=True,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_ECU_HARDWARE_NUMBER_RESPONSE],
      bus=0,
      logging=True,
    ),
  ],
)

DBC = CAR.create_dbc_map()
