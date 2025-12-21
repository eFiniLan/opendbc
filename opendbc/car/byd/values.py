from dataclasses import dataclass, field

from opendbc.car.lateral import AngleSteeringLimits, ISO_LATERAL_ACCEL
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, structs
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts

GEAR_MAP = {
  0: structs.CarState.GearShifter.unknown,
  1: structs.CarState.GearShifter.park,
  2: structs.CarState.GearShifter.reverse,
  4: structs.CarState.GearShifter.drive,
}

# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll lowers lateral acceleration

class CarControllerParams:
  STEER_STEP = 2  # Angle command is sent at 50 Hz
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    # EPAS faults above this angle
    50,  # deg
    ([], []),
    ([], []),
  )


@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.byd]))


@dataclass
class BydDMOSuperHybridPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'byd_general_pt'})


class CAR(Platforms):
  SHARK_6_PHEV = BydDMOSuperHybridPlatformConfig(
    [BydCarDocs("BYD Shark 6 PHEV 2025")],
    CarSpecs(mass=2710., wheelbase=3.26, steerRatio=15.2),
  )


DBC = CAR.create_dbc_map()
