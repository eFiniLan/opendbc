from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, GEAR_MAP

GearShifter = structs.CarState.GearShifter

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    # can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam.pt]

    ret = structs.CarState()

    ## Required for basic lateral control
    # * `brakePressed`
    ret.brake = cp.vl["PEDAL"]['BRAKE_PEDAL']
    ret.brakePressed = ret.brake > 0.01

    # * `cruiseState`
    ret.cruiseState.available = cp_cam.vl["ACC_HUD_ADAS"]["ACC_ON2"]
    ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SET_SPEED"]
    #ret.cruiseState.enabled =

    # * `doorOpen`
    ret.doorOpen = any([cp.vl["METER_CLUSTER"]['FRONT_LEFT_DOOR'],
                       cp.vl["METER_CLUSTER"]['FRONT_RIGHT_DOOR'],
                       cp.vl["METER_CLUSTER"]['REAR_LEFT_DOOR'],
                       cp.vl["METER_CLUSTER"]['REAR_RIGHT_DOOR']])

    # * `espDisabled`
    # @todo
    ret.espDisabled = False

    # * `gasPressed`
    ret.gas = cp.vl["PEDAL"]['GAS_PEDAL']
    ret.gasPressed = ret.gas >= 0.01

    # * `gearShifter`
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["PEDAL"]["GEAR"]), GearShifter.unknown)

    # * `leftBlinker` / `rightBlinker`
    ret.leftBlinker = bool(cp.vl["STALKS"]["TURN_SIGNAL_SWITCH"] in (2, 3))
    ret.rightBlinker = bool(cp.vl["STALKS"]["TURN_SIGNAL_SWITCH"] in (4, 5))

    # * `seatbeltUnlatched`
    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]['SEATBELT_DRIVER'] == 0
    # * `standstill`
    ret.standstill = ret.vEgoRaw < 0.01
    # * `steeringAngleDeg`
    ret.steeringAngleDeg = cp.vl["STEERING_TORQUE"]['ANGLE']
    # * `steeringPressed`
    ret.steeringPressed = cp.vl["STEER_MODULE_2"]['DRIVER_EPS_TORQUE'] > 6.
    # * `steeringTorque`
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE']
    # * `steerFaultPermanent`
    # @todo
    ret.steerFaultPermanent = False
    # * `steerFaultTemporary`
    # @todo
    ret.steerFaultTemporary = False

    ########################################### * `vCruise`

    # * `wheelSpeeds.[fl|fr|rl|rr]`
    ret.wheelSpeeds = self.parse_wheel_speeds(
      ret,
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_FL'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_FR'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_RL'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_RR'],
    )


    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
