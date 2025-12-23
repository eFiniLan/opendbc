from opendbc.can import CANParser
from opendbc.car import Bus, structs, create_button_events
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, GEAR_MAP

GearShifter = structs.CarState.GearShifter
ButtonType = structs.CarState.ButtonEvent.Type

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    self._btn_set_prev = 0
    self._btn_res_prev = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()

    ret.doorOpen = any([cp.vl["METER_CLUSTER"]['FRONT_LEFT_DOOR'],
                       cp.vl["METER_CLUSTER"]['FRONT_RIGHT_DOOR'],
                       cp.vl["METER_CLUSTER"]['REAR_LEFT_DOOR'],
                       cp.vl["METER_CLUSTER"]['REAR_RIGHT_DOOR']])

    # @todo
    ret.espDisabled = False

    ret.gasPressed = cp.vl["PEDAL"]['AcceleratorPedal'] >= 0.01

    ret.brake = cp.vl["PEDAL"]['BrakePedal']
    ret.brakePressed = ret.brake > 0.01

    ret.gearShifter = GEAR_MAP.get(int(cp.vl["PEDAL"]["Gear"]), GearShifter.unknown)

    ret.leftBlinker = bool(cp.vl["STALKS"]["TURN_SIGNAL_SWITCH"] in (2, 3))
    ret.rightBlinker = bool(cp.vl["STALKS"]["TURN_SIGNAL_SWITCH"] in (4, 5))

    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]['SEATBELT_DRIVER'] == 0
    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringAngleDeg = cp.vl["STEERING_TORQUE"]['ANGLE']
    ret.steeringPressed = cp.vl["STEER_MODULE"]['STEERING_RATE'] > 6.
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE']
    # @todo
    ret.steerFaultPermanent = False
    ret.steerFaultTemporary = cp.vl["STEERING_TORQUE"]["TORQUE_TEMP_FAILED"] == 1

    # vEgo, vEgoRaw
    self.parse_wheel_speeds(
      ret,
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_FL'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_FR'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_RL'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_RR'],
    )

    ret.parkingBrake = cp.vl["EPB_NG"]["EPB_STATUS"] != 1

    # PCM signals
    if not self.CP.openpilotLongitudinalControl:
      ret.cruiseState.available = cp_cam.vl["ACC_HUD_ADAS"]["ACC_ON2"] == 1
      ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SET_SPEED"]
      ret.cruiseState.enabled = cp_cam.vl["ACC_HUD_ADAS"]["CRUISE_STATE"] == 3

      # rick:
      # seen this when ACC enabled
      btn_set = cp.vl["PCM_BUTTONS"]["SET_BTN"]
      # unseen
      btn_res = cp.vl["PCM_BUTTONS"]["RES_BTN"]
      # increase set speed (always +5)
      # btn_acc_speed_inc = cp.vl["PCM_BUTTONS"]["BTN_ACC_DEC"] == 3
      # decrease set speed (always -5)
      # btn_acc_speed_dec = cp.vl["PCM_BUTTONS"]["BTN_ACC_DEC"] == 1

      ret.buttonEvents = [
        *create_button_events(btn_set, self._btn_set_prev, {1: ButtonType.setCruise}),
        *create_button_events(btn_res, self._btn_res_prev, {1: ButtonType.resumeCruise}),
      ]

      self._btn_set_prev = btn_set
      self._btn_res_prev = btn_res

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
