
from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.byd.bydcan import create_steer_command

SteerControlType = structs.CarParams.SteerControlType


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.apply_angle_last = 0
    self.lat_active_last = False

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    lat_active = CC.latActive and not CS.out.standstill

    # LKASPrepare: send 1 for one frame on rising edge of lat_active
    lkas_prepare = lat_active and not self.lat_active_last

    new_actuators = actuators.as_builder()
    # Pure forwarding mode - no sending
    # if self.CP.steerControlType == SteerControlType.angle:
    #   if self.frame % 2 == 0:
    #     apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
    #                                               CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)
    #
    #     can_sends.append(create_steer_command(self.packer, apply_angle, lat_active, lkas_prepare))
    #
    #     self.apply_angle_last = apply_angle
    #
    #   new_actuators.steeringAngleDeg = self.apply_angle_last

    self.lat_active_last = lat_active
    self.frame += 1
    return new_actuators, can_sends
