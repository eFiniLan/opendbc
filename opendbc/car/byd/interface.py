
from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "byd"

    ret.dashcamOnly = False

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = False

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True
    ret.pcmCruise = True


    ret.alphaLongitudinalAvailable = False

    return ret
