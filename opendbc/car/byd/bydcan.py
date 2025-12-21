
def create_steer_command(packer, steer_angle, steer_req):

  set_me_xe = 0xE if not steer_req else 0xB

  values = {
    "STEER_REQ": steer_req,
    # to recover from ecu fault, it must be momentarily pulled low.
    "EPS_OK": True,
    "STEER_ANGLE": steer_angle,
    # must be 0x1 to steer
    "SET_ME_X01": 0x1 if steer_req else 0,
    # 0xB fault lesser, maybe higher value fault lesser, 0xB also seem to have the highest angle limit at high speed.
    "SET_ME_XE": set_me_xe if steer_req else 0,
    "SET_ME_FF": 0xFF,
    "SET_ME_F": 0xF,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
    "UNKNOWN": 2773 if steer_req else 0,
    }
  return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)
