from typing import List, Union

def create_steer_command(packer, steer_angle, steer_req):

  values = {
    "JerkUpperLimit": 300,
    "JerkLowerLimit": -300,
    "LKASPrepare": 0,
    "LKAS_ACTIVE": 1 if steer_req else 0,
    "SET_ME_3": 3,
    "LKAS_Output": steer_angle if steer_req else 0,
    "SET_ME_FF": 0xff,
    "SET_ME_F": 0xf,
    }

  return packer.make_can_msg("MPC_LKAS_CMD_ANGLE", 0, values)

def byd_checksum(address: int, sig, d: bytearray) -> int:
    # According to the implementation logic, the key is fixed as 0xAF
    byte_key = 0xAF

    # According to DBC, the checksum is at the last byte (index 7).
    # We slice the first 7 bytes for calculation.
    dat = d[:7]

    # Sum up the high nibbles and low nibbles of all data bytes separately
    first_bytes_sum = sum(byte >> 4 for byte in dat)
    second_bytes_sum = sum(byte & 0xF for byte in dat)

    # Extract the carry (remainder) from the low nibble sum
    remainder = second_bytes_sum >> 4

    # Mix the byte_key into the sums (Cross-mixing high/low nibbles)
    second_bytes_sum += byte_key >> 4
    first_bytes_sum += byte_key & 0xF

    # Perform complement and offset transformation: (-sum + 9) & 0xF
    first_part = ((-first_bytes_sum + 0x9) & 0xF)
    second_part = ((-second_bytes_sum + 0x9) & 0xF)

    # Combine the parts back into an 8-bit value
    # High nibble includes the remainder correction and a constant offset of 5
    return (((first_part + (-remainder + 5)) << 4) + second_part) & 0xFF