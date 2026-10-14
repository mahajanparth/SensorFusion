"""Shared bag-reading helpers (anaconda python, `rosbags`)."""
import sys, types

# rosbags imports lz4 for mcap/rosbag1 even though sqlite3 bags don't need it; stub it if missing.
try:
    import lz4.frame  # noqa: F401
except ImportError:
    lz4 = types.ModuleType('lz4'); lz4.frame = types.ModuleType('lz4.frame')
    lz4.frame.decompress = None
    sys.modules['lz4'] = lz4; sys.modules['lz4.frame'] = lz4.frame

from rosbags.rosbag2 import Reader  # noqa: E402
from rosbags.typesys import Stores, get_typestore, get_types_from_msg  # noqa: E402

BAG = '/home/parth/Downloads/linkou/linkou-2023-12-27-2-med'

NAVRELPOSNED9 = """
uint8 version
uint8 reserved0
uint16 ref_station_id
uint32 i_tow
int32 rel_pos_n
int32 rel_pos_e
int32 rel_pos_d
int32 rel_pos_length
int32 rel_pos_heading
uint8[4] reserved1
int8 rel_pos_hpn
int8 rel_pos_hpe
int8 rel_pos_hpd
int8 rel_pos_hp_length
uint32 acc_n
uint32 acc_e
uint32 acc_d
uint32 acc_length
uint32 acc_heading
uint8[4] reserved2
uint32 flags
"""


def typestore():
    ts = get_typestore(Stores.ROS2_FOXY)
    ts.register(get_types_from_msg(NAVRELPOSNED9, 'ublox_msgs/msg/NavRELPOSNED9'))
    return ts
