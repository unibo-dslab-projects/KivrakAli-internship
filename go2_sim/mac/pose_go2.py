import time, sys
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# Joint order (per leg): [hip, thigh, calf] | legs: FR, FL, RR, RL | radians
POSES = {
    "stand": [ 0.0, 0.61, -1.22,  0.0, 0.61, -1.22,  0.0, 0.61, -1.22,  0.0, 0.61, -1.22],
    "low":   [ 0.0, 1.00, -2.00,  0.0, 1.00, -2.00,  0.0, 1.00, -2.00,  0.0, 1.00, -2.00],
    "high":  [ 0.0, 0.35, -0.95,  0.0, 0.35, -0.95,  0.0, 0.35, -0.95,  0.0, 0.35, -0.95],
    "sit":   [ 0.0, 0.45, -0.90,  0.0, 0.45, -0.90,  0.0, 1.40, -2.50,  0.0, 1.40, -2.50],
    "bow":   [ 0.0, 1.20, -2.30,  0.0, 1.20, -2.30,  0.0, 0.40, -1.00,  0.0, 0.40, -1.00],
    "splay": [ 0.25, 0.61, -1.22, -0.25, 0.61, -1.22,  0.25, 0.61, -1.22, -0.25, 0.61, -1.22],
    "paw":   [ 0.0, 0.00, -0.90,  0.0, 0.80, -1.55,  0.0, 0.95, -1.80,  0.0, 0.95, -1.80],
}

DURATION = 2.0     # transition time to target (s)
KP, KD = 60.0, 4.0
dt = 0.002

name = sys.argv[1] if len(sys.argv) > 1 else "stand"
if name not in POSES:
    print("No such pose:", name, "| options:", ", ".join(POSES))
    sys.exit(1)
target = np.array(POSES[name], dtype=float)

low_state = None
def OnState(msg):
    global low_state
    low_state = msg

if __name__ == "__main__":
    ChannelFactoryInitialize(1, "lo0")
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(OnState, 10)
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    print("Waiting for robot state...")
    while low_state is None:
        time.sleep(0.01)
    start = np.array([low_state.motor_state[i].q for i in range(12)], dtype=float)
    print(f"Moving to '{name}' pose (press Ctrl+C to exit)")

    crc = CRC()
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE; cmd.head[1] = 0xEF; cmd.level_flag = 0xFF; cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].kd = 0.0

    t = 0.0
    while True:
        s = time.perf_counter()
        t += dt
        ph = min(t / DURATION, 1.0)
        for i in range(12):
            cmd.motor_cmd[i].q = (1 - ph) * start[i] + ph * target[i]
            cmd.motor_cmd[i].kp = KP
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = KD
            cmd.motor_cmd[i].tau = 0.0
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)
        rem = dt - (time.perf_counter() - s)
        if rem > 0:
            time.sleep(rem)
