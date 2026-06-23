import time, sys
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

POSES = {
    "stand": [ 0.0, 0.61, -1.22,  0.0, 0.61, -1.22,  0.0, 0.61, -1.22,  0.0, 0.61, -1.22],
    "low":   [ 0.0, 1.00, -2.00,  0.0, 1.00, -2.00,  0.0, 1.00, -2.00,  0.0, 1.00, -2.00],
    "high":  [ 0.0, 0.35, -0.95,  0.0, 0.35, -0.95,  0.0, 0.35, -0.95,  0.0, 0.35, -0.95],
    "sit":   [ 0.0, 0.50, -1.05,  0.0, 0.50, -1.05,  0.0, 1.05, -2.10,  0.0, 1.05, -2.10],
    "bow":   [ 0.0, 1.05, -2.05,  0.0, 1.05, -2.05,  0.0, 0.45, -1.05,  0.0, 0.45, -1.05],
    "splay": [ 0.20, 0.61, -1.22, -0.20, 0.61, -1.22,  0.20, 0.61, -1.22, -0.20, 0.61, -1.22],
    "paw":   [ 0.0, 0.10, -1.00,  0.0, 0.85, -1.60,  0.0, 1.00, -1.85,  0.0, 1.00, -1.85],
}

# (pose, transition_time_sec, waiting_time_sec)
SEQUENCE = [
    ("stand", 2.0, 2.0), ("low", 2.0, 1.5), ("high", 2.0, 1.5), ("stand", 2.0, 1.5),
    ("sit", 3.0, 3.0), ("low", 2.5, 0.8), ("stand", 2.5, 1.5),
    ("bow", 3.0, 3.0), ("low", 2.5, 0.8), ("stand", 2.5, 2.0),
]

KP, KD = 60.0, 4.0
dt = 0.002
loop = "--loop" in sys.argv

low_state = None
def OnState(msg):
    global low_state
    low_state = msg

def make_cmd():
    crc = CRC()
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE; cmd.head[1] = 0xEF; cmd.level_flag = 0xFF; cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].kd = 0.0
    return cmd, crc

def goto(pub, cmd, crc, start, target, transition, hold):
    steps = int(transition / dt)
    for s in range(steps + 1):
        t0 = time.perf_counter()
        lin = s / steps if steps else 1.0
        ph = lin * lin * (3 - 2 * lin) #smooth_step
        for i in range(12):
            cmd.motor_cmd[i].q = (1 - ph) * start[i] + ph * target[i]
            cmd.motor_cmd[i].kp = KP; cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = KD; cmd.motor_cmd[i].tau = 0.0
        cmd.crc = crc.Crc(cmd); pub.Write(cmd)
        rem = dt - (time.perf_counter() - t0)
        if rem > 0: time.sleep(rem)
    for _ in range(int(hold / dt)):
        t0 = time.perf_counter()
        cmd.crc = crc.Crc(cmd); pub.Write(cmd)
        rem = dt - (time.perf_counter() - t0)
        if rem > 0: time.sleep(rem)

if __name__ == "__main__":
    ChannelFactoryInitialize(1, "lo0")
    ChannelSubscriber("rt/lowstate", LowState_).Init(OnState, 10)
    pub = ChannelPublisher("rt/lowcmd", LowCmd_); pub.Init()
    print("Waiting robot state...")
    while low_state is None:
        time.sleep(0.01)
    start = np.array([low_state.motor_state[i].q for i in range(12)], dtype=float)
    cmd, crc = make_cmd()
    print("=== Go2 pose demo (Ctrl+C for exit) ===")
    while True:
        for name, trans, hold in SEQUENCE:
            target = np.array(POSES[name], dtype=float)
            print(f"  -> {name}")
            goto(pub, cmd, crc, start, target, trans, hold)
            start = target
        if not loop:
            break
    print("=== Demo finished successfully ===")
