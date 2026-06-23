import sys, time, select, termios, tty
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

# --- tuning ---
KP, KD = 50.0, 3.5
dt = 0.002
STEP = 0.04                 # per-keypress increment
LIMIT = 0.35                # clamp for height/roll/pitch
SMOOTH = 0.02               # target tracking smoothness (0..1 per step)
C_MIN, C_MAX = -0.4, 1.3    # per-leg crouch clamp
BASE_THIGH, BASE_CALF = 0.61, -1.22

# leg layout (FR, FL, RR, RL): is_front, is_right
LEG_FRONT = [True, True, False, False]
LEG_RIGHT = [True, False, True, False]

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def compute_targets(H, P, R):
    """Map body height/pitch/roll to 12 joint targets; feet stay planted."""
    q = np.zeros(12)
    for leg in range(4):
        c = -H
        c += (-P if LEG_FRONT[leg] else P)    # P>0 -> nose up (front legs longer)
        c += (R if LEG_RIGHT[leg] else -R)    # R>0 -> roll right (right legs shorter)
        c = clamp(c, C_MIN, C_MAX)
        q[leg*3 + 0] = 0.0                    # hip
        q[leg*3 + 1] = BASE_THIGH + 0.40 * c  # thigh
        q[leg*3 + 2] = BASE_CALF  - 0.80 * c  # calf
    return q

# presets: (H, P, R)
PRESETS = {
    '1': (0.0, 0.0, 0.0),       # stand
    '2': (-0.10, 0.30, 0.0),    # sit-ish (rear down, nose up)
    '3': (-0.05, -0.30, 0.0),   # bow (nose down)
}

HELP = """
=== Go2 keyboard teleop (turtlesim-style) ===
  W / S : body up / down
  A / D : roll left / right
  Q / E : pitch nose down / up
  1 2 3 : presets (stand / sit / bow)
  space : neutral stand
  Ctrl+C: quit
Publishing LowCmd -> rt/lowcmd ...
"""

low_state = None
def on_state(msg):
    global low_state
    low_state = msg

def get_key(timeout):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else ''

def main():
    ChannelFactoryInitialize(1, "lo0")
    ChannelSubscriber("rt/lowstate", LowState_).Init(on_state, 10)
    pub = ChannelPublisher("rt/lowcmd", LowCmd_); pub.Init()

    print("Waiting for robot state...")
    while low_state is None:
        time.sleep(0.01)
    cmd_q = np.array([low_state.motor_state[i].q for i in range(12)], dtype=float)

    crc = CRC()
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE; cmd.head[1] = 0xEF; cmd.level_flag = 0xFF; cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].kd = 0.0

    H = P = R = 0.0
    print(HELP)
    last_status = 0.0
    while True:
        loop_t = time.perf_counter()
        k = get_key(0.0)
        if k:
            kl = k.lower()
            if   kl == 'w': H = clamp(H + STEP, -LIMIT, LIMIT)
            elif kl == 's': H = clamp(H - STEP, -LIMIT, LIMIT)
            elif kl == 'd': R = clamp(R + STEP, -LIMIT, LIMIT)
            elif kl == 'a': R = clamp(R - STEP, -LIMIT, LIMIT)
            elif kl == 'e': P = clamp(P + STEP, -LIMIT, LIMIT)
            elif kl == 'q': P = clamp(P - STEP, -LIMIT, LIMIT)
            elif k == ' ': H = P = R = 0.0
            elif k in PRESETS: H, P, R = PRESETS[k]

        target = compute_targets(H, P, R)
        cmd_q += SMOOTH * (target - cmd_q)
        for i in range(12):
            cmd.motor_cmd[i].q = cmd_q[i]
            cmd.motor_cmd[i].kp = KP; cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = KD; cmd.motor_cmd[i].tau = 0.0
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        now = time.perf_counter()
        if now - last_status > 0.1:
            sys.stdout.write(f"\r height={H:+.2f}  roll={R:+.2f}  pitch={P:+.2f}    ")
            sys.stdout.flush()
            last_status = now
        rem = dt - (time.perf_counter() - loop_t)
        if rem > 0: time.sleep(rem)

if __name__ == "__main__":
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        new = termios.tcgetattr(fd); new[3] &= ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSANOW, new)
        main()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\nteleop stopped.")
