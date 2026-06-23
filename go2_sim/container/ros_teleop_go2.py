#!/usr/bin/env python3
"""ROS2 (rclpy) keyboard teleop for Go2 sim. Publishes unitree_go/msg/LowCmd to /lowcmd,
subscribes /lowstate. CRC is computed via the SDK LowCmd_ (same field layout)."""
import sys, time, select, termios, tty
import numpy as np
import rclpy
from rclpy.node import Node
from unitree_go.msg import LowCmd, LowState
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_

KP, KD = 50.0, 3.5
PUB_DT = 0.002
STEP = 0.04
LIMIT = 0.35
SMOOTH = 0.02
C_MIN, C_MAX = -0.4, 1.3
BASE_THIGH, BASE_CALF = 0.61, -1.22
LEG_FRONT = [True, True, False, False]   # FR, FL, RR, RL
LEG_RIGHT = [True, False, True, False]

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def compute_targets(H, P, R):
    q = np.zeros(12)
    for leg in range(4):
        c = -H
        c += (-P if LEG_FRONT[leg] else P)
        c += (R if LEG_RIGHT[leg] else -R)
        c = clamp(c, C_MIN, C_MAX)
        q[leg*3+0] = 0.0
        q[leg*3+1] = BASE_THIGH + 0.40*c
        q[leg*3+2] = BASE_CALF  - 0.80*c
    return q

PRESETS = {'1': (0.0,0.0,0.0), '2': (-0.10,0.30,0.0), '3': (-0.05,-0.30,0.0)}

HELP = """
=== Go2 ROS2 keyboard teleop (rclpy -> /lowcmd) ===
  W / S  : body up / down
  A / D  : roll left / right
  Q / E  : pitch nose down / up
  arrows : <- / -> roll,  up / down pitch
  1 2 3  : presets (stand / sit / bow)
  space  : neutral stand
  Ctrl+C : quit
"""

def read_keys():
    r, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not r:
        return []
    data = sys.stdin.read(1)
    while select.select([sys.stdin], [], [], 0)[0]:
        data += sys.stdin.read(1)
    toks, i = [], 0
    while i < len(data):
        if data[i] == '\x1b' and i+2 < len(data) and data[i+1] == '[':
            a = {'A':'UP','B':'DOWN','C':'RIGHT','D':'LEFT'}.get(data[i+2])
            if a: toks.append(a); i += 3; continue
            i += 1; continue
        toks.append(data[i]); i += 1
    return toks

def apply(tok, H, P, R):
    k = tok.lower() if len(tok) == 1 else tok
    if   k == 'w': H = clamp(H+STEP, -LIMIT, LIMIT)
    elif k == 's': H = clamp(H-STEP, -LIMIT, LIMIT)
    elif k == 'd': R = clamp(R+STEP, -LIMIT, LIMIT)
    elif k == 'a': R = clamp(R-STEP, -LIMIT, LIMIT)
    elif k == 'e': P = clamp(P+STEP, -LIMIT, LIMIT)
    elif k == 'q': P = clamp(P-STEP, -LIMIT, LIMIT)
    elif tok == 'RIGHT': R = clamp(R+STEP, -LIMIT, LIMIT)
    elif tok == 'LEFT':  R = clamp(R-STEP, -LIMIT, LIMIT)
    elif tok == 'UP':    P = clamp(P+STEP, -LIMIT, LIMIT)
    elif tok == 'DOWN':  P = clamp(P-STEP, -LIMIT, LIMIT)
    elif tok == ' ': H = P = R = 0.0
    elif tok in PRESETS: H, P, R = PRESETS[tok]
    return H, P, R

class Teleop(Node):
    def __init__(self):
        super().__init__('go2_ros_teleop')
        self.pub = self.create_publisher(LowCmd, 'lowcmd', 10)
        self.sub = self.create_subscription(LowState, 'lowstate', self.on_state, 10)
        self.crc = CRC()
        self.q_meas = None
        self.cmd_q = None
        self.H = self.P = self.R = 0.0
        self.last_status = 0.0

    def on_state(self, msg):
        self.q_meas = np.array([msg.motor_state[i].q for i in range(12)], dtype=float)

    def build_cmd(self, cmd_q):
        msg = LowCmd()
        msg.head[0] = 0xFE; msg.head[1] = 0xEF
        msg.level_flag = 0xFF
        msg.gpio = 0
        for i in range(20):
            msg.motor_cmd[i].mode = 0x01
            if i < 12:
                msg.motor_cmd[i].q = float(cmd_q[i])
                msg.motor_cmd[i].kp = KP
                msg.motor_cmd[i].dq = 0.0
                msg.motor_cmd[i].kd = KD
                msg.motor_cmd[i].tau = 0.0
            else:
                msg.motor_cmd[i].q = 0.0; msg.motor_cmd[i].kp = 0.0
                msg.motor_cmd[i].dq = 0.0; msg.motor_cmd[i].kd = 0.0
                msg.motor_cmd[i].tau = 0.0
        sdk = unitree_go_msg_dds__LowCmd_()
        sdk.head = [0xFE, 0xEF]; sdk.level_flag = 0xFF; sdk.gpio = 0
        for i in range(20):
            sdk.motor_cmd[i].mode = msg.motor_cmd[i].mode
            sdk.motor_cmd[i].q   = msg.motor_cmd[i].q
            sdk.motor_cmd[i].dq  = msg.motor_cmd[i].dq
            sdk.motor_cmd[i].kp  = msg.motor_cmd[i].kp
            sdk.motor_cmd[i].kd  = msg.motor_cmd[i].kd
            sdk.motor_cmd[i].tau = msg.motor_cmd[i].tau
        msg.crc = self.crc.Crc(sdk)
        return msg

def main():
    rclpy.init()
    node = Teleop()
    print("Waiting for /lowstate ...")
    while rclpy.ok() and node.q_meas is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.cmd_q = node.q_meas.copy()
    print(HELP)
    try:
        while rclpy.ok():
            loop_t = time.perf_counter()
            rclpy.spin_once(node, timeout_sec=0.0)
            for tok in read_keys():
                node.H, node.P, node.R = apply(tok, node.H, node.P, node.R)
            target = compute_targets(node.H, node.P, node.R)
            node.cmd_q += SMOOTH * (target - node.cmd_q)
            node.pub.publish(node.build_cmd(node.cmd_q))
            now = time.perf_counter()
            if now - node.last_status > 0.1:
                sys.stdout.write(f"\r height={node.H:+.2f}  roll={node.R:+.2f}  pitch={node.P:+.2f}    ")
                sys.stdout.flush(); node.last_status = now
            rem = PUB_DT - (time.perf_counter() - loop_t)
            if rem > 0: time.sleep(rem)
    finally:
        node.destroy_node()
        rclpy.shutdown()

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
        print("\nROS teleop stopped.")
