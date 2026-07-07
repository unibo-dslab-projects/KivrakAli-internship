#!/usr/bin/env python3
"""Keyboard-teleop crawl gait for the Go2 sim.
  up / down arrow : walk forward / backward
  space           : stop (stand in place)
  w / s           : increase / decrease stride length (live tuning)
  e / d           : increase / decrease swing height (live tuning)
  r / f           : increase / decrease weight-shift (live tuning)
  t / g           : faster / slower cycle time (live tuning)
  Ctrl+C          : quit"""
import sys, time, math, select, termios, tty
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import LowCmd, LowState
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_

L_HIP_Y = 0.0955
L_THIGH = 0.213
L_CALF  = 0.213
Z0      = -0.30
KP, KD  = 50.0, 3.5
PUB_DT  = 0.002
SMOOTH  = 0.05
CYCLE_T  = 5.0
DUTY     = 0.75
STEP_LEN = 0.08
STEP_H   = 0.06
SHIFT_Y  = 0.05
SHIFT_X  = 0.02

FX_SIGN = [ +1, +1, -1, -1 ]
FY_SIGN = [ -1, +1, -1, +1 ]
IS_RIGHT = [ True, False, True, False ]
LEG_ORDER = [0, 3, 1, 2]
SWING_START = [0.0]*4
for pos, leg in enumerate(LEG_ORDER):
    SWING_START[leg] = pos / 4.0
SW = 1.0 - DUTY

def leg_ik_plane(x, z):
    d = min(math.hypot(x, z), L_THIGH + L_CALF - 1e-6)
    ck = (L_THIGH**2 + L_CALF**2 - d**2) / (2*L_THIGH*L_CALF)
    qc = -(math.pi - math.acos(max(-1, min(1, ck))))
    cb = (d**2 + L_THIGH**2 - L_CALF**2) / (2*d*L_THIGH)
    qt = math.atan2(x, -z) + math.acos(max(-1, min(1, cb)))
    return qt, qc

def leg_ik_3d(fx, fy, fz, right):
    dy = -L_HIP_Y if right else L_HIP_Y
    r2 = fy*fy + fz*fz - dy*dy
    if r2 < 0: r2 = 0.0
    z_sag = -math.sqrt(r2)
    qh = math.atan2(fz, fy) - math.atan2(z_sag, dy)
    qt, qc = leg_ik_plane(-fx, z_sag)
    return qh, qt, qc

def gait_targets(phase, stride, lift, shx, shy):
    Bx = By = 0.0
    for leg in range(4):
        p = (phase - SWING_START[leg]) % 1.0
        if p < SW:
            s = p / SW
            bump = math.sin(math.pi*s)
            Bx = -shx * FX_SIGN[leg] * bump
            By = -shy * FY_SIGN[leg] * bump
            break
    q = np.zeros(12)
    for leg in range(4):
        p = (phase - SWING_START[leg]) % 1.0
        if p < SW:
            s = p / SW
            gx = -stride/2 + stride*s
            gz = lift*math.sin(math.pi*s)
        else:
            u = (p - SW) / DUTY
            gx = stride/2 - stride*u
            gz = 0.0
        dy = -L_HIP_Y if IS_RIGHT[leg] else L_HIP_Y
        qh, qt, qc = leg_ik_3d(gx - Bx, dy - By, Z0 + gz, IS_RIGHT[leg])
        q[leg*3+0] = qh; q[leg*3+1] = qt; q[leg*3+2] = qc
    return q

def stand_targets():
    q = np.zeros(12)
    for leg in range(4):
        dy = -L_HIP_Y if IS_RIGHT[leg] else L_HIP_Y
        qh, qt, qc = leg_ik_3d(0.0, dy, Z0, IS_RIGHT[leg])
        q[leg*3+0] = qh; q[leg*3+1] = qt; q[leg*3+2] = qc
    return q

def read_keys():
    r, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not r: return []
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

class GaitTeleop(Node):
    def __init__(self):
        super().__init__('gait_teleop')
        self.pub = self.create_publisher(LowCmd, 'lowcmd', 10)
        self.sub = self.create_subscription(LowState, 'lowstate', self.on_state, qos_profile_sensor_data)
        self.crc = CRC()
        self.q_meas = None
        self.cmd_q = None
        self.direction = 0
        self.stride = STEP_LEN
        self.lift = STEP_H
        self.shy = SHIFT_Y
        self.shx = SHIFT_X
        self.cycle = CYCLE_T
        self.phase = 0.0

    def on_state(self, msg):
        self.q_meas = np.array([msg.motor_state[i].q for i in range(12)], dtype=float)

    def apply_key(self, tok):
        k = tok.lower() if len(tok) == 1 else tok
        if   tok == 'UP':    self.direction = +1
        elif tok == 'DOWN':  self.direction = -1
        elif tok == ' ':     self.direction = 0
        elif k == 'w': self.stride = min(0.16, self.stride + 0.01)
        elif k == 's': self.stride = max(0.02, self.stride - 0.01)
        elif k == 'e': self.lift = min(0.12, self.lift + 0.01)
        elif k == 'd': self.lift = max(0.02, self.lift - 0.01)
        elif k == 'r': self.shy = min(0.10, self.shy + 0.01)
        elif k == 'f': self.shy = max(0.0,  self.shy - 0.01)
        elif k == 't': self.cycle = max(2.0, self.cycle - 0.5)
        elif k == 'g': self.cycle = min(10.0, self.cycle + 0.5)

    def build(self, q):
        msg = LowCmd()
        msg.head[0]=0xFE; msg.head[1]=0xEF; msg.level_flag=0xFF; msg.gpio=0
        for i in range(20):
            msg.motor_cmd[i].mode = 0x01
            if i < 12:
                msg.motor_cmd[i].q=float(q[i]); msg.motor_cmd[i].kp=KP
                msg.motor_cmd[i].dq=0.0; msg.motor_cmd[i].kd=KD; msg.motor_cmd[i].tau=0.0
            else:
                msg.motor_cmd[i].q=0.0; msg.motor_cmd[i].kp=0.0
                msg.motor_cmd[i].dq=0.0; msg.motor_cmd[i].kd=0.0; msg.motor_cmd[i].tau=0.0
        sdk = unitree_go_msg_dds__LowCmd_()
        sdk.head=[0xFE,0xEF]; sdk.level_flag=0xFF; sdk.gpio=0
        for i in range(20):
            m=msg.motor_cmd[i]; s=sdk.motor_cmd[i]
            s.mode=m.mode; s.q=m.q; s.dq=m.dq; s.kp=m.kp; s.kd=m.kd; s.tau=m.tau
        msg.crc = self.crc.Crc(sdk)
        return msg

HELP = """
=== Go2 crawl-gait teleop ===
  up/down : forward / backward      space : stop (stand)
  w/s : stride +/-    e/d : swing height +/-    r/f : weight-shift +/-    t/g : faster/slower
  Ctrl+C : quit
"""

def main():
    rclpy.init()
    node = GaitTeleop()
    print("Waiting for /lowstate ...")
    while rclpy.ok() and node.q_meas is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.cmd_q = node.q_meas.copy()
    print(HELP)
    last_status = 0.0
    try:
        while rclpy.ok():
            loop_t = time.perf_counter()
            rclpy.spin_once(node, timeout_sec=0.0)
            for tok in read_keys():
                node.apply_key(tok)
            if node.direction == 0:
                target = stand_targets()
            else:
                node.phase = (node.phase + PUB_DT / node.cycle) % 1.0
                stride = node.stride * node.direction
                target = gait_targets(node.phase, stride, node.lift, node.shx, node.shy)
            node.cmd_q += SMOOTH * (target - node.cmd_q)
            node.pub.publish(node.build(node.cmd_q))
            now = time.perf_counter()
            if now - last_status > 0.2:
                d = {1:"FWD ",-1:"BACK",0:"STOP"}[node.direction]
                sys.stdout.write(f"\r {d} stride={node.stride:.2f} lift={node.lift:.2f} shy={node.shy:.2f} cyc={node.cycle:.1f}   ")
                sys.stdout.flush(); last_status = now
            rem = PUB_DT - (time.perf_counter() - loop_t)
            if rem > 0: time.sleep(rem)
    finally:
        node.destroy_node(); rclpy.shutdown()
        print("\nstopped.")

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
        print("\nGait teleop stopped.")
