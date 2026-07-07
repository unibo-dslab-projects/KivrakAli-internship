#!/usr/bin/env python3
"""Static crawl gait for the Go2 sim (forward walking).
One leg swings at a time (duty 0.75); body weight shifts toward the support
triangle before each lift. Closed-form 3D leg IK. Publishes LowCmd to /lowcmd."""
import time, math
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
Z0      = -0.32

KP, KD  = 50.0, 3.5
PUB_DT  = 0.002
SMOOTH  = 0.05

CYCLE_T  = 6.0
DUTY     = 0.75
STEP_LEN = -0.08
STEP_H   = 0.08
SHIFT_Y  = 0.05
SHIFT_X  = 0.02
SETTLE_T = 2.0
RAMP_T   = 2.0

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
    knee = math.acos(max(-1, min(1, ck)))
    qc = -(math.pi - knee)
    cb = (d**2 + L_THIGH**2 - L_CALF**2) / (2*d*L_THIGH)
    b = math.acos(max(-1, min(1, cb)))
    qt = math.atan2(x, -z) + b
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
        fx = gx - Bx
        fy = dy - By
        fz = Z0 + gz
        qh, qt, qc = leg_ik_3d(fx, fy, fz, IS_RIGHT[leg])
        q[leg*3+0] = qh
        q[leg*3+1] = qt
        q[leg*3+2] = qc
    return q, Bx, By

class Gait(Node):
    def __init__(self):
        super().__init__('gait_go2')
        self.pub = self.create_publisher(LowCmd, 'lowcmd', 10)
        self.sub = self.create_subscription(LowState, 'lowstate', self.on_state, qos_profile_sensor_data)
        self.crc = CRC()
        self.q_meas = None
        self.cmd_q = None
        self.t0 = None
        self.last_status = 0.0

    def on_state(self, msg):
        self.q_meas = np.array([msg.motor_state[i].q for i in range(12)], dtype=float)

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

def main():
    rclpy.init()
    node = Gait()
    print("Waiting for /lowstate ...")
    while rclpy.ok() and node.q_meas is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.cmd_q = node.q_meas.copy()
    node.t0 = time.perf_counter()
    print("Crawl gait: settling, then walking forward. Ctrl+C to stop.")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            t = time.perf_counter() - node.t0
            if t < SETTLE_T:
                target, Bx, By = gait_targets(0.0, 0.0, 0.0, 0.0, 0.0)
                phase = 0.0
            else:
                tw = t - SETTLE_T
                r = min(1.0, tw / RAMP_T)
                phase = (tw / CYCLE_T) % 1.0
                target, Bx, By = gait_targets(phase, STEP_LEN*r, STEP_H*r, SHIFT_X*r, SHIFT_Y*r)
            node.cmd_q += SMOOTH * (target - node.cmd_q)
            node.pub.publish(node.build(node.cmd_q))
            now = time.perf_counter()
            if now - node.last_status > 0.2:
                sys_ph = 0.0 if t < SETTLE_T else phase
                print(f"\r t={t:5.1f}s phase={sys_ph:.2f} Bx={Bx:+.3f} By={By:+.3f}   ", end="")
                node.last_status = now
            time.sleep(PUB_DT)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()
        print("\nstopped.")

if __name__ == "__main__":
    main()
