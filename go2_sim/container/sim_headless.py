import time
import mujoco
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge
import config

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)
mj_model.opt.timestep = config.SIMULATE_DT

ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
bridge = UnitreeSdk2Bridge(mj_model, mj_data)
if config.PRINT_SCENE_INFORMATION:
    bridge.PrintSceneInformation()

print(">>> HEADLESS sim running (no window). DDS: domain", config.DOMAIN_ID, "iface", config.INTERFACE)
print(">>> Press Ctrl+C to stop.")
while True:
    t0 = time.perf_counter()
    mujoco.mj_step(mj_model, mj_data)
    dt = mj_model.opt.timestep - (time.perf_counter() - t0)
    if dt > 0:
        time.sleep(dt)
