"""Probe the actual OpenGL renderer and render the V1 model off-screen."""
import json
import sys
import time

import glfw
import mujoco
from OpenGL.GL import glGetString, glFinish, GL_VENDOR, GL_RENDERER, GL_VERSION
from openarm_app.kinematics import OpenArmIK


def main():
    if not glfw.init():
        raise RuntimeError("GLFW initialization failed")
    glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
    window = glfw.create_window(1024, 768, "OpenArm GPU diagnostic", None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("OpenGL context creation failed")
    context = None
    try:
        glfw.make_context_current(window)
        report = {"python": sys.executable, "base_python": sys._base_executable}
        for key, enum in (("vendor", GL_VENDOR), ("renderer", GL_RENDERER), ("version", GL_VERSION)):
            report[key] = glGetString(enum).decode()
        print(json.dumps(report, indent=2), flush=True)
        arm = OpenArmIK()
        model, data = arm.setup.model, arm.setup.data
        model.vis.quality.shadowsize = 2048
        scene = mujoco.MjvScene(model, maxgeom=10000)
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, camera)
        option = mujoco.MjvOption()
        context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)
        rect = mujoco.MjrRect(0, 0, 1024, 768)
        mujoco.mjv_updateScene(model, data, option, None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
        for shadows in (True, False):
            scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = shadows
            for _ in range(5):
                mujoco.mjr_render(rect, scene, context)
            glFinish()
            start = time.perf_counter()
            for _ in range(30):
                mujoco.mjr_render(rect, scene, context)
                glFinish()
            elapsed = time.perf_counter() - start
            print(json.dumps({"shadows": shadows, "resolution": "1024x768",
                              "render_fps": 30/elapsed, "frame_ms": elapsed*1000/30}), flush=True)
    finally:
        if context is not None:
            context.free()
        glfw.destroy_window(window)
        glfw.terminate()


if __name__ == "__main__":
    main()
