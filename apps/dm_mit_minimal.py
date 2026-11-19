"""单电机 MIT 最小示例。运行前关闭上位机，固定空载电机，确认电机为 MIT 模式。"""

from pathlib import Path
import math
import sys
import time

import serial

SDK_DIR = Path(__file__).resolve().parents[1] / "达妙电机/motor-sdk-master/Python例程/u2can"
sys.path.insert(0, str(SDK_DIR))
from DM_CAN import Motor, MotorControl, DM_Motor_Type, DM_variable

# 常用参数在这里修改，角度 rad、速度 rad/s、力矩 N·m。
PORT = "COM9"
MOTOR_ID, MASTER_ID = 0x01, 0x11
POSITION = 0
VELOCITY = 0.5
KP, KD = 0.0, 0.0
TORQUE = 0.0
DURATION = 3.0
PERIOD = 0.05


class FeedbackMotor(Motor):
    """为 SDK 的缓存状态增加接收序号，检测通信中断。"""

    sequence = 0

    def recv_data(self, q, dq, tau, err):
        super().recv_data(q, dq, tau, err)
        self.sequence += 1


def wait_feedback(controller, motor, previous):
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline:
        controller.recv()
        if motor.sequence > previous:
            return
        time.sleep(0.002)
    raise RuntimeError("反馈超时，请检查电机和通信。断线后需现场停止。")


def main():
    values = (POSITION, VELOCITY, KP, KD, TORQUE, DURATION, PERIOD)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("参数必须为有限数值")
    if not (-12.5 <= POSITION <= 12.5 and abs(VELOCITY) <= 30
            and 0 <= KP <= 500 and 0 <= KD <= 5 and abs(TORQUE) <= 10
            and DURATION > 0 and PERIOD > 0):
        raise ValueError("参数超出当前电机协议映射范围，或运行时间无效")
    if KP > 0 and KD == 0:
        raise ValueError("MIT 位置控制需要 KD > 0，请设置阻尼")
    print(f"MIT 参数：位置={POSITION}，速度={VELOCITY}，KP={KP}，KD={KD}，"
          f"前馈力矩={TORQUE}，运行={DURATION}秒", flush=True)
    motor = FeedbackMotor(DM_Motor_Type.DM4310, MOTOR_ID, MASTER_ID)
    port = serial.Serial(port=None, baudrate=921600, timeout=0.02, write_timeout=0.2)
    port.port = PORT
    controller = None
    enable_attempted = False
    try:
        controller = MotorControl(port)
        controller.addMotor(motor)
        controller.change_limit_param(DM_Motor_Type.DM4310, 12.5, 30, 10)
        if controller.read_motor_param(motor, DM_variable.CTRL_MODE) != 1:
            raise RuntimeError("请先在上位机设为 MIT 模式，再关闭上位机后运行")
        enable_attempted = True
        previous = motor.sequence
        controller.enable(motor)
        wait_feedback(controller, motor, previous)
        if motor.getError() != 1:
            raise RuntimeError(f"使能失败，状态码 {motor.getError()}")
        started = time.monotonic()
        while time.monotonic() - started < DURATION:
            previous = motor.sequence
            controller.controlMIT(motor, KP, KD, POSITION, VELOCITY, TORQUE)
            wait_feedback(controller, motor, previous)
            if motor.getError() != 1:
                raise RuntimeError(f"电机未使能或发生故障，状态码 {motor.getError()}")
            print(f"位置 {motor.getPosition():.3f} rad  "
                  f"速度 {motor.getVelocity():.3f} rad/s  力矩 {motor.getTorque():.3f} N·m")
            time.sleep(PERIOD)
    finally:
        try:
            if controller is not None and enable_attempted:
                previous = motor.sequence
                controller.disable(motor)
                wait_feedback(controller, motor, previous)
                if motor.getError() != 0:
                    raise RuntimeError("失能未确认，请现场停止")
                print("电机已失能")
        finally:
            port.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("用户中止")
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"运行失败：{exc}", file=sys.stderr)
        raise SystemExit(1)
