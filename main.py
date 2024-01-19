from zdt_emmv5 import *
from ar4_configs import *
from ps5_joystick_handler import DualSense
import PySimpleGUI as psg
import time
import serial


class Joint:
    def __init__(self, motor_driver, joint_cfg) -> None:
        self.driver = motor_driver
        self.cfg = joint_cfg

        self.id = joint_cfg["motor_addr"]
        self.name = joint_cfg["name"]
        self.enable = joint_cfg["enable"]
        self.axis_pos_limit_angle = joint_cfg["max_angle"]
        self.axis_neg_limit_angle = joint_cfg["min_angle"]

        self.step_master_ = 0
        self.gear_ratio = joint_cfg["gear_ratio"]
        self.steps_pre_degree = joint_cfg["steps_per_degree"]
        self.micro_steps = joint_cfg["microstep"]
        self.isStalled = False
        self.dir = joint_cfg["motor_direction"]
        self.speed = joint_cfg["max_speed_rpm"]
        self.acc = joint_cfg["acc_and_dec"]

        self.offset_angle = joint_cfg["calibration_zero_offset_angle"]

        if self.enable:
            self.init_joint()

    def joint_error_handler(self):
        pass

    def init_joint(self):
        # setup zeroing

        """Used for homing"""
        self.home_direction = self.cfg["home_direction"]
        self.home_speed_rpm = self.cfg["home_speed_rpm"]
        self.home_timeout_ms = self.cfg["home_timeout_ms"]
        self.home_method = self.cfg["home_method"]

        pass

    def move_joint_schedule(self, angle):
        new_neg = (
            abs(self.axis_neg_limit_angle) + self.cfg["calibration_zero_offset_angle"]
        )

        # always use pos mode and abs on fixed direction
        total_angle_limit = abs(new_neg) + abs(self.axis_pos_limit_angle)
        self.legal_full_steps = self.steps_pre_degree * total_angle_limit
        self.t_pose_zero_steps = abs(new_neg) * self.steps_pre_degree

        # print(self.legal_full_steps, self.t_pose_zero_steps, total_angle_limit)

        # self.acc = 10
        s = self.t_pose_zero_steps + (angle * self.steps_pre_degree)
        self.driver.set_position_control(
            self.dir,
            self.speed,
            self.acc,
            s * self.micro_steps,
            absolute_mode=True,
            wait_broadcast_signal=True,
        )

        # calc acceleration pulse count
        # 注意： 加速度档位为 0 表示不使用曲线加减速， 直接按照设定的速度运行。 曲线加
        # 减速时间计算公式： t2 - t1 = (256 - acc) * 50(us)， Vt2 = Vt1 + 1(RPM)；
        # （注： acc 为加速度档位， Vt1为 t1 时刻的转速， ， Vt2 为 t2 时刻的转速）

    def move_joint_nonblocking(self, angle):
        pass

    def check_motor_status(self):
        pass

    def on_arrived_flag(self, message):
        pass

    def check_collision_flag(self, message):
        pass


class Joints:
    def __init__(self, cfg) -> None:
        self.homing_sequence = cfg.homing_sequence
        self.joints = []
        self.cur_joint_angle = []

        self.broadcast_channel = 0x00

        self.serial_backend = serial.Serial(COM_PORT, 115200, timeout=0.001)
        self.hw_interface = SerialPort(self.serial_backend)
        self.broadcast = ZDT_EMMV5_MOTOR(self.hw_interface, self.broadcast_channel)

        for cfg in cfg.joints:
            driver = ZDT_EMMV5_MOTOR(self.hw_interface, cfg["motor_addr"])
            self.joints.append(Joint(driver, cfg))
            self.cur_joint_angle.append(-9999)

    def disable_all_joints(self):
        self.broadcast.disable_motor()

    def stop_all_joints(self):
        self.broadcast.stop_now()

    def run_scheduled_task(self):
        self.broadcast.broadcast_run()

    def check_all_arrived(self):
        pass

    def homing(self, wait_arrived=True):
        self.broadcast.trigger_sensor_zeroing()

        if not wait_arrived:
            return

        while 1:
            arrived = True
            for j in self.joints:
                if not arrived:
                    continue
                status = j.driver.read_zeroing_status()
                print("j", j.name, status)
                if status is None:
                    continue
                if status["ZEROING_FAILED"]:
                    raise Exception(j.name + "zeroing failed")
                if status["ZEROING_WORKING"] == True:
                    arrived = False
            if arrived:
                print("Zeroing success.")
                break

    def move_joints_joint_move_motion(self, angles, wait_arrived=True):
        for j in self.joints:
            if self.cur_joint_angle[int(j.id) - 1] != angles[int(j.id) - 1]:
                print(angles[int(j.id) - 1])
                j.move_joint_schedule(angles[int(j.id) - 1])

        self.run_scheduled_task()

        if not wait_arrived:
            self.cur_joint_angle = angles
            return

        while True:
            arrived = True
            for j in self.joints:
                if not arrived:
                    continue
                status = j.driver.read_motor_status_flags()
                if status is None:
                    continue
                if not status["MOTOR_ARRIVED_TARGET"]:
                    arrived = False

            if arrived:
                break

        self.cur_joint_angle = angles
        print("move joints to: ", self.cur_joint_angle, " success")


if __name__ == "__main__":
    ui_joint_angles = [0, 0, 0, 0, 0, 0, 0, 0]
    new_target_angle = [0, 0, 0, 0, 0, 0, 0, 0]

    joints = Joints(AR4_CFG)

    joints.homing()

    # time.sleep(10)

    print(joints.joints[0].axis_neg_limit_angle, joints.joints[0].axis_pos_limit_angle)

    layout = [
        [
            psg.Text(
                "Joint 1: \t\t" + str(joints.joints[0].axis_neg_limit_angle),
                font=("Arial Bold", 10),
            ),
            psg.Slider(
                range=(
                    joints.joints[0].axis_neg_limit_angle,
                    joints.joints[0].axis_pos_limit_angle,
                ),
                default_value=0,
                expand_x=True,
                orientation="horizontal",
                key="-J1-",
            ),
            psg.Text(
                str(joints.joints[0].axis_pos_limit_angle), font=("Arial Bold", 10)
            ),
        ],
        [
            psg.Text(
                "Joint 2: \t\t" + str(joints.joints[1].axis_neg_limit_angle),
                font=("Arial Bold", 10),
            ),
            psg.Slider(
                range=(
                    joints.joints[1].axis_neg_limit_angle,
                    joints.joints[1].axis_pos_limit_angle,
                ),
                default_value=0,
                expand_x=True,
                orientation="horizontal",
                key="-J2-",
            ),
            psg.Text(
                str(joints.joints[1].axis_pos_limit_angle), font=("Arial Bold", 10)
            ),
        ],
        [
            psg.Text(
                "Joint 3: \t\t" + str(joints.joints[2].axis_neg_limit_angle),
                font=("Arial Bold", 10),
            ),
            psg.Slider(
                range=(
                    joints.joints[2].axis_neg_limit_angle,
                    joints.joints[2].axis_pos_limit_angle,
                ),
                default_value=0,
                expand_x=True,
                orientation="horizontal",
                key="-J3-",
            ),
            psg.Text(
                str(joints.joints[2].axis_pos_limit_angle), font=("Arial Bold", 10)
            ),
        ],
        [
            psg.Text(
                "Joint 4: \t\t" + str(joints.joints[3].axis_neg_limit_angle),
                font=("Arial Bold", 10),
            ),
            psg.Slider(
                range=(
                    joints.joints[3].axis_neg_limit_angle,
                    joints.joints[3].axis_pos_limit_angle,
                ),
                default_value=0,
                expand_x=True,
                orientation="horizontal",
                key="-J4-",
            ),
            psg.Text(
                str(joints.joints[3].axis_pos_limit_angle), font=("Arial Bold", 10)
            ),
        ],
        [
            psg.Text(
                "Joint 5: \t\t" + str(joints.joints[4].axis_neg_limit_angle),
                font=("Arial Bold", 10),
            ),
            psg.Slider(
                range=(
                    joints.joints[4].axis_neg_limit_angle,
                    joints.joints[4].axis_pos_limit_angle,
                ),
                default_value=0,
                expand_x=True,
                orientation="horizontal",
                key="-J5-",
            ),
            psg.Text(
                str(joints.joints[4].axis_pos_limit_angle), font=("Arial Bold", 10)
            ),
        ],
        [
            psg.Text(
                "Joint 6: \t\t" + str(joints.joints[5].axis_neg_limit_angle),
                font=("Arial Bold", 10),
            ),
            psg.Slider(
                range=(
                    joints.joints[5].axis_neg_limit_angle,
                    joints.joints[5].axis_pos_limit_angle,
                ),
                default_value=0,
                expand_x=True,
                orientation="horizontal",
                key="-J6-",
            ),
            psg.Text(
                str(joints.joints[5].axis_pos_limit_angle), font=("Arial Bold", 10)
            ),
        ],
    ]

    window = psg.Window("AR4 Controller", layout)

    increment = 10

    def joystick_event_handler(event):
        # print(event)
        layer = event[0]
        table = event[1]

        for e in table:
            if e == "left-joystick-x":
                if layer == 1:
                    if table[e] == 1:
                        if (
                            new_target_angle[0] + increment
                            > joints.joints[0].axis_pos_limit_angle
                        ):
                            new_target_angle[0] = joints.joints[0].axis_pos_limit_angle
                        new_target_angle[0] += increment
                    else:
                        if (
                            new_target_angle[0] - increment
                            < joints.joints[0].axis_neg_limit_angle
                        ):
                            new_target_angle[0] = joints.joints[0].axis_neg_limit_angle
                        new_target_angle[0] -= increment

                if layer == 2:
                    if table[e] == 1:
                        if (
                            new_target_angle[2] + increment
                            > joints.joints[2].axis_pos_limit_angle
                        ):
                            new_target_angle[2] = joints.joints[2].axis_pos_limit_angle
                        new_target_angle[2] += increment
                    else:
                        if (
                            new_target_angle[2] - increment
                            < joints.joints[2].axis_neg_limit_angle
                        ):
                            new_target_angle[2] = joints.joints[2].axis_neg_limit_angle
                        new_target_angle[2] -= increment
            if e == "left-joystick-y":
                pass

            if e == "right-joystick-x":
                print(layer, table[e])
                if layer == 1:
                    if table[e] == 1:
                        if (
                            new_target_angle[1] + increment
                            > joints.joints[1].axis_pos_limit_angle
                        ):
                            new_target_angle[1] = joints.joints[1].axis_pos_limit_angle
                        new_target_angle[1] += increment
                    else:
                        if (
                            new_target_angle[1] - increment
                            < joints.joints[1].axis_neg_limit_angle
                        ):
                            new_target_angle[1] = joints.joints[1].axis_neg_limit_angle
                        new_target_angle[1] -= increment
                if layer == 2:
                    if table[e] == 1:
                        if (
                            new_target_angle[3] + increment
                            > joints.joints[3].axis_pos_limit_angle
                        ):
                            new_target_angle[3] = joints.joints[3].axis_pos_limit_angle
                        new_target_angle[3] += increment
                    else:
                        if (
                            new_target_angle[3] - increment
                            < joints.joints[3].axis_neg_limit_angle
                        ):
                            new_target_angle[3] = joints.joints[3].axis_neg_limit_angle
                        new_target_angle[3] -= increment

            if e == "right-joystick-y":
                pass
            if e == "left-trigger":
                if layer == 1:
                    if table[e] == 1:
                        if (
                            new_target_angle[4] + increment
                            > joints.joints[4].axis_pos_limit_angle
                        ):
                            new_target_angle[4] = joints.joints[4].axis_pos_limit_angle
                        new_target_angle[4] += increment

                    if table[e] == 2:
                        if (
                            new_target_angle[5] + increment
                            > joints.joints[5].axis_pos_limit_angle
                        ):
                            new_target_angle[5] = joints.joints[5].axis_pos_limit_angle
                        new_target_angle[5] += increment

            if e == "right-trigger":
                if layer == 1:
                    if table[e] == 1:
                        if (
                            new_target_angle[4] - increment
                            > joints.joints[4].axis_pos_limit_angle
                        ):
                            new_target_angle[4] = joints.joints[4].axis_pos_limit_angle
                        new_target_angle[4] -= increment
                if layer == 2:
                    if table[e] == 1:
                        if (
                            new_target_angle[5] - increment
                            > joints.joints[5].axis_pos_limit_angle
                        ):
                            new_target_angle[5] = joints.joints[5].axis_pos_limit_angle
                        new_target_angle[5] -= increment

        print(new_target_angle)
        joints.move_joints_joint_move_motion(new_target_angle, wait_arrived=True)

    joints.broadcast.enable_motor()
    joints.broadcast.clear_stall_error()
    joints.homing()
    joints.move_joints_joint_move_motion([-160, -30, -45, -60, -80, -90, 0, 0])
    joints.homing()
    joints.move_joints_joint_move_motion([0,0,0,0,0,0,0,0])

    # while 1:

    # joints.move_joints_joint_move_motion(
    #     [
    #         joints.joints[0].axis_neg_limit_angle + 10,
    #         joints.joints[1].axis_neg_limit_angle + 10,
    #         joints.joints[2].axis_neg_limit_angle + 10,
    #         joints.joints[3].axis_neg_limit_angle + 10,
    #         joints.joints[4].axis_neg_limit_angle + 10,
    #         joints.joints[5].axis_neg_limit_angle + 10,
    #         0,
    #         0,
    #     ]
    # )
    #     # time.sleep(1)
    # joints.move_joints_joint_move_motion([20,20,3,20,34,40,0,0])
    #     joints.move_joints_joint_move_motion([-20,60,30,20,-10,-34,0,0])
    #     joints.move_joints_joint_move_motion([0,0,0,0,0,0,0,0])

    # # self.broadcast.disable_motor()
    # # time.sleep(5)

    # ds = DualSense(joystick_event_handler)

    # while ds.is_running:
    # event, values = window.read(timeout=500)

    # print(event, "---", values)
    # if event == psg.WIN_CLOSED or event == "Exit":
    # break

    # new_target_angle[0] = int(values["-J1-"])
    # new_target_angle[1] = int(values["-J2-"])
    # new_target_angle[2] = int(values["-J3-"])
    # new_target_angle[3] = int(values["-J4-"])
    # new_target_angle[4] = int(values["-J5-"])
    # new_target_angle[5] = int(values["-J6-"])

    # print(new_target_angle)

    # if new_target_angle != ui_joint_angles:
    #     print("new target angle", new_target_angle)
    #     # ui_joint_angles = new_target_angle
    #     ui_joint_angles[0] = new_target_angle[0]
    #     ui_joint_angles[1] = new_target_angle[1]
    #     ui_joint_angles[2] = new_target_angle[2]
    #     ui_joint_angles[3] = new_target_angle[3]
    #     ui_joint_angles[4] = new_target_angle[4]
    #     ui_joint_angles[5] = new_target_angle[5]
    #     ui_joint_angles[6] = new_target_angle[6]
    #     ui_joint_angles[7] = new_target_angle[7]
    #     joints.move_joints_joint_move_motion(ui_joint_angles, wait_arrived=False)

    # print(event, values)
