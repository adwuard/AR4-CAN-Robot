from zdt_emmv5 import *
from ar4_configs import *
from joint import *
from ps5_joystick_handler import DualSense
import PySimpleGUI as psg

import time
import serial

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
