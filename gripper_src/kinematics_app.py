from zdt_emmv5 import *
from ar4_configs import *
from joint import *
import PySimpleGUI as psg

import time
import serial


def clamp_angle(angle, neg_limit, pos_limit):
    angle = float(angle)
    neg_limit = float(neg_limit)
    pos_limit = float(pos_limit)
    if angle < neg_limit:
        return neg_limit
    elif angle > pos_limit:
        return pos_limit
    else:
        return angle


if __name__ == "__main__":
    psg.theme("Dark")  # Add a touch of color

    joints = Joints(AR4_CFG)

    ui_joints_target = [0, 0, 0, 0, 0, 0, 0, 0]

    joint_jog_panel = []

    for i in range(len(joints.joints)):
        if not joints.joints[i].enable:
            continue

        joint_jog_panel.append(
            [
                psg.Text(f"Joint {i + 1}: \t", font=("Arial Bold", 10)),
                psg.Text(
                    str(joints.joints[i].axis_neg_limit_angle), font=("Arial Bold", 10)
                ),
                psg.Slider(
                    range=(
                        joints.joints[i].axis_neg_limit_angle,
                        joints.joints[i].axis_pos_limit_angle,
                    ),
                    default_value=0.0,
                    resolution=0.1,
                    expand_x=True,
                    orientation="horizontal",
                    key=f"-J{i + 1}-Slider-",
                    enable_events=True,
                ),
                psg.Text(
                    str(joints.joints[i].axis_pos_limit_angle), font=("Arial Bold", 10)
                ),
                psg.InputText(
                    str(ui_joints_target[i]),
                    key=f"-J{i + 1}-Input-",
                    size=(10, 1),
                    enable_events=True,
                ),
            ]
        )


    joint_radio = [psg.Text("Joint Jog")]
    joint_radio.extend(
        [
            psg.Radio(f"J{i + 1}", "RADIO1", key=f"-radio1-J{i + 1}-")
            for i in range(len(joints.joints))
            if joints.joints[i].enable
        ]
    )

    # cartesian_radio = [psg.Text("Cartesian Jog")]
    # cartesian_radio.extend(
    #     [
    #         psg.Radio(axis, "RADIO1", key=f"-radio1-{axis}-")
    #         for axis in ["X", "Y", "Z", "Rz", "Ry", "Rx"]
    #     ]
    # )
        # cartesian_radio,


                # current_angles,
    layout = [
        [
            [
                psg.Button("Homing", size=(10, 2), key='-homing-btn-', button_color=("white", "blue")),
                psg.Button("T-Pose-Zero", size=(10, 2), key='-t-pose-zero-btn-', button_color=("white", "blue")),
                psg.Button("PANIC STOP!", size=(10, 2), key='-panic-stop-btn-', button_color=("white", "red")),
            ],
        ],
        [
            psg.Frame(
                "Joint Controls", joint_jog_panel, font="Any 18", title_color="White"
            )
        ],
    ]

    window = psg.Window("Vibe Gripper", layout, finalize=True)

    for i in range(len(joints.joints)):
        if joints.joints[i].enable:
            window[f"-J{i + 1}-Input-"].bind("<Return>", "_Enter")
            window[f"-J{i + 1}-Slider-"].bind("<ButtonRelease-1>", " _Release")

    increment = 10

    while True:
        event, values = window.read(timeout=500)

        if event == psg.WIN_CLOSED or event == "Exit":
            break

        update_xyzuvw_render = False
        update_joint_render = False

        isJointEvent = False
        isCarEvent = False

        if event == '-t-pose-zero-btn-':
            print("Performing T-Pose")
            joints.move_joints_joint_move_motion([0], wait_arrived=False)
            
        if event == '-homing-btn-':
            print("Performing Homing")
            joints.homing()
        
        if event == '-panic-stop-btn-':
            print("Panic STOP!!")
            joints.stop_all_joints()
            

        # Slider updates input-box, and vice-versa
        for i in range(len(joints.joints)):
            if not joints.joints[i].enable:
                continue

            if event == "-J{}-Input-".format(i + 1) + "_Enter":
                try:
                    slider_value = values["-J{}-Input-".format(i + 1)]
                    slider_value = clamp_angle(
                        slider_value,
                        joints.joints[i].axis_neg_limit_angle,
                        joints.joints[i].axis_pos_limit_angle,
                    )
                    window["-J{}-Slider-".format(i + 1)].update(slider_value)
                    isJointEvent = True
                except ValueError:
                    pass

            elif event == "-J{}-Slider-".format(i + 1) + " _Release":
                try:
                    input_value = values["-J{}-Slider-".format(i + 1)]
                    input_value = clamp_angle(
                        input_value,
                        joints.joints[i].axis_neg_limit_angle,
                        joints.joints[i].axis_pos_limit_angle,
                    )
                    window["-J{}-Input-".format(i + 1)].update(input_value)
                    isJointEvent = True
                except ValueError:
                    pass

            ui_joints_target[i] = values["-J{}-Slider-".format(i + 1)]

        
        # joints.move_joints_joint_move_linear_motion([40], wait_arrived=False)
        # joints.move_joints_joint_move_motion([ui_joints_target[1:-1], wait_arrived=False)
        
        if update_joint_render:
            for i in range(len(joints.joints)):
                if not joints.joints[i].enable:
                    continue

                window["-J{}-Input-".format(i + 1)].update("{:.2f}".format(ui_joints_target[i]))
                window["-J{}-Slider-".format(i + 1)].update("{:.2f}".format(ui_joints_target[i]))
                window["-J{}-Slider-".format(i + 1)].update("{:.2f}".format(ui_joints_target[i]))
                


        # if event != "__TIMEOUT__":
        # print(event)

    window.close()
