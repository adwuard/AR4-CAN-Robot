from zdt_emmv5 import *
from ar4_configs import *
from ar4_kinematics import *
from joint import *
from ps5_joystick_handler import DualSense
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

    kinematics = Kinematics()
    kinematics.Set_Tool_Frame(0, 0, 0, 0, 0, 0)

    joints = Joints(AR4_CFG)

    ui_joints_target = [0, 0, 0, 0, 0, 0, 0, 0]
    ui_xyzuvw_target = kinematics.SolveFowardKinematic(ui_joints_target)

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

    current_angles = [
        [
            psg.Text("X: ", font=("Arial Bold", 10)),
            psg.Text(
                str("{:.2f}".format(ui_xyzuvw_target[0])),
                font=("Arial Bold", 10),
                key="-x_curr-",
            ),
        ],
        [
            psg.Text("Y: ", font=("Arial Bold", 10)),
            psg.Text(
                str("{:.2f}".format(ui_xyzuvw_target[1])),
                font=("Arial Bold", 10),
                key="-y_curr-",
            ),
        ],
        [
            psg.Text("Z: ", font=("Arial Bold", 10)),
            psg.Text(
                str("{:.2f}".format(ui_xyzuvw_target[2])),
                font=("Arial Bold", 10),
                key="-z_curr-",
            ),
        ],
        [
            psg.Text("Rz: ", font=("Arial Bold", 10)),
            psg.Text(
                str("{:.2f}".format(ui_xyzuvw_target[3])),
                font=("Arial Bold", 10),
                key="-rz_curr-",
            ),
        ],
        [
            psg.Text("Ry: ", font=("Arial Bold", 10)),
            psg.Text(
                str("{:.2f}".format(ui_xyzuvw_target[4])),
                font=("Arial Bold", 10),
                key="-ry_curr-",
            ),
        ],
        [
            psg.Text("Rx: ", font=("Arial Bold", 10)),
            psg.Text(
                str("{:.2f}".format(ui_xyzuvw_target[5])),
                font=("Arial Bold", 10),
                key="-rx_curr-",
            ),
        ],
    ]

    joint_radio = [psg.Text("Joint Jog")]
    joint_radio.extend(
        [
            psg.Radio(f"J{i + 1}", "RADIO1", key=f"-radio1-J{i + 1}-")
            for i in range(len(joints.joints))
            if joints.joints[i].enable
        ]
    )

    cartesian_radio = [psg.Text("Cartesian Jog")]
    cartesian_radio.extend(
        [
            psg.Radio(axis, "RADIO1", key=f"-radio1-{axis}-")
            for axis in ["X", "Y", "Z", "Rz", "Ry", "Rx"]
        ]
    )

    jogging_layout = [
        joint_radio,
        cartesian_radio,
        [
            psg.Button(
                "Zero",
                key=f"-J{i + 1}-set-zero-",
                size=(3, 1),
                button_color=("white", "gray"),
            ),
            psg.Text("||||", font=("Arial Bold", 10)),
            psg.Button("-10", key=f"-Dec10-", size=(3, 1)),
            psg.Button("-5", key=f"-Dec5-", size=(3, 1)),
            psg.Button("-1", key=f"-Dec1-", size=(3, 1)),
            psg.Text("||||", font=("Arial Bold", 10)),
            psg.Button("+1", key=f"-Inc1-", size=(3, 1)),
            psg.Button("+5", key=f"-Inc5-", size=(3, 1)),
            psg.Button("+10", key=f"-Inc10-", size=(3, 1)),
        ],
    ]

    layout = [
        [
            psg.Frame(
                "Current Cartesian Coord.",
                current_angles,
                font="Any 18",
                title_color="White",
            ),
            [
                psg.Button("Homing", size=(10, 2), button_color=("white", "blue")),
                psg.Button("PANIC STOP!", size=(10, 2), button_color=("white", "red")),
            ],
        ],
        [
            psg.Frame(
                "Joint Controls", joint_jog_panel, font="Any 18", title_color="White"
            )
        ],
        [psg.Frame("Jogging", jogging_layout, font="Any 18", title_color="White")],
    ]

    window = psg.Window("Forward/Inverse Kinematics", layout, finalize=True)

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


        isJogIncrementEvent = False
        incValue = 0
        if "Inc"in event or "Dec" in event:
            isJogIncrementEvent = True
            if event =="-Inc1-":
                incValue =1
            if event =="-Inc5-":
                incValue =5
            if event =="-Inc10-":
                incValue =10
            if event =="-Dec1-":
                incValue =-1
            if event =="-Dec5-":
                incValue =-5
            if event =="-Dec10-":
                incValue =-10
                
            print(isJogIncrementEvent, incValue)

        if isJogIncrementEvent:
            if values['-radio1-X-']:
                ui_xyzuvw_target[0] += incValue
                window['-x_curr-'].update("{:.2f}".format(ui_xyzuvw_target[0])) 
                isCarEvent=True
            if values['-radio1-Y-']:
                ui_xyzuvw_target[1] += incValue
                window['-y_curr-'].update("{:.2f}".format(ui_xyzuvw_target[1])) 
                isCarEvent=True
            if values['-radio1-Z-']:
                ui_xyzuvw_target[2] += incValue
                window['-z_curr-'].update("{:.2f}".format(ui_xyzuvw_target[2])) 
                isCarEvent=True
            if values['-radio1-Rz-']:
                ui_xyzuvw_target[3] += incValue
                window['-rz_curr-'].update("{:.2f}".format(ui_xyzuvw_target[3])) 
                isCarEvent=True
            if values['-radio1-Ry-']:
                ui_xyzuvw_target[4] += incValue
                window['-ry_curr-'].update("{:.2f}".format(ui_xyzuvw_target[4])) 
                isCarEvent=True
            if values['-radio1-Rx-']:
                ui_xyzuvw_target[5] += incValue
                window['-rx_curr-'].update("{:.2f}".format(ui_xyzuvw_target[5])) 
                isCarEvent=True
            
                
        print(values)
        
        
        
        if isJointEvent:
            print(ui_joints_target)
            ui_xyzuvw_target = kinematics.SolveFowardKinematic(ui_joints_target)
            update_xyzuvw_render = True
        
        if isCarEvent:
            print(ui_xyzuvw_target)
            ui_joints_target = kinematics.SolveInverseKinematic(ui_xyzuvw_target)
            update_joint_render = True

        # print(values)
        # cartesian_radio

        """ Last update """
        if update_xyzuvw_render:
            window["-x_curr-"].update("{:.2f}".format(ui_xyzuvw_target[0]))
            window["-y_curr-"].update("{:.2f}".format(ui_xyzuvw_target[1]))
            window["-z_curr-"].update("{:.2f}".format(ui_xyzuvw_target[2]))
            window["-rz_curr-"].update("{:.2f}".format(ui_xyzuvw_target[3]))
            window["-ry_curr-"].update("{:.2f}".format(ui_xyzuvw_target[4]))
            window["-rx_curr-"].update("{:.2f}".format(ui_xyzuvw_target[5]))


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
