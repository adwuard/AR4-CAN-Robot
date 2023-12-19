class RobotConfigs:
    joint_1 = {
        'name': 'Joint-1-Shouder-Swing',
        'id': 0x01,
        'enabled': True,
        'motor_direction': 0, # 1 or -1
        'microstep': 128,
        'steps_per_rev': 200,
        'gear_ratio': 1,
        'home_offset': 0,
        'home_direction': 1,
        'home_speed': 100,
        'home_accel': 100,
        'acceleration': 100,
        'min_speed': 20,  
        'max_speed': 254,
        'home_method': 'endswitch',
        'max_angle': 170,
        'min_angle': -170,

    }

    joint_2 = {
        'name': 'Joint-2-Shouder-Rotate',
        'id': 0x02,
        'enabled': True,
        'motor_direction': 1, # 1 or -1
        'microstep': 128,
        'steps_per_rev': 200,
        'gear_ratio': 1,
        'home_offset': 0,
        'home_direction': 0,
        'home_speed': 100,
        'home_accel': 100,
        'acceleration': 100,
        'min_speed': 20,  
        'max_speed': 100,
        'home_method': 'endswitch',
        'max_angle': 90,
        'min_angle': -42,
    }

    joint_3 = {
        'name': 'Joint-3-Elbow-Swing',
        'id': 0x03,
        'enabled': True,
        'microstep': 128,
        'steps_per_rev': 200,
        'gear_ratio': 1,
        'home_offset': 0,
        'home_direction': 1,
        'home_speed': 100,
        'home_accel': 100,
        'acceleration': 100,
        'min_speed': 20,  
        'max_speed': 100,
        'home_method': 'endswitch',
        'max_angle': 52,
        'min_angle': -89,

    }

    joint_4 = {
        'name': 'Joint-4-Elbow-Rotate',
        'id': 0x04,
        'enabled': True,
        'motor_direction': 1, # 1 or -1
        'microstep': 128,
        'steps_per_rev': 200,
        'gear_ratio': 1,
        'home_offset': 0,
        'home_direction': 0,
        'home_speed': 100,
        'home_accel': 100,
        'acceleration': 100,
        'min_speed': 20,  
        'max_speed': 100,
        'home_method': 'endswitch',
        'max_angle': 165,
        'min_angle': -165,
    }

    joint_5 = {
        'name': 'Joint-5-Wrist-Swing',
        'id': 0x05,
        'enabled': True,
        'motor_direction': 1, # 1 or -1
        'microstep': 128,
        'steps_per_rev': 200,
        'gear_ratio': 1,
        'home_offset': 0,
        'home_direction': 0,
        'home_speed': 100,
        'home_accel': 100,
        'acceleration': 100,
        'min_speed': 20,  
        'max_speed': 100,
        'home_method': 'endswitch',
        'max_angle': 105,
        'min_angle': -105,

    }

    joint_6 = {
        'name': 'Joint-6-Wrist-Rotate',
        'id': 0x06,
        'enabled': True,
        'motor_direction': 1, # 1 or -1
        'microstep': 128,
        'steps_per_rev': 200,
        'gear_ratio': 1,
        'home_offset': 0,
        'home_direction': 1,
        'home_speed': 100,
        'home_accel': 100,
        'acceleration': 100,
        'min_speed': 20,  
        'max_speed': 100,
        'home_method': 'endswitch',
        'max_angle': 155,
        'min_angle': -155,
    }

    joint_7 = {
        'name': 'Joint-7-Gripper',
        'id': 0x07,
        'enabled': True,
        'motor_direction': 1, # 1 or -1
        'microstep': 128,
        'steps_per_rev': 200,
        'gear_ratio': 1,
        'home_offset': 0,
        'home_direction': 0,
        'home_speed': 100,
        'home_accel': 100,
        'acceleration': 100,
        'min_speed': 20,  
        'max_speed': 100,
        'home_method': 'endswitch',
        'max_angle': 3333,
        'min_angle': 0,
    }

    joint_8 = {
        'name': 'Joint-8-Base-Slider',
        'id': 0x08,
        'enabled': False,
        'motor_direction': 1, # 1 or -1
        'microstep': 128,
        'steps_per_rev': 200,
        'gear_ratio': 1,
        'home_offset': 0,
        'home_direction': 0,
        'home_speed': 100,
        'home_accel': 100,
        'acceleration': 100,
        'min_speed': 20,  
        'max_speed': 100,
        'home_method': 'endswitch',
        'max_angle': 3333,
        'min_angle': 0,
    }


    robot_configs = {
        'joints': [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7, joint_8],
        'homeing_sequence': [[1,2,3,4,5],[6,7,8]],
        'dh_params': [
            [    0,    -90,  169.77,   64.2  ],
            [  -90,      0,       0,    305  ],
            [  180,     90,       0, -.0001  ],
            [    0,    -90,  222.63,      0  ],
            [    0,     90,       0,      0  ],
            [    0,      0,   36.25,      0  ]
        ],
        
        
    }
