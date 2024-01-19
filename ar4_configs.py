class AR4_CONFIG:
    CW = 0x00
    CCW = 0x01
    
    joint_1 = {
        'name': 'Joint-1-Shoulder-Swing',
        'motor_addr': 0x01,
        'enable': True,
        'microstep': 16,
        
        'home_method': 'endswitch', # 'endswitch' or 'sensorless'
        'home_direction': CW,
        'home_speed_rpm': 100,
        'home_timeout_ms': 20000,
    
    
        # Joint motion configs
        'motor_direction': CCW, # motion direction from homing position 
        'steps_per_rev': 200, # 200 = 360/1.8 motor step angle
        'gear_ratio': 10, # joint shaft's to step's gear ratio
        'steps_per_degree': 22.22222222, # steps needed relative to joint's 1 degree motion 
        
        'max_angle': 170,
        'min_angle': -170,
        'max_speed_rpm': 800, 
        'acc_and_dec': 80, # in percentage
        
        'calibration_zero_offset_angle': -8, # offset angle to joint zero position
    }

    joint_2 = {
        'name': 'Joint-2-Shouder-Rotate',
        'motor_addr': 0x02,
        'enable': True,
        'motor_direction': CW,
        'microstep': 16,
        
        'home_method': 'endswitch', # 'endswitch' or 'sensorless'
        'home_direction': CCW,
        'home_speed_rpm': 100,
        'home_timeout_ms': 20000,
    
    
        # Joint motion configs
        'motor_direction': CW, # motion direction from homing position 
        'steps_per_rev': 200, # 200 = 360/1.8 motor step angle
        'gear_ratio': 50, # joint shaft's to step's gear ratio
        'steps_per_degree': 27.77777778, # steps needed relative to joint's 1 degree motion 
        
        'max_angle': 90,
        'min_angle': -42,
        'max_speed_rpm': 60, 
        # 'max_speed_rpm': 300, 
        # 'acc_and_dec': 10, # in percentage
        'acc_and_dec': 80, # in percentage
        
        'calibration_zero_offset_angle': 2.85, # offset angle to joint zero position
    }

    joint_3 = {
        'name': 'Joint-3-Elbow-Swing',
        'motor_addr': 0x03,
        'enable': True,
        'microstep': 16,
        
        'home_method': 'endswitch', # 'endswitch' or 'sensorless'
        'home_direction': CCW,
        'home_speed_rpm': 300,
        'home_timeout_ms': 20000,
    
    
        # Joint motion configs
        'motor_direction': CW, # motion direction from homing position 
        'steps_per_rev': 200, # 200 = 360/1.8 motor step angle
        'gear_ratio': 50, # joint shaft's to step's gear ratio
        'steps_per_degree': 27.77777778, # steps needed relative to joint's 1 degree motion 
        
        'max_angle': 52,
        'min_angle': -89,
        'max_speed_rpm': 300, 
        'acc_and_dec': 80, # in percentage
        
        'calibration_zero_offset_angle': -30+2.85, # offset angle to joint zero position
    }

    joint_4 = {
        'name': 'Joint-4-Elbow-Rotate',
        'motor_addr': 0x04,
        'enable': True,
        'microstep': 16,
        
        'home_method': 'endswitch', # 'endswitch' or 'sensorless'
        'home_direction': CW,
        'home_speed_rpm': 100,
        'home_timeout_ms': 20000,
    
    
        # Joint motion configs
        'motor_direction': CCW, # motion direction from homing position 
        'steps_per_rev': 200, # 200 = 360/1.8 motor step angle
        'gear_ratio': 44.8, # joint shaft's to step's gear ratio
        'steps_per_degree': 21.77777778, # steps needed relative to joint's 1 degree motion 
        
        'max_angle': 90,
        'min_angle': -90,
        'max_speed_rpm': 300, 
        'acc_and_dec': 80, # in percentage
        
        'calibration_zero_offset_angle': -2.2, # offset angle to joint zero position
    }

    joint_5 = {
        'name': 'Joint-5-Wrist-Swing',
        'motor_addr': 0x05,
        'enable': True,
        'microstep': 16,
        
        'home_method': 'endswitch', # 'endswitch' or 'sensorless'
        'home_direction': CW,
        'home_speed_rpm': 50,
        'home_timeout_ms': 20000,
    
    
        # Joint motion configs
        'motor_direction': CCW, # motion direction from homing position 
        'steps_per_rev': 200, # 200 = 360/1.8 motor step angle
        'gear_ratio': 9.837111997, # joint shaft's to step's gear ratio
        'steps_per_degree': 5.465062221, # steps needed relative to joint's 1 degree motion 
        
        'max_angle': 105,
        'min_angle': -105,
        'max_speed_rpm': 80, 
        'acc_and_dec': 80, # in percentage
        
        'calibration_zero_offset_angle': -3.85, # offset angle to joint zero position

    }

    joint_6 = {
        'name': 'Joint-6-Wrist-Rotate',
        'motor_addr': 0x06,
        'enable': True,
        'microstep': 16,
        
        'home_method': 'endswitch', # 'endswitch' or 'sensorless'
        'home_direction': CW,
        'home_speed_rpm': 50,
        'home_timeout_ms': 20000,
    
    
        # Joint motion configs
        'motor_direction': CCW, # motion direction from homing position 
        'steps_per_rev': 200, # 200 = 360/1.8 motor step angle
        'gear_ratio': 20, # joint shaft's to step's gear ratio
        'steps_per_degree': 10.55555556, # steps needed relative to joint's 1 degree motion 
        
        'max_angle': 180-5,
        'min_angle': -180+5,
        'max_speed_rpm': 300, 
        'acc_and_dec': 80, # in percentage
        
        'calibration_zero_offset_angle': 3.9+2.4, # offset angle to joint zero position
    }

    joint_7 = {
        'name': 'Joint-7-Gripper',
        'motor_addr': 0x07,
        'enable': False,
        'microstep': 16,
        
        'home_method': 'endswitch', # 'endswitch' or 'sensorless'
        'home_direction': CCW,
        'home_speed_rpm': 100,
        'home_timeout_ms': 20000,
    
    
        # Joint motion configs
        'motor_direction': CW, # motion direction from homing position 
        'steps_per_rev': 200, # 200 = 360/1.8 motor step angle
        'gear_ratio': 50, # joint shaft's to step's gear ratio
        'steps_per_degree': 100, # steps needed relative to joint's 1 degree motion 
        
        'max_angle': 170,
        'min_angle': -170,
        'max_speed_rpm': 80, 
        'acc_and_dec': 80, # in percentage
        
        'calibration_zero_offset_angle': 0, # offset angle to joint zero position
    }

    joint_8 = {
        'name': 'Joint-8-Base-Slider',
        'motor_addr': 0x08,
        'enable': False,
        'microstep': 16,
        
        'home_method': 'endswitch', # 'endswitch' or 'sensorless'
        'home_direction': CCW,
        'home_speed_rpm': 100,
        'home_timeout_ms': 20000,
    
    
        # Joint motion configs
        'motor_direction': CW, # motion direction from homing position 
        'steps_per_rev': 200, # 200 = 360/1.8 motor step angle
        'gear_ratio': 50, # joint shaft's to step's gear ratio
        'steps_per_degree': 0, # steps needed relative to joint's 1 degree motion 
        
        'max_angle': 170,
        'min_angle': -170,
        'max_speed_rpm': 80, 
        'acc_and_dec': 80, # in percentage
        
        'calibration_zero_offset_angle': 0, # offset angle to joint zero position
    }

    joints  = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7, joint_8]
    homing_sequence = [[1,2,3,4,5],[6,7,8]] # split to avoid collision when toolend is attached
    dh_params = [
            [    0,    -90,  169.77,   64.2  ],
            [  -90,      0,       0,    305  ],
            [  180,     90,       0, -.0001  ],
            [    0,    -90,  222.63,      0  ],
            [    0,     90,       0,      0  ],
            [    0,      0,   36.25,      0  ]
        ]

AR4_CFG = AR4_CONFIG()
COM_PORT = 'COM6'
