class AR4_CONFIG:
    CW = 0x00
    CCW = 0x01
    
    joint_1 = {
        'name': 'Joint-Gripper',
        'motor_addr': 0x01,
        'enable': True,
        'microstep': 1,
        
        'home_method': 'endswitch', # 'endswitch' or 'sensorless'
        'home_direction': CW,
        'home_speed_rpm': 100,
        'home_timeout_ms': 20000,
    
        'flip_joint_direction': True,
        # Joint motion configs
        'motor_direction': CCW, # motion direction from homing position 
        'steps_per_rev': 200, # 200 = 360/1.8 motor step angle
        'gear_ratio': 10, # joint shaft's to step's gear ratio
        'steps_per_degree': 22.22222222, # steps needed relative to joint's 1 degree motion 
        
        'max_angle': 170,
        'min_angle': -145,
        'max_speed_rpm': 800, 
        'acc_and_dec': 80, # in percentage
        
        'calibration_zero_offset_angle': -8, # offset angle to joint zero position
    }

    # joints  = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7, joint_8]
    joints  = [joint_1]

AR4_CFG = AR4_CONFIG()
COM_PORT = 'COM3'
