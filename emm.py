"""
Created Date: 4 Jul 2022
Author: Edward Lai
Copyright (c) 2022 Vibe, Inc.
----------------------------------------------------------------------
This is a driver and implementation of the Vibe Audio Lab DUT Turntable.
The turentable is equipped with High performance FOC Servo Motor. Motor is communicated
through CAN Bus Protocol. The turntable is also equiped with an `Eathernet to CAN converter`.
Once the Converter's IP is setup properly. (see Datasheet  E810 DTU for more setup guide) You 
will be able to send and recieve Can bus message throught SocketCan/TCP Socket . The data format 
of the docket is also implemented here.   

Key Features to FOC Motor API
- Check Motor Status
- Check Encoder Position
- GET/SET PID value of speed/toruqe/position
- Control motor with Speed FOC
- Control motor with Toruqe FOC
- Control motor with Position FOC (This is what turntable uses)
- Other Misc Getters and Setters.


Foc Motor API is NOT thread save. Please make API request in sequential order.


    CAN PROTOCOL SPECS
    ⦁	CAN总线
    ⦁	参数
    总线接口：CAN
    波特率：1Mbps
    ⦁	报文格式
    标识符：单电机指令发送：0x140 + ID(1~32)
            多电机指令发送：0x280        
            回复：0x240 + ID(1~32)
    帧格式：数据帧
    帧类型：标准帧
    DLC：8字节
"""

import binascii
import time
import struct


# User changable constants
HOST = "192.168.51.210"
PORT = 8886  # Port to listen on (non-privileged ports are > 1023)

# todo: UDP broadcast device serach is not implemented.
# Search compatiable device over UDP
SEARCH_NAME = "turntable-dut"
SEARCH_PORT = 1901

DEVICE_MAC_ALLOW_LIST = ["aa-f3-ef-ed-f3-0c"]

CAN_BUDRATE = 5000000
# End User changable constants

# Fixed Constants
# DRIVER_VERSION="3.1"
# MOTOR_CAN_ID_SND=140
# MOTOR_CAN_ID_RCV=240
# ROUND_FACTOR = 92.55
# ROTATION_ANGLE_SCALE_FACTOR = ROUND_FACTOR/360
# PULSE_PER_CYCLE = 147397
# End Fixed Constants


def mapFromTo(x, a, b, c, d):
    y = (x - a) / (b - a) * (d - c) + c
    return y


def degree2pulse(degree) -> int:
    return mapFromTo(degree, 0, 360, 0, PULSE_PER_CYCLE)


def pulse2degree(pulse) -> int:
    pulse %= PULSE_PER_CYCLE
    return mapFromTo(pulse, 0, PULSE_PER_CYCLE, 0, 360)


class EMMV5_VERSION:
    HARDWARE = 0x78
    FIRMWARE = 0xF4

class EMMV5_COMMAND:
    CMD_W_MOTOR_ENABLE = 0xF3  # 电机使能控制
    CMD_W_SPEED_CTL_MODE = 0xF6  # 速度模式控制
    CMD_W_POSITION_CTL_MODE = 0xFD  # 位置模式控制
    CMD_W_STOP_NOW = 0xFE  # 立即停止
    CMD_W_BROADCAST_RUN = 0xFF  # 多机同步运动
    CMD_W_SET_SINGLE_ROTATION_ZERO_POS = 0x93  # 设置单圈回零的零点位置
    CMD_W_TRIGGER_ZEROING = 0x9A  # 触发回零
    CMD_W_FORCE_EXIT_ZEROING = 0x9C  # 强制中断并退出回零操作
    CMD_R_ZEROING_PARAMS = 0x22  # 读取原点回零参数
    CMD_W_ZEROING_PARAMS = 0x4C  # 修改原点回零参数
    CMD_R_ZEROING_STATUS = 0x3B  # 读取回零状态标志位
    CMD_W_SET_ENCODER_CAL = 0x06  # 触发编码器校准
    CMD_W_SET_CURRENT_POS_ZREO = 0x0A  # 将当前的位置角度清零
    CMD_W_CLEAR_STALL_ERROR = 0x0E  # 解除堵转保护
    CMD_W_RESET_FACTORY_SETTINGS = 0x0F  # 恢复出厂设置
    CMD_R_HARDWARE_FIRMWARE_VER = 0x1F  # 读取固件版本和对应的硬件版本
    CMD_R_PHASE_RES_IND = 0x20  # 读取相电阻和相电感
    CMD_R_PID_PARAMS = 0x21  # 读取位置环 PID 参数
    CMD_R_POWER_LINE_VOLTAGE = 0x24  # 读取总线电压
    CMD_R_PHASE_CURRENT = 0x27  # 读取相电流
    CMD_R_LINEAR_ENCODER = 0x31  # 读取经过线性化校准后的编码器值
    CMD_R_PULSE_CNT = 0x32  # 读取输入脉冲数
    CMD_R_TARGET_POS = 0x33  # 读取电机目标位置
    CMD_R_OPEN_LOOP_REALTIME_TARGET_POS = 0x34  # 读取电机实时设定的目标位置（开环模式的实时位置）
    CMD_R_RPM = 0x35  # 读取电机实时转速
    CMD_R_POS = 0x36  # 读取电机实时位置
    CMD_R_POS_ERROR = 0x37  # 读取电机位置误差
    CMD_R_MOTOR_STATUS = 0x3A  # 读取电机状态标志位
    CMD_R_DRIVER_PARAMS = 0x42  # 读取驱动配置参数
    CMD_R_SYSTEM_STATUS = 0x43  # 读取系统状态参数
    CMD_W_MICROSTEPS = 0x84  # 修改任意细分
    CMD_W_ID_ADDRESS = 0xAE  # 修改任意 ID 地址
    CMD_W_SEL_DRIVE_MODE = 0x46  # 切换开环/闭环模式（ P_Pul 菜单选项）
    CMD_W_OPENLOOP_CURRENT_LIMIT = 0x44  # 修改开环模式的工作电流
    CMD_W_DRIVER_PARAMS = 0x48  # 修改驱动配置参数
    CMD_W_PID_PARAMS = 0x4A  # 修改位置环 PID 参数
    CMD_W_DEFAULT_POWERON_SPEED_MODE = 0xF7  # 存储一组速度模式参数（方向、 速度、 加速度） ， 上电自动运行
    CMD_W_COM_SPEED = 0x4F  # 修改通讯控制的输入速度是否缩小 10 倍输入（S_Vel_IS 菜单选项）


class HwProtocolInterfaceBase:
    """
    Factory class for platform specific communication protocol implementation.
    """

    def __init__(self) -> None:
        pass

    def send(self, mesg):
        raise NotImplemented

    def recv(self):
        raise NotImplemented

    def send_and_recv(self, mesg):
        raise NotImplemented


class SerialPortDeiver(HwProtocolInterfaceBase):
    """
    Implements the Serial communication for generic platform that supports serial communication
    """

    def __init__(self, serial_instance) -> None:
        super().__init__()

        self.serial_ = serial_instance

    def send(self, mesg):
        self.serial_.write(mesg)
        return None

    def recv(self):
        data = self.serial_.read(40)
        if not data:
            return None
        return data

    def send_and_recv(self, mesg):
        self.send(mesg)
        data = self.recv()
        if not data:
            return None
        return data


class EMMV5_MOROT:
    """
    driver APIS for the EMM FOC Stepper Motor
    protocol version 5.0
    """

    def __init__(self, com_interface) -> None:
        self.com = com_interface
        self.crc_bit = 0x6B
        self.motor_id = 0x01

        self.multi_motor_sync = 0x01

    def send_and_recv(self, mesg):
        return self.com.send_and_recv(mesg)

    def send(self, mesg):
        return self.com.send(mesg)

    def broadcast_run(self):
        """
        Signal broadcast all holding commands to run.
        Motor commands had to be executed with `wait_broadcast_signal` flag enabled before calling this function.
        """

        cmd = EMMV5_COMMAND.CMD_W_BROADCAST_RUN

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray([0x00, cmd, 0x66, self.crc_bit])

        return _handle_resp(self.send_and_recv(mesg))

    def enable_motor(self, enable=True, wait_broadcast_signal=False):
        """
        Enabling and disabling motor
        """

        cmd = EMMV5_COMMAND.CMD_W_MOTOR_ENABLE

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        disable_cmd = bytearray(
            [self.motor_id, cmd, 0xAB, 0x00, wait_broadcast_signal, self.crc_bit]
        )
        enable_cmd = bytearray(
            [self.motor_id, cmd, 0xAB, 0x01, wait_broadcast_signal, self.crc_bit]
        )

        if enable:
            return _handle_resp(self.send_and_recv(enable_cmd))
        else:
            return _handle_resp(self.send_and_recv(disable_cmd))

    def set_speed_control(self, dir, speed, accel, wait_broadcast_signal=False):
        """
        motor_addr + 0xF6 + dir[1] + speed[2] + accel[1] + broadcast[1] + crc[1]

        Data format:
        - dir: (0x00=CW/ 0x01=CCW)
        - speed:  (0-5000)rmp
        - accel: 0x00-0xFF(0-255), 0 start imminently, 1-255 is the speed
        """

        cmd = EMMV5_COMMAND.CMD_W_SPEED_CTL_MODE

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        if dir == 0:
            dir = 0x00
        else:
            dir = 0x01

        if speed > 5000 or speed < 0:
            raise Exception("Speed out of range")
        if accel > 255 or accel < 0:
            raise Exception("Accel out of range")

        speed_fmt = struct.pack(">H", int(speed))
        mesg = (
            bytearray([self.motor_id, cmd, dir])
            + speed_fmt
            + bytearray([accel, wait_broadcast_signal, self.crc_bit])
        )
        print(mesg)
        return _handle_resp(self.send_and_recv(mesg))

    def set_position_control(
        self, dir, speed, accel, pulse, absolute_mode=True, wait_broadcast_signal=False
    ):
        """
        motor_addr + 0xFD + dir[1] + speed[2] +  accel[1] + pulse[4] + 相对/绝对模式标志[1] + broadcast[1] + crc[1]

        Data format:
        - dir: (0x00=CW/ 0x01=CCW)
        - speed:  (0-5000)rmp
        - accel: 0x00-0xFF(0-255), 0 start imminently, 1-255 is the speed
        - pulse: (4bytes) 0-147397
            (In 16 micro-setps sending  3200 step pulse = 1 rev, so 32000 pulse steps is 10 revs)
        - absolute_mode: (0x00=relative/ 0x01=absolute): moving relative to current position or from absolute zero
        """
        cmd = EMMV5_COMMAND.CMD_W_POSITION_CTL_MODE

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        if dir == 0:
            dir = 0x00
        else:
            dir = 0x01

        if absolute_mode > 1 or absolute_mode < 0:
            raise Exception("Absolute args out of range")
        if speed > 5000 or speed < 0:
            raise Exception("Speed out of range")
        if accel > 255 or accel < 0:
            raise Exception("Accel out of range")

        speed_fmt = struct.pack(">H", int(speed))
        pulse_fmt = struct.pack(">I", int(pulse))

        mesg = (
            bytearray([self.motor_id, cmd, dir])
            + speed_fmt
            + bytearray([accel])
            + pulse_fmt
            + bytearray([absolute_mode, wait_broadcast_signal, self.crc_bit])
        )

        return _handle_resp(self.send_and_recv(mesg))

    def stop_now(self, wait_broadcast_signal=False):
        cmd = EMMV5_COMMAND.CMD_W_STOP_NOW

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray(
            [self.motor_id, cmd, 0x98, wait_broadcast_signal, self.crc_bit]
        )
        return _handle_resp(self.send_and_recv(mesg))

    def set_single_ration_zero_position(self, save_rom=False):
        """power off the motor, manually rotate the motor to the zero position, power on the motor, then call this function to set the zero position"""
        raise NotImplemented

        cmd = EMMV5_COMMAND.CMD_W_SET_SINGLE_ROTATION_ZERO_POS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray(
            [self.motor_id, cmd, 0x98, wait_broadcast_signal, self.crc_bit]
        )
        return _handle_resp(self.send_and_recv(mesg))

    def read_zeroing_mode(self, wait_broadcast_signal=False):
        """
        命令格式： 地址 + 0x9A + 回零模式 + 多机同步标志 + 校验字节39
        命令返回： 地址 + 0x9A + 命令状态 + 校验字节
        命令示例： 发送 01 9A 00 00 6B， 正确返回 01 9A 02 6B， 条件不满足返回 01 9A
        E2 6B， 错误命令返回 01 00 EE 6B
        （条件不满足情况： 触发了堵转保护、 电机没使能、 单圈回零的零点位置值无效）
        数据解析： 设置完原点回零参数后， 可以发送该命令触发原点回零功能。 其中， 00
        表示触发单圈就近回零， 01 表示触发单圈方向回零， 02 表示触发多圈无限位碰撞回零，
        03 表示触发多圈有限位开关回零
        """
        raise NotImplemented

        cmd = EMMV5_COMMAND.CMD_W_TRIGGER_ZEROING

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray()
        return _handle_resp(self.send_and_recv(mesg))

    def set_single_ration_zero_position(self, save_rom=False):
        """命令格式： 地址 + 0x9C + 0x48 + 校验字节
        命令返回： 地址 + 0x9C + 命令状态 + 校验字节
        命令示例： 发送 01 9C 0x48 6B， 正 确返回 01 9C 02 6B， 条件不满足返回 01 9C E2
        6B， 错误命令返回 01 00 EE 6B
        （条件不满足的情况有： 当前没有触发回零操作）
        数据解析： 正在回零的过程中， 可以使用该命令强制中断并退出回零操作。"""

        raise NotImplemented

        cmd = EMMV5_COMMAND.CMD_W_FORCE_EXIT_ZEROING

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray()
        return _handle_resp(self.send_and_recv(mesg))

    def read_zeroing_params(self) -> dict:
        cmd = EMMV5_COMMAND.CMD_R_ZEROING_PARAMS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            print(mesg)
            zero_params_dic = {}

            # ZEROING_MODE: 00 表示触发单圈就近回零， 01 表示触发单圈方向回零， 02 表示触发多圈无限位碰撞回零，03 表示触发多圈有限位开关回零
            zero_params_dic["ZEROING_MODE"] = mesg[2]
            zero_params_dic["ZEROING_DIR"] = mesg[3]
            zero_params_dic["ZEROING_SPEED_RPM"] = struct.unpack(
                ">h", bytes(mesg[4:6])
            )[0]
            zero_params_dic["ZEROING_TIMEOUT_MS"] = struct.unpack(
                ">i", bytes(mesg[6:10])
            )[0]
            zero_params_dic["SENSORLESS_ZEROING_DETECT_SPEED_RPM"] = struct.unpack(
                ">h", bytes(mesg[10:12])
            )[0]
            zero_params_dic["SENSORLESS_ZEROING_DETECT_CURRENT_MA"] = struct.unpack(
                ">h", bytes(mesg[12:14])
            )[0]
            zero_params_dic["SENSORLESS_ZEROING_DETECT_TIME_MS"] = struct.unpack(
                ">h", bytes(mesg[14:16])
            )[0]
            zero_params_dic["POWERON_ZEROING_ENABLED"] = mesg[16]

            return zero_params_dic

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def write_zeroing_params(self, zero_params_dic: dict):
        """ """
        raise NotImplemented

        cmd = EMMV5_COMMAND.CMD_W_ZEROING_PARAMS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_zeroing_status(self):
        """
        命令功能： 读取回零状态标志位
        命令格式： 地址 + 0x3B + 校验字节
        命令返回： 地址 + 0x3B + 回零状态标志+ 校验字节
        命令示例： 发送 01 3B 6B， 正确返回 01 3B 回零状态标志字节 6B， 错误命令返回
        01 00 EE 6B
        数据解析： 返回的回零状态标志字节的每一位都代表一种状态， 比如返回的回零状
        态标志字节为 0x03， 它将按照下面的数据格式进行排列和数据的转换：
        编码器就绪状态标志位 = 0x03 & 0x01 = 0x01
        校准表就绪状态标志位 = 0x03 & 0x02 = 0x01
        正在回零标志位 = 0x03 & 0x04 = 0x00
        回零失败标志位 = 0x03 & 0x08 = 0x00
        """
        raise NotImplemented

        cmd = EMMV5_COMMAND.CMD_R_ZEROING_STATUS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def run_encoder_calibration(self):
        """
            命令格式： 地址 + 0x06 + 0x45 + 校验字节
            命令返回： 地址 + 0x06 + 状态码 + 校验字节
            命令示例： 发送 01 06 45 6B， 正确返回 01 06 02 6B， 条件不满足返回 01 06 E2 6B，
            错误命令返回 01 00 EE 6B
            （ 条件不满足的情况有： 当前是开环模式、 触发了堵转保护）
            数据解析： 触发编码器校准， 对应屏幕上的“ Cal” 菜单
        """
        raise NotImplemented

        cmd = EMMV5_COMMAND.CMD_W_SET_ENCODER_CAL

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def set_current_position_as_zero(self):
        """
        命令格式： 地址 + 0x0A + 0x6D + 校验字节
        命令返回： 地址 + 0x0A + 状态码 + 校验字节
        命令示例： 发送 01 0A 6D 6B， 正确返回 01 0A 02 6B， 错误命令返回 01 00 EE 6B
        数据解析： 将当前位置角度、 位置误差、 脉冲数等全部清零。
        """
        raise NotImplemented

        cmd = EMMV5_COMMAND.CMD_W_SET_CURRENT_POS_ZREO

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def clear_stall_error(self):
        """
        命令功能： 解除堵转保护
        命令格式： 地址 + 0x0E + 0x52 + 校验字节
        命令返回： 地址 + 0x0E + 状态码 + 校验字节
        命令示例： 发送 01 0E 52 6B， 正确返回 01 0E 02 6B， 条件不满足返回 01 0E E2 6B，
        错误命令返回 01 00 EE 6B
        （ 条件不满足的情况有： 没有触发到堵转保护）
        数据解析： 电机发生堵转后， 发送该命令可以解除堵转保护。
        """
        raise NotImplemented

        cmd = EMMV5_COMMAND.CMD_W_CLEAR_STALL_ERROR

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def factory_reset(self):
        """

        """
        raise NotImplemented

        cmd = EMMV5_COMMAND.CMD_W_RESET_FACTORY_SETTINGS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))
    
    def read_hardware_version(self):
        """

        """
        # raise NotImplemented

        cmd = EMMV5_COMMAND.CMD_R_HARDWARE_FIRMWARE_VER

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            print(hex(mesg[0]), hex(mesg[1]), hex(mesg[2]), hex(mesg[3]), hex(mesg[4]))
            sw_rev = int(mesg[2])
            hw_rev = int(mesg[3])
            return {"sw_rev": sw_rev, "hw_rev": hw_rev}

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))
    
    def read_phase_resistance_and_inductance(self):
        """

        """
        raise NotImplemented

        cmd = EMMV5_COMMAND.CMD_R_PHASE_RES_IND

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_pid(self)->dict:
        """
        MOROR ID + CMD + [4] KP + [4] KI + [4] KD
        """
        cmd = EMMV5_COMMAND.CMD_R_PID_PARAMS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            pid_dic = {}
            pid_dic["KP"] = struct.unpack(">i", bytes(mesg[2:6]))[0]
            pid_dic["KI"] = struct.unpack(">i", bytes(mesg[6:10]))[0]
            pid_dic["KD"] = struct.unpack(">i", bytes(mesg[10:14]))[0]
            return pid_dic

        mesg = bytearray([0x01, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_vbus_voltage_mV(self):
        cmd = EMMV5_COMMAND.CMD_R_POWER_LINE_VOLTAGE

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            voltage_mV = struct.unpack(">h", bytes(mesg[2:4]))[0]
            return voltage_mV

        mesg = bytearray([0x01, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    # CMD_R_PHASE_CURRENT
    # CMD_R_LINEAR_ENCODER
    # CMD_R_PULSE_CNT
    # CMD_R_TARGET_POS
    # CMD_R_OPEN_LOOP_REALTIME_TARGET_POS
    # CMD_R_RPM
    # CMD_R_POS
    # CMD_R_POS_ERROR
    # CMD_R_MOTOR_STATUS
    # CMD_R_DRIVER_PARAMS
    # CMD_R_SYSTEM_STATUS
    # CMD_W_MICROSTEPS
    # CMD_W_ID_ADDRESS
    # CMD_W_SEL_DRIVE_MODE
    # CMD_W_OPENLOOP_CURRENT_LIMIT
    # CMD_W_DRIVER_PARAMS
    # CMD_W_PID_PARAMS
    # CMD_W_DEFAULT_POWERON_SPEED_MODE
    # CMD_W_COM_SPEED


def main():
    import serial

    ser = serial.Serial("COM6", 115200, timeout=0.05)

    em1 = EMMV5_MOROT(SerialPortDeiver(ser))
    print(em1.read_pid())
    # while(1):
    #     em1.set_position_control(1, 1000, 202, 2000, absolute_mode=False)
    #     time.sleep(1)
    #     em1.set_position_control(0, 1000, 202, 2000, absolute_mode=False)
    #     time.sleep(1)

    print(em1.read_hardware_version())


import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process some integers.")
    parser.add_argument(
        "--ip", type=str, default=HOST, help="ip address of the can bus"
    )
    parser.add_argument("--port", type=int, default=PORT, help="port of the can bus")
    parser.add_argument(
        "--setangle", type=int, help="set the angle of the turntable", default=None
    )

    parser.add_argument(
        "--calibrate", action="store_true", help="calibrate the turntable"
    )
    parser.add_argument(
        "--test_sequence",
        action="store_true",
        help="test sequence [0-360] rotate in 45 degree increments",
    )

    args = parser.parse_args()

    try:
        main()
        # main(args.ip, args.port, args.setangle, args.calibrate, demo=args.test_sequence)
        print("Done")
    except Exception as e:
        print("Failed, err: ", e)
