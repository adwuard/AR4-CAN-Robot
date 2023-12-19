"""
Created Date: 19 Dec 2023
Author: Edward Lai
Copyright (c) 2023 Edward Lai.
----------------------------------------------------------------------
"""

import time
import struct


class EMMV5_VERSION:
    HARDWARE = 0x78
    FIRMWARE = 0xF4

class EMMV5_COMMAND:
    CMD_W_MOTOR_ENABLE = 0xF3
    CMD_W_SPEED_CTL_MODE = 0xF6
    CMD_W_POSITION_CTL_MODE = 0xFD
    CMD_W_STOP_NOW = 0xFE
    CMD_W_BROADCAST_RUN = 0xFF
    CMD_W_SET_SINGLE_ROTATION_ZERO_POS = 0x93
    CMD_W_TRIGGER_ZEROING = 0x9A
    CMD_W_FORCE_EXIT_ZEROING = 0x9C
    CMD_R_ZEROING_PARAMS = 0x22
    CMD_W_ZEROING_PARAMS = 0x4C
    CMD_R_ZEROING_STATUS = 0x3B
    CMD_W_SET_ENCODER_CAL = 0x06
    CMD_W_SET_CURRENT_POS_ZREO = 0x0A
    CMD_W_CLEAR_STALL_ERROR = 0x0E
    CMD_W_RESET_FACTORY_SETTINGS = 0x0F
    CMD_R_HARDWARE_FIRMWARE_VER = 0x1F
    CMD_R_PHASE_RES_IND = 0x20
    CMD_R_PID_PARAMS = 0x21
    CMD_R_POWER_LINE_VOLTAGE = 0x24
    CMD_R_PHASE_CURRENT = 0x27
    CMD_R_LINEAR_ENCODER = 0x31
    CMD_R_PULSE_CNT = 0x32
    CMD_R_TARGET_POS = 0x33
    CMD_R_OPEN_LOOP_REALTIME_TARGET_POS = 0x34
    CMD_R_RPM = 0x35
    CMD_R_POS = 0x36
    CMD_R_POS_ERROR = 0x37
    CMD_R_MOTOR_STATUS = 0x3A
    CMD_R_DRIVER_PARAMS = 0x42
    CMD_R_SYSTEM_STATUS = 0x43
    CMD_W_MICROSTEPS = 0x84
    CMD_W_ID_ADDRESS = 0xAE
    CMD_W_SEL_DRIVE_MODE = 0x46
    CMD_W_OPENLOOP_CURRENT_LIMIT = 0x44
    CMD_W_DRIVER_PARAMS = 0x48
    CMD_W_PID_PARAMS = 0x4A
    CMD_W_DEFAULT_POWERON_RUN = 0xF7
    CMD_W_COM_DATA_SCALE = 0x4F


class HwProtocolInterfaceBase:
    """
    Base class for platform specific communication protocol implementation.
    """

    def __init__(self) -> None:
        pass

    def send(self, mesg):
        raise NotImplemented

    def recv(self):
        raise NotImplemented

    def send_and_recv(self, mesg):
        raise NotImplemented


class SerialPort(HwProtocolInterfaceBase):
    """
    Implements the Serial communication for platforms that supports lib `serial` serial communication
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


class ZDT_EMMV5_MOTOR:
    """
    driver APIS for the EMM FOC Stepper Motor
    protocol version 5.0
    """

    def __init__(self, com_interface, motor_id=0x00) -> None:
        self.com = com_interface
        self.crc_bit = 0x6B
        self.motor_id = motor_id

        self.multi_motor_sync = 0x01

    def send_and_recv(self, mesg):
        return self.com.send_and_recv(mesg)

    def send(self, mesg):
        return self.com.send(mesg)

    def set_id(self, motor_id):
        self.motor_id = motor_id
        
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
    
    def disable_motor(self, wait_broadcast_signal=False):
        self.enable_motor(enable=False, wait_broadcast_signal=wait_broadcast_signal)
        
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
        motor_addr + 0xFD + dir[1] + speed[2] +  accel[1] + pulse[4] + relative/absolute flag[1] + broadcast[1] + crc[1]

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

    def set_single_rotation_zero_position(self):
        """
        Power off the motor, manually rotate the motor to the zero position, power on the motor,
        then call this function to set the zero position, this only capable of single rev zeroing,
        no multi revs.
        """

        cmd = EMMV5_COMMAND.CMD_W_SET_SINGLE_ROTATION_ZERO_POS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray([self.motor_id, cmd, 0x98, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def trigger_nearest_zeroing(self, wait_broadcast_signal=False):
        cmd = EMMV5_COMMAND.CMD_W_TRIGGER_ZEROING

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray(
            [self.motor_id, cmd, 0x00, wait_broadcast_signal, self.crc_bit]
        )
        return _handle_resp(self.send_and_recv(mesg))

    def trigger_direction_zeroing(self, wait_broadcast_signal=False):
        cmd = EMMV5_COMMAND.CMD_W_TRIGGER_ZEROING

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray(
            [self.motor_id, cmd, 0x01, wait_broadcast_signal, self.crc_bit]
        )
        return _handle_resp(self.send_and_recv(mesg))

    def trigger_sensorless_zeroing(self, wait_broadcast_signal=False):
        cmd = EMMV5_COMMAND.CMD_W_TRIGGER_ZEROING

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray(
            [self.motor_id, cmd, 0x02, wait_broadcast_signal, self.crc_bit]
        )
        return _handle_resp(self.send_and_recv(mesg))

    def trigger_sensor_zeroing(self, wait_broadcast_signal=False):
        cmd = EMMV5_COMMAND.CMD_W_TRIGGER_ZEROING

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray(
            [self.motor_id, cmd, 0x03, wait_broadcast_signal, self.crc_bit]
        )
        return _handle_resp(self.send_and_recv(mesg))

    def abort_zeroing(self):
        cmd = EMMV5_COMMAND.CMD_W_FORCE_EXIT_ZEROING

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray([self.motor_id, cmd, 0x48, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_zeroing_params(self) -> dict:
        cmd = EMMV5_COMMAND.CMD_R_ZEROING_PARAMS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            zero_params_dic = {}

            # ZEROING_MODE: 00 表示触发单圈就近回零， 01 表示触发单圈方向回零， 02 表示触发多圈无限位碰撞回零，03 表示触发多圈有限位开关回零
            if mesg[2] == 0x00:
                zero_params_dic["ZEROING_MODE"] = "nearest"
            elif mesg[2] == 0x01:
                zero_params_dic["ZEROING_MODE"] = "direction"
            elif mesg[2] == 0x02:
                zero_params_dic["ZEROING_MODE"] = "sensorless"
            elif mesg[2] == 0x03:
                zero_params_dic["ZEROING_MODE"] = "sensor"

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

    def write_zeroing_params(self, zero_params_dic: dict, save_rom=False):
        cmd = EMMV5_COMMAND.CMD_W_ZEROING_PARAMS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        if zero_params_dic["ZEROING_MODE"] == "nearest":
            arg1 = 0x00
        elif zero_params_dic["ZEROING_MODE"] == "direction":
            arg1 = 0x01
        elif zero_params_dic["ZEROING_MODE"] == "sensorless":
            arg1 = 0x02
        elif zero_params_dic["ZEROING_MODE"] == "sensor":
            arg1 = 0x03
        else:
            raise Exception("ZEROING_MODE not valid")

        arg2 = zero_params_dic["ZEROING_DIR"]
        arg3 = struct.pack(">h", int(zero_params_dic["ZEROING_SPEED_RPM"]))
        arg4 = struct.pack(">I", int(zero_params_dic["ZEROING_TIMEOUT_MS"]))
        arg5 = struct.pack(
            ">h", int(zero_params_dic["SENSORLESS_ZEROING_DETECT_SPEED_RPM"])
        )
        arg6 = struct.pack(
            ">h", int(zero_params_dic["SENSORLESS_ZEROING_DETECT_CURRENT_MA"])
        )
        arg7 = struct.pack(
            ">h", int(zero_params_dic["SENSORLESS_ZEROING_DETECT_TIME_MS"])
        )
        arg8 = zero_params_dic["POWERON_ZEROING_ENABLED"]

        mesg = (
            bytearray([self.motor_id, cmd, 0xAE, save_rom, arg1, arg2])
            + arg3
            + arg4
            + arg5
            + arg6
            + arg7
            + bytearray([arg8, self.crc_bit])
        )
        return _handle_resp(self.send_and_recv(mesg))

    def read_zeroing_status(self):
        cmd = EMMV5_COMMAND.CMD_R_ZEROING_STATUS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            status = mesg[2]

            flags = {}
            flags["ENCODER_READY"] = bool(status & 0x01)
            flags["CAL_TABLE_READY"] = bool(status & 0x02)
            flags["ZEROING_WORKING"] = bool(status & 0x04)
            flags["ZEROING_FAILED"] = bool(status & 0x08)
            return flags

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def run_encoder_calibration(self):
        """
        trigger encoder calibration, only needed on new encoder or magnet installation or after motor disassembly.
        """

        cmd = EMMV5_COMMAND.CMD_W_SET_ENCODER_CAL

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray([self.motor_id, cmd, 0x45, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def set_current_position_as_zero(self):
        """
        reset current angle, position error, and pulse coun to zero
        """
        cmd = EMMV5_COMMAND.CMD_W_SET_CURRENT_POS_ZREO

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray([self.motor_id, cmd, 0x6D, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def clear_stall_error(self):
        """
        If motor stall happens, this function can be used to clear the stall error.
        If the motor is not stalled, this function will return error.
        """

        cmd = EMMV5_COMMAND.CMD_W_CLEAR_STALL_ERROR

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        mesg = bytearray([self.motor_id, cmd, 0x52, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def factory_reset(self):
        """
        Perform driver board factory reset. Reboot after reset is required
        """
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
        Read firmware and hardware version from the motor and perform version check
        """
        cmd = EMMV5_COMMAND.CMD_R_HARDWARE_FIRMWARE_VER

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            sw_version = int(mesg[2])
            hw_version = int(mesg[3])

            return {"sw_version": sw_version, "hw_version": hw_version}

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def hardware_check(self):
        """
        Note: This is just a quick hardware version/firmware version check.
            Other hardware/firmware version is not fully tested, it may also work.
        """
        rev = self.read_hardware_version()
        if (
            rev["sw_version"] < EMMV5_VERSION.FIRMWARE
            or rev["hw_version"] < EMMV5_VERSION.HARDWARE
        ):
            raise Exception("Firmware/Hardware driver version not compatiable")
        return True

    def read_phase_resistance_and_inductance(self):
        cmd = EMMV5_COMMAND.CMD_R_PHASE_RES_IND

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            dic = {}
            dic["PHASE_RESISTENCE_mOhm"] = struct.unpack(">h", bytes(mesg[2:4]))[0]
            dic["PHASE_INDUCTENCE_uH"] = struct.unpack(">h", bytes(mesg[4:6]))[0]
            return dic

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_pid(self) -> dict:
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

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
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

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_phase_current_mA(self):
        cmd = EMMV5_COMMAND.CMD_R_PHASE_CURRENT

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            phase_current_mA = struct.unpack(">h", bytes(mesg[2:4]))[0]
            return phase_current_mA

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_linear_encoder(self):
        """
        Absolute encoder raw data 12bit data per revolution
        """
        cmd = EMMV5_COMMAND.CMD_R_LINEAR_ENCODER

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            steps = struct.unpack(">h", bytes(mesg[2:4]))[0]
            return steps

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_pulse_count(self):
        cmd = EMMV5_COMMAND.CMD_R_PULSE_CNT

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            isNeg = mesg[2]
            pulse_count = struct.unpack(">i", bytes(mesg[3:7]))[0]

            if isNeg == 0x01:
                pulse_count = -pulse_count

            return pulse_count

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_target_position_degree(self):
        cmd = EMMV5_COMMAND.CMD_R_TARGET_POS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            isNeg = mesg[2]
            degree_count = struct.unpack(">i", bytes(mesg[3:7]))[0]

            if isNeg == 0x01:
                degree_count = -degree_count

            return (degree_count * 360) / 65536

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_openloop_rt_target_position(self):
        cmd = EMMV5_COMMAND.CMD_R_OPEN_LOOP_REALTIME_TARGET_POS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            isNeg = mesg[2]
            degree_count = struct.unpack(">i", bytes(mesg[3:7]))[0]

            if isNeg == 0x01:
                degree_count = -degree_count

            return (degree_count * 360) / 65536

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_rpm(self):
        cmd = EMMV5_COMMAND.CMD_R_RPM

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            isNeg = mesg[2]
            rpm = struct.unpack(">h", bytes(mesg[3:5]))[0]

            if isNeg == 0x01:
                rpm = -rpm

            return rpm

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_pos(self):
        cmd = EMMV5_COMMAND.CMD_R_POS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            isNeg = mesg[2]
            motor_rt_pos_cnt = struct.unpack(">i", bytes(mesg[3:7]))[0]

            if isNeg == 0x01:
                motor_rt_pos_cnt = -motor_rt_pos_cnt

            return (motor_rt_pos_cnt * 360) / 65536

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_pos_error(self):
        cmd = EMMV5_COMMAND.CMD_R_POS_ERROR

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            isNeg = mesg[2]
            error = struct.unpack(">i", bytes(mesg[3:7]))[0]

            if isNeg == 0x01:
                error = -error

            return (error * 360) / 65536

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))

    def read_motor_status_flags(self):
        cmd = EMMV5_COMMAND.CMD_R_MOTOR_STATUS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            status = mesg[2]

            flags = {}
            flags["MOTOR_ENABLED"] = bool(status & 0x01)
            flags["MOTOR_ARRIVED_TARGET"] = bool(status & 0x02)
            flags["MOTOR_STALL"] = bool(status & 0x04)
            flags["MOTOR_STALL_PROTECT_ENABLED"] = bool(status & 0x08)
            return flags

        mesg = bytearray([self.motor_id, cmd, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))
    
    def read_driver_status(self):
        cmd = EMMV5_COMMAND.CMD_R_DRIVER_PARAMS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")

            motor_params_dic = {}
            motor_params_dic['PACKET_SIZE'] = mesg[2]
            motor_params_dic['TOTAL_ATTRS'] = mesg[3]
            if attr_dic['PACKET_SIZE'] != 0x21 or attr_dic['TOTAL_ATTRS'] != 0x15:
                raise Exception("Driver status packet size not correct, firmware version in-compatible")
            
            # TODO: Type parse
            motor_params_dic['MOTOR_TYPE'] = mesg[4]
            motor_params_dic['PULSE_PORT_MODE'] = mesg[5]
            motor_params_dic['COM_PORT_MODE'] = mesg[6]
            motor_params_dic['ENABLE_PIN_LEVEL'] = mesg[7]
            motor_params_dic['DIR_PIN_ACTIVE_HIGH_DIR'] = mesg[8]
            motor_params_dic['MICROSTEP_MODE'] = mesg[9]
            motor_params_dic['MICROSTEPPING_ENABLE'] = mesg[10]
            motor_params_dic['AUTO_SCREEN_OFF'] = mesg[11]
            motor_params_dic['OPENLOOP_CURRENT'] = struct.unpack(">h", bytes(mesg[12:14]))[0]
            motor_params_dic['CLOSEDLOOP_STALL_MAX_CURRENT'] = struct.unpack(">h", bytes(mesg[14:16]))[0]
            motor_params_dic['CLOSEDLOOP_MAX_CURRENT'] = struct.unpack(">h", bytes(mesg[16:18]))[0]
            motor_params_dic['COM_BUDRATE'] = mesg[19]
            motor_params_dic['ID_ADDRESS'] = mesg[20]
            motor_params_dic['CRC_MODE'] = mesg[21]
            motor_params_dic['RESPONSE_MODE'] = mesg[22]
            motor_params_dic['STALL_PROTECTION_ENABLED'] = mesg[23]
            motor_params_dic['STALL_SPEED_THRESHOLD_RPM'] = struct.unpack(">h", bytes(mesg[24:26]))[0]
            motor_params_dic['STALL_CURRENT_THRESHOLD_mA'] = struct.unpack(">h", bytes(mesg[26:28]))[0]
            motor_params_dic['STALL_TIME_THRESHOLD_ms'] = struct.unpack(">h", bytes(mesg[28:30]))[0]
            motor_params_dic['ARRIVED_WINDOW_ANGLE'] = mesg[31]
            
            return motor_params_dic

        mesg = bytearray([self.motor_id, cmd, 0x6C, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))
    

    
    def read_system_status(self):
        """ 
        All registers can be access with it's specific get/set functions 
        calls, this function should be debug purpose only. NotImplemented for now
        """
        cmd = EMMV5_COMMAND.CMD_R_SYSTEM_STATUS
        raise NotImplementedError
    
    
        
    def set_microsteps(self, usteps, save_rom=False):
            cmd = EMMV5_COMMAND.CMD_W_MICROSTEPS

            def _handle_resp(mesg):
                if not mesg:
                    return None
                if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                    raise Exception("Command response error")
                return True

            if usteps > 256 or usteps < 0:
                raise Exception("args out of range")

            if usteps ==256:
                reg = 0x00
            else:
                reg = usteps

            mesg = bytearray([self.motor_id, cmd, 0x8A, save_rom, reg, self.crc_bit])
            return _handle_resp(self.send_and_recv(mesg))
        


    def change_motor_id(self, new_id, save_rom=False):
        cmd = EMMV5_COMMAND.CMD_W_ID_ADDRESS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True

        if new_id > 128 or new_id < 0:
            raise Exception("args out of range")

        mesg = bytearray([self.motor_id, cmd, 0x4B, save_rom, new_id, self.crc_bit])
        resp =  _handle_resp(self.send_and_recv(mesg))
        
        self.motor_id = new_id
    
        return resp
    
    
    def set_driver_to_open_loop(self, save_rom=False):
        cmd = EMMV5_COMMAND.CMD_W_SEL_DRIVE_MODE

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True
        
        mesg = bytearray([self.motor_id, cmd, 0x69, save_rom, 0x01, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))
    
    def set_driver_to_close_loop(self, save_rom=False):
        cmd = EMMV5_COMMAND.CMD_W_SEL_DRIVE_MODE

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg) - 1] != self.crc_bit:
                raise Exception("Command response error")
            return True
        
        mesg = bytearray([self.motor_id, cmd, 0x69, save_rom, 0x02, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))
    
        
    def set_openloop_current_limit(self, current_mA, save_rom=False):
        cmd = EMMV5_COMMAND.CMD_W_OPENLOOP_CURRENT_LIMIT

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd):
                raise Exception("Command response error")
            return True

        if current_mA > 10000 or current_mA < 0:
            raise Exception("args out of range")

        current_fmt = struct.pack(">h", int(current_mA))

        mesg = (
            bytearray([self.motor_id, cmd, 0x33, save_rom])
            + current_fmt
            + bytearray([self.crc_bit])
        )
        return _handle_resp(self.send_and_recv(mesg))
    
    
    def set_driver_params(self, motor_params_dic, save_rom=False):
        cmd = EMMV5_COMMAND.CMD_W_DRIVER_PARAMS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd):
                raise Exception("Command response error")
            return True
        raise NotImplementedError
    
    
    def set_pid(self, kp, ki, kd, save_rom=False):
        cmd = EMMV5_COMMAND.CMD_W_PID_PARAMS

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd):
                raise Exception("Command response error")
            return True

        if kp > 100000 or kp < 0:
            raise Exception("args out of range")
        if ki > 100000 or ki < 0:
            raise Exception("args out of range")
        if kd > 100000 or kd < 0:
            raise Exception("args out of range")

        kp_fmt = struct.pack(">i", int(kp))
        ki_fmt = struct.pack(">i", int(ki))
        kd_fmt = struct.pack(">i", int(kd))

        mesg = (
            bytearray([self.motor_id, cmd, 0xC3, save_rom])
            + kp_fmt
            + ki_fmt
            + kd_fmt
            + bytearray([self.crc_bit])
        )
        return _handle_resp(self.send_and_recv(mesg))
    
    
    
    def set_default_poweron_run(self, dir, speed, accel, use_en_pin_to_stop=False, save_rom=False):
        
        cmd = EMMV5_COMMAND.CMD_W_DEFAULT_POWERON_RUN

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd):
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
        
        mesg = bytearray([self.motor_id, cmd, 0x1C, save_rom, dir])+speed_fmt+bytearray([accel, use_en_pin_to_stop, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))


    def set_data_scale(self, enable_x10_scale=False, save_rom=False):
        """
        speed, angle, rpm reading will down scaled by 10
        ie: 1rpm will be 0.1rpm
        """
        cmd = EMMV5_COMMAND.CMD_W_COM_DATA_SCALE

        def _handle_resp(mesg):
            if not mesg:
                return None
            if mesg[1] != int(cmd) or mesg[len(mesg)-1] != self.crc_bit:
                raise Exception("Command response error")
            return True
        
        mesg = bytearray([self.motor_id, cmd, 0x71, save_rom, enable_x10_scale, self.crc_bit])
        return _handle_resp(self.send_and_recv(mesg))
    


def main():
    serial_backend = serial.Serial("COM6", 115200, timeout=0.05)
    interface = SerialPort(serial_backend)
    motor1_id = 0x01 
    motor = ZDT_EMMV5_MOTOR(interface, motor1_id)
    
    
    print(motor.read_pid())
    motor.set_current_position_as_zero()
    motor.enable_motor()

    print(motor.read_motor_status_flags())
    
    
    # motor.change_motor_id(0x03, save_rom=True)
    motor.set_position_control(1, 2000, 244, 2000, absolute_mode=False)
    
    # motor.set_openloop_current_limit(100, save_rom=False)
    
    # while(1):
    #     time.sleep(1)
        # print(motor.read_pos(), motor.read_pulse_count(), motor.read_pos_error())
    #     motor.set_position_control(0, 2000, 244, 2000, absolute_mode=False)
    #     time.sleep(1)
    #     print(motor.read_pos(), motor.read_pulse_count(), motor.read_pos_error())

    # motor.enable_motor(enable=False)

    # while 1:
    # print(motor.read_linear_encoder())
    # time.sleep(0.2)

    # print(motor.read_zeroing_params())

    # params = motor.read_zeroing_params()
    # params["ZEROING_MODE"] = "direction"
    # params["ZEROING_SPEED_RPM"] = 31
    # print(motor.write_zeroing_params(params))

    # print("status", motor.read_zeroing_status())
    # motor.trigger_sensor_zeroing()
    # print("status", motor.read_zeroing_status())
    # time.sleep(1)
    # motor.abort_zeroing()
    # print("status", motor.read_zeroing_status())


    # print("new id", motor.)
    
    # dict = motor.read_driver_status()
    # for k in dict:
        # print(k, "--> ", dict[k])
    

if __name__ == "__main__":
    import serial
    import argparse
    
    parser = argparse.ArgumentParser(description="Process some integers.")
    # parser.add_argument(
    # "--ip", type=str, default=HOST, help="ip address of the can bus"
    # )
    # parser.add_argument("--port", type=int, default=PORT, help="port of the can bus")
    # parser.add_argument(
    # "--setangle", type=int, help="set the angle of the turntable", default=None
    # )

    # parser.add_argument(
    # "--calibrate", action="store_true", help="calibrate the turntable"
    # )
    # parser.add_argument(
    # "--test_sequence",
    # action="store_true",
    # help="test sequence [0-360] rotate in 45 degree increments",
    # )

    args = parser.parse_args()
    main()
