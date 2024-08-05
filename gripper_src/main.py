from zdt_emmv5 import *
import socket
import logging
import threading


class Vibe_XGripper_V1:
    class OPEN_DIR():
        OPEN = 0
        CLOSE = 1
        
    def __init__(self, ip_host, ip_port, motor_id=0x01, zeroing_on_boot=False) -> None:        
        super().__init__()
        
        
        self.HOST = ip_host
        self.PORT = ip_port
        self.socket = socket
        
        if not self._connection_check():
            raise ConnectionError("Timeout: Fail to connect to Gripper IP:", self.HOST, ":", self.PORT)
        else:
            logger.info("Connected to Gripper IP: "+ str(self.HOST)+ ":"+ str(self.PORT))

        # setting up CANBUS over ethernet and protocol driver
        self.hw_interface = SocketCAN(self.HOST, self.PORT)        
        self.motor_ctx = ZDT_EMMV5_MOTOR(self.hw_interface, motor_id)
        
        # Callbacks registration
        self.on_arrival_cb = None    
        self.on_stall_cb = None
        
        K_STEP_RES = 1.8
        K_STEPS_PRE_REV = 360 / K_STEP_RES
        K_MICRO_STEPS = 1
        K_GEAR_RATIO = 3.7 # stepper motor gearbox gear ratio 1:3.7
        K_BELT_RATIO = 1 # 1:1 motor output shaft to leadscrew
        K_LEAD_SCREW_PITCH = 2 # 1:1 motor shaft to leadscrew
        
        
        self.acceleration = 250
        self.rpm = 1400
        
        self.steps_per_mm = K_STEPS_PRE_REV * K_MICRO_STEPS * K_GEAR_RATIO * K_BELT_RATIO / K_LEAD_SCREW_PITCH / 2

        """ Motor limits from limit the switch"""
        # Open distance beyond the limit switch, multiplying 2 for bi-directional lead screw 
        self.K_OPEN_POS_LIMIT_MM = 5 * 2
        # Closing distance within the limit switch, multiplying 2 for bi-directional lead screw 
        self.K_CLOSE_POS_LIMIT_MM = 36 * 2
        

        self.print_motor_info()
        
        if zeroing_on_boot:
            self.zero_gripper()
        
        
        self.busy = False
        self.check_thread = None
        self.check_thread_exit_flag = False
        
    
    def print_motor_info(self):
        print("==============VIBE XGripper=====================")
        print("Gripper Distance: ", self.K_OPEN_POS_LIMIT_MM + self.K_CLOSE_POS_LIMIT_MM)
        print("Acceleration (0-255): ", self.acceleration) 
        print("steps Per mm: ", self.steps_per_mm)  
        print("==============MOTOR_STATUS=====================")
        print("Stepper motor driver version:", str(self.motor_ctx.read_hardware_version()))
        print("Motor Status: ", str(self.motor_ctx.read_motor_status_flags()))
        print("Zeroing Status: ", str(self.motor_ctx.read_zeroing_status()))
        print("Voltage: ", self.motor_ctx.read_vbus_voltage_mV()*0.001, "VDC")
        print("Current: {:.2f}A".format(self.motor_ctx.read_phase_current_mA()*0.001))
        print("Pulse Cnt: ", self.motor_ctx.read_pulse_count(), "Steps")
        print("Position Error: {:.2f}%".format(self.motor_ctx.read_pos_error()))
        print("==============================================")
    
    # def set_speed_mm_per_sec(self, speed_mm_per_sec):
    #     self.rpm_speed_mm_per_min = speed_mm_per_sec * 60
    #     self.rpm_speed_mm_per_min = int(self.rpm_speed_mm_per_min)    
    #     print("rpm", (self.rpm_speed_mm_per_min / self.steps_per_mm)*3.7*2)    
    #     return
    
    # def register_cb(self, on_arrival_cb = None, on_stall_cb = None):
    #     if on_arrival_cb:
    #         self.on_arrival_cb = on_arrival_cb
    #     if on_stall_cb:
    #         self.on_stall_cb = on_stall_cb
    
    
    # def _on_arrival(self):
    #     logger.info("EVENT: Arrived at target")
    #     self.busy = False
    #     if self.on_arrival_cb:
    #         self.on_arrival_cb()
    #     return

    # def _on_stall(self):
    #     logger.info("EVENT: Motor Stalled!!")
    #     self.busy = False
    #     if self.on_stall_cb:
    #         self.on_stall_cb()
    #     return
    
    def zero_gripper(self):
        def on_zeroed_cb():
            logger.info("Zeroing completed")
            return True
        
        logger.info("***** Zeroing gripper *****")    
        self.motor_ctx.clear_stall_error()
        
        self.move_distance_mm(70)
        self.arrival_loop_check() # blocking check
        
        self.motor_ctx.trigger_sensor_zeroing()
        self.arrival_loop_check(arrival_cb=on_zeroed_cb) 
        return
    
    def home(self):
        self.move_distance_mm(0)
    
    def set_speed(self, speed):
        if speed > 255 or speed <0:
            raise ValueError("Speed should be between 0-255")
        # prevent sudden start stop acceleration. "0" settings is not allowed 
        # this is from motor driver defination 
        if speed == 0:
            speed = 255
        self.acceleration = speed
        return        
        
    def syncronous_set():
        pass
        
    def asyncronous_set():
        pass

    def _connection_check(self):
        s = self.socket.socket(self.socket.AF_INET, self.socket.SOCK_STREAM)
        s.settimeout(0.1) #Timeout in case of port not open
        try:
            s.connect((self.HOST, self.PORT))
            return True
        except:
            return False
        
    def arrival_loop_check(self, arrival_cb = None, stall_cb = None):
        self.busy = True
        while not self.check_thread_exit_flag:
            time.sleep(0.8)
            try:
                mstatus = self.motor_ctx.read_motor_status_flags()
                if mstatus is not None:
                    if mstatus['MOTOR_STALL']:
                        if stall_cb:
                            logger.info("EVENT: Motor Stalled")
                            stall_cb()
                
                time.sleep(.2)

                pos_err = self.motor_ctx.read_pos_error()
                if abs(pos_err) < 5:
                    if arrival_cb:
                        logger.info("EVENT: Motor Arrived at Target")
                        arrival_cb()
                    return
            except:
                logger.error("Timeout. re-checking..")
        return

    def _start_arrival_loop_check_thread(self):
        self.check_thread = threading.Thread(target=self.arrival_loop_check, args=(self.on_arrival_cb, self.on_stall_cb))
        self.check_thread.start()
        

    def _stop_arrival_loop_check_thread(self):
        if self.check_thread and self.check_thread.is_alive():
            self.check_thread_exit_flag = True
            self.check_thread.join()
        self.check_thread = None
        self.check_thread_exit_flag = False
        return

        
    """ Move fripper in percentage
    - 100% = all the way closed
    - 0% = all the way open
    """
    def move_percentage(self, percentage, arrival_cb = None, stall_cb = None, syncronous=False):
        if percentage < 0:
            raise ValueError("Percentage should be greater than 0")
        elif percentage > 100:
            raise ValueError("Percentage should be less than 100")

        full_distance = self.K_OPEN_POS_LIMIT_MM + self.K_CLOSE_POS_LIMIT_MM
        distance_mm = full_distance * (percentage / 100)
        
        if arrival_cb:
            self.on_arrival_cb = arrival_cb
        if stall_cb:
            self.on_stall_cb = stall_cb
            
        logger.info("Moving gripper to: "+ str(percentage) + "% , Distance: "+ str(distance_mm) + "mm")

        steps = int(distance_mm * self.steps_per_mm)
        

        if self.check_thread and not syncronous:
            self._stop_arrival_loop_check_thread()
    
        try:
            self.motor_ctx.clear_stall_error()

            if steps <= self.K_OPEN_POS_LIMIT_MM * self.steps_per_mm:
                # open gripper beyond the limit switch
                steps = abs(steps-(self.K_OPEN_POS_LIMIT_MM * self.steps_per_mm))
                self.motor_ctx.set_position_control(self.OPEN_DIR.OPEN, self.rpm, self.acceleration, steps , absolute_mode=True)
            else:
                steps = steps - (self.K_OPEN_POS_LIMIT_MM * self.steps_per_mm)
                self.motor_ctx.set_position_control(self.OPEN_DIR.CLOSE, self.rpm, self.acceleration, steps, absolute_mode=True)
        except Exception as e:
            print(e)
            
        if not syncronous:
            self._start_arrival_loop_check_thread()
        else:
            self.arrival_loop_check(arrival_cb=self.on_arrival_cb, stall_cb=self.on_stall_cb)




    """ Move gripper finger distance distance in mm
    - 0 = all the way closed
    - max_travel = all the way open
    """
    def move_distance_mm(self, distance_mm, arrival_cb = None, stall_cb = None, syncronous=False):
        full_distance = self.K_OPEN_POS_LIMIT_MM + self.K_CLOSE_POS_LIMIT_MM
        
        if distance_mm < 0:
            raise ValueError("distance should be greater than 0")
        elif distance_mm > full_distance:
            raise ValueError("Percentage should be less than", full_distance)


        if arrival_cb:
            self.on_arrival_cb = arrival_cb
        if stall_cb:
            self.on_stall_cb = stall_cb
        
        from_open_mm = (full_distance - distance_mm)
        
        
        logger.info("Moving gripper to Distance: "+ str(distance_mm))
        
        if self.check_thread and not syncronous:
            self._stop_arrival_loop_check_thread()
        
        try:
            self.motor_ctx.clear_stall_error()
            if  from_open_mm <= self.K_OPEN_POS_LIMIT_MM:
                # open gripper beyond the limit switch
                
                steps = abs(int(from_open_mm * self.steps_per_mm))
                steps = abs(steps-(self.K_OPEN_POS_LIMIT_MM * self.steps_per_mm))
                self.motor_ctx.set_position_control(self.OPEN_DIR.OPEN, self.rpm, self.acceleration, steps , absolute_mode=True)
            else:
                steps = int(from_open_mm * self.steps_per_mm)
                steps = steps - (self.K_OPEN_POS_LIMIT_MM * self.steps_per_mm)
                self.motor_ctx.set_position_control(self.OPEN_DIR.CLOSE, self.rpm, self.acceleration, steps, absolute_mode=True)
        except Exception as e:
            print(e)
        
        if not syncronous:
            self._start_arrival_loop_check_thread()
        else:
            self.arrival_loop_check(arrival_cb=self.on_arrival_cb, stall_cb=self.on_stall_cb)



logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

HOST = "192.168.50.253"
PORT = 8886 # Port to listen on (non-privileged ports are > 1023)
gripper = Vibe_XGripper_V1(HOST, PORT, zeroing_on_boot=False)


def zeroing_demo():    
    if gripper.zero_gripper():
        print("Zeroing successful")

                            
def syncronous_demo():
    # Syncronous move, blocking call
    print("Syncronous Move: to distance 70%")
    gripper.move_distance_mm(70, syncronous=True)
    
    print("Syncronous Move: to 10%")
    gripper.move_distance_mm(10, syncronous=True)    



is_busy = False
def async_demo():
    global is_busy
    
    def on_arrival_cb():
        global is_busy
        is_busy = False
        print("Async Call: Callback, Arrived at target")
        return

    def on_stall_cb():
        global is_busy
        is_busy = False
        print("Async Call: Callback, Motor Stalled!")
        return
    
    while 1:
        for i in [0, 20, 40, 50, 70, 82]:
            # nonblocking call
            gripper.move_distance_mm(i, arrival_cb=on_arrival_cb, stall_cb=on_stall_cb)
            
            is_busy = True
            while is_busy:
                print('tick')
                time.sleep(.2)

    
    
if __name__ == "__main__":
    print("========= Zeroing DEMO ==============")
    zeroing_demo()
    print("")
    print("")
    time.sleep(1)
    
    print("========= Sync DEMO ==============")
    syncronous_demo()    
    print("")
    print("")
    time.sleep(1)
    
    print("========= Async DEMO ==============")
    async_demo()
    
    
    exit(0)
    
    
    
                
    
        
        