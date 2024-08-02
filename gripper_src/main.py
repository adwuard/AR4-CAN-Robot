from zdt_emmv5 import *
from ar4_configs import *
from joint import *
# import PySimpleGUI as psg
import can


HOST = "192.168.51.210"
PORT = 8886 # Port to listen on (non-privileged ports are > 1023)


class Gripper_Controller:
    def __init__(self):
        # self.interface = can.interface.Bus(bustype='slcan', channel='COM27', bitrate=500000)
        
        
        self.hw_interface = SocketCAN(HOST, PORT)
        # self.hw_interface = SLCANPort(self.interface)
        
        self.gripper_motor = ZDT_EMMV5_MOTOR(self.hw_interface, 0x01)
        
        self.gripper_motor.clear_stall_error()
        # self.gripper_motor.trigger_sensor_zeroing()
    
gc = Gripper_Controller()
# gc.open_gripper()
# time.sleep(5)
# gc.gripper_motor.set_position_control(1, 100, 240, 2000, absolute_mode=True, wait_broadcast_signal=False
gc.gripper_motor.set_speed_control(0, 2000, 245, wait_broadcast_signal=False)

 
import matplotlib.pyplot as plt

# Create empty lists to store data
time_data = []
current_data = []
data2 = []

# Set up the plot
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()
line, = ax.plot(time_data, current_data)
ax.set_xlabel('Time')
ax.set_ylabel('Speed')

# Start the real-time plotting loop
while True:
    # n = gc.gripper_motor.read_phase_current_mA()
    n = gc.gripper_motor.read_rpm()
    
    time_data.append(len(time_data) + 1)  # Increment time
    current_data.append(n)
    # data2.append(x)
    
    # Update t
    # he plot
    line.set_data(time_data, current_data)
    # line.set_data(time_data, data2)
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()
    fig.canvas.flush_events()
    
    # Pause for a short time
    # plt.pause(0.1)
# time.sleep(.2)