
from bitstring import BitArray
import struct



# # Open the COM port
# ser = serial.Serial('COM6', 115200, timeout=1)


# data =  0x011F6B
# # Write data to the COM port
# ser.write(data.to_bytes(3, 'big'))
          

# # Read data from the COM port
# data = ser.read(10) 
# print("response:", data)

# # Close the COM port
# ser.close()



def float_to_uint(x, x_min, x_max, numBits):
    span = x_max - x_min
    offset = x_min
    # Attempt to speedup by using pre-computation. Not used currently.
    if numBits == 16:
        bitRange = maxRawPosition
    elif numBits == 12:
        bitRange = maxRawVelocity
    else:
        bitRange = 2**numBits - 1
    return int(((x - offset) * (bitRange)) / span)


def uint_to_float(x_int, x_min, x_max, numBits):
    span = x_max - x_min
    offset = x_min
    if numBits == 16:
        bitRange = maxRawPosition
    elif numBits == 12:
        bitRange = maxRawVelocity
    else:
        bitRange = 2**numBits - 1
    return ((x_int * span) / (bitRange)) + offset


def waitOhneSleep(dt):
    startTime = time.time()
    while time.time() - startTime < dt:
        pass




class EMMV5MotorController:
    com_port = 'COM6'
    baud_rate = 115200




    

    def __init__(self, com_port="COM6", motor_id=0x01, timeout=0.05):
        self.com_port = com_port
        self.motor_id = motor_id
        self.timeout = timeout
        self.ser = serial.Serial(self.com_port, self.baud_rate, timeout=self.timeout)


         # Initialize the command BitArrays for performance optimization
        self._p_des_BitArray = BitArray(
            uint=float_to_uint(0, self.motorParams["P_MIN"], self.motorParams["P_MAX"], 16), length=16
        )
        self._v_des_BitArray = BitArray(
            uint=float_to_uint(0, self.motorParams["V_MIN"], self.motorParams["V_MAX"], 12), length=12
        )
        self._kp_BitArray = BitArray(uint=0, length=12)
        self._kd_BitArray = BitArray(uint=0, length=12)
        self._tau_BitArray = BitArray(uint=0, length=12)
        self._cmd_bytes = BitArray(uint=0, length=64)
        self._recv_bytes = BitArray(uint=0, length=48)

    def _send_can_frame(self, data):
        """
        Send raw CAN data frame (in bytes) to the motor.
        """
        can_dlc = len(data)
        can_msg = struct.pack(can_frame_fmt_send, self.motor_id, can_dlc, data)
        try:
            CanMotorController.motor_socket.send(can_msg)
        except Exception as e:
            print("Unable to Send CAN Frame.")
            print("Error: ", e)

    def _recv_can_frame(self):
        """
        Receive a CAN frame and unpack it. Returns can_id, can_dlc (data length), data (in bytes)
        """
        try:
            # The motor sends back only 6 bytes.
            frame, addr = CanMotorController.motor_socket.recvfrom(recvBytes)
            can_id, can_dlc, data = struct.unpack(can_frame_fmt_recv, frame)
            return can_id, can_dlc, data[:can_dlc]
        except Exception as e:
            print("Unable to Receive CAN Frame.")
            print("Error: ", e)


    def decode_motor_status(self, data_frame):
        """
        Function to decode the motor status reply message into its constituent raw values.

        /// CAN Reply Packet Structure ///
        /// 16 bit position, between -4*pi and 4*pi
        /// 12 bit velocity, between -30 and + 30 rad/s
        /// 12 bit current, between -40 and 40;
        /// CAN Packet is 5 8-bit words
        /// Formatted as follows.  For each quantity, bit 0 is LSB
        /// 0: [position[15-8]]
        /// 1: [position[7-0]]
        /// 2: [velocity[11-4]]
        /// 3: [velocity[3-0], current[11-8]]
        /// 4: [current[7-0]]

        returns: the following raw values as (u)int: position, velocity, current
        """

        # Convert the message from motor to a bit string as this is easier to deal with than hex
        # while seperating individual values.
        self._recv_bytes.bytes = data_frame
        dataBitArray = self._recv_bytes.bin

        # Separate motor status values from the bit string.
        # Motor ID not considered necessary at the moment.
        # motor_id = dataBitArray[:8]
        positionBitArray = dataBitArray[8:24]
        velocityBitArray = dataBitArray[24:36]
        currentBitArray = dataBitArray[36:48]

        # motor_id = int(motor_id, 2)
        positionRawValue = int(positionBitArray, 2)
        velocityRawValue = int(velocityBitArray, 2)
        currentRawValue = int(currentBitArray, 2)

        # TODO: Is it necessary/better to return motor_id?
        # return motor_id, positionRawValue, velocityRawValue, currentRawValue
        return positionRawValue, velocityRawValue, currentRawValue
    
