import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Int32MultiArray

global serialFlag
serialFlag = 1;

import serial
roboteq_obj = serial.Serial(
port='/dev/ttyACM0',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1)

#serialFlag = 1
#except:
#    print("Serial doesn't exist.")

class motor_driver(Node):

    def __init__(self):
        super().__init__('cmd_roboteq2')
        self.inCmd = 0.0
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/p2/ch_vals',
            self.cmd_callback,
            5)
        self.subscription  # prevent unused variable warning

    def move_motor_ch1(self, val):
        global serialFlag
        payload1 = "!G 1 " + str(-val) + "_"
        #if(serialFlag):
        roboteq_obj.write(str.encode(payload1))
    
    def move_motor_ch2(self, val):
        global serialFlag
        payload2 = "!G 2 " + str(val) + "_"
        #if(serialFlag):
        roboteq_obj.write(str.encode(payload2))
            
    def cmd_callback(self, msg):
        inCmd1 = msg.data[0]
        inCmd2 = msg.data[1]
        self.move_motor_ch1(inCmd1)
        self.move_motor_ch2(inCmd2)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = motor_driver()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
