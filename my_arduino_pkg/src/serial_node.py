import rospy
from bbox_det.msg import order_to_pwm
import serial

rospy.init_node('serial_node')
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Set your Arduino port and baudrate

def order_to_pwm_callback(msg):
    # Callback function for order_to_pwm topic
    diff_cx = msg.diff_cx
    diff_cy = msg.diff_cy
    rospy.loginfo("Received order_to_pwm message. diff_cx: %.2f, diff_cy: %.2f", diff_cx, diff_cy)

    if diff_cx > 130:
        rospy.loginfo("Performing right steering control.")
        pwm_msg = 100  # Example PWM value for right steering
    elif diff_cx < -130:
        rospy.loginfo("Performing left steering control.")
        pwm_msg = 200  # Example PWM value for left steering
    else:
        rospy.loginfo("Performing straight steering control.")
        pwm_msg = 150  # Example PWM value for straight steering

    # Send calculated PWM value to Arduino through serial communication
    ser.write(str(pwm_msg).encode('utf-8'))

# Subscribe to order_to_pwm topic
rospy.Subscriber("order_to_pwm", order_to_pwm, order_to_pwm_callback)

rospy.spin()
