import rospy
import pyautogui
from std_msgs.msg import UInt16

a=4
def test_callback(msg):
    pwm_value = msg.data
    global a
    if pwm_value == 3:
        if a != 3:
            pyautogui.press('a')
            a = 3
    elif pwm_value == 1:
        if a != 1:
            pyautogui.press('d')
            a = 1
    elif pwm_value == 2:
        if a != 2:
            pyautogui.press('w')
            a = 2
    else:
        print("Unknown PWM value.")

def test_node():
    rospy.init_node('pwm_listener', anonymous=True)
    rospy.Subscriber('pwm_to_arduino', UInt16, test_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        test_node()
    except rospy.ROSInterruptException:
        pass
