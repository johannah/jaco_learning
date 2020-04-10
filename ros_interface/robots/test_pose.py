import fence
import jaco
import rospy
from ros_interface.srv import reset, step, home, get_state
import time

class TestPose():
    def __init__(self):
        self.setup_ros()

    def setup_ros(self):
        print('setting up ros')
        rospy.wait_for_service('/reset')
        self.service_reset = rospy.ServiceProxy('/reset', reset)
        print('setup service: reset')
        rospy.wait_for_service('/home')
        self.service_home = rospy.ServiceProxy('/home', home)
        print('setup service: home')
        rospy.wait_for_service('/get_state')
        self.service_get_state = rospy.ServiceProxy('/get_state', get_state)
        print('setup service: get_state')
        rospy.wait_for_service('/step')
        self.service_step = rospy.ServiceProxy('/step', step)
        print('setup service: step')
        print('finished setting up ros')

    def test_go(self):
        pos = [0.11797773838,
               -0.620377063751,
               0.548168230057,
               0.704044687489,
               0.152672781875,
               0.281919003591,
               0.633666927579]
        relative = False
        self.service_step('POSE', relative, 'mq', pos)
        time.sleep(.1)


    def test_twist(self):
        # TODO go to position
        #joint1: 283.299957275
        #joint2: 162.709121704
        #joint3: 0.0044891834259
        #joint4: 45.9832077026
        #joint5: 265.231140137
        #joint6: 257.519989014
        #joint7: 294.616455078
        print("test_up")
        vel = [0, 0, 0, 640, 0, 0, 0]
        for i in range(100):
            self.service_step('VEL', False, 'degs', vel)
            time.sleep(.01)

    def test_rel_up(self):
        print("test_left")
        pos = [0.0, 0.0, 0.1, 0, 0, 0]
        relative = True
        for i in range(1):
            self.service_step('POSE', relative, 'mdeg', pos)
            time.sleep(.1)

if __name__ == '__main__':
    testp = TestPose()
    testp.test_rel_up()
    #testp.test_go()


