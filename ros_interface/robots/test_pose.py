import fence
import jaco
import rospy
from ros_interface.srv import reset, step, home, get_state
import time

class TestPose():
    def __init__(self):
        self.defx = fence.minx+((fence.maxx-fence.minx)/2.0)
        self.defy = fence.miny+((fence.maxy-fence.miny)/2.0)
        self.defz = fence.minz+((fence.maxz-fence.minz)/2.0)
        print('using default x,y,z of {},{},{}'.format(self.defx,self.defy,self.defz))
        # test to ensure fence works on points 
        #  leftmost is left when facing the kinova label
        self.leftmost = [fence.minx-1, self.defy,  self.defz]
        self.rightmost = [fence.maxx+1, self.defy, self.defz]
        ## y is forward when human is facing kinova label (opp cables)
        ## y is most backward when y is most positive
        self.forwardmost =  [self.defx, fence.miny+1, self.defz]
        self.backwardmost = [self.defx, fence.maxy-1, self.defz]
        self.lowermost = [self.defx, self.defy, fence.minz-1]
        self.uppermost = [self.defx, self.defy, fence.maxz+1]

        self.leftforwardupper =   [fence.minx, fence.miny, fence.maxz]
        self.leftforwardlower =   [fence.minx, fence.miny, fence.minz]
        self.leftbackwardupper =  [fence.minx, fence.maxy, fence.maxz]
        self.leftbackwardlower =  [fence.minx, fence.maxy, fence.minz]
        self.rightbackwardupper = [fence.maxx, fence.maxy, fence.maxz]
        self.rightbackwardlower = [fence.maxx, fence.maxy, fence.minz]
        self.rightforwardupper =  [fence.maxx, fence.miny, fence.maxz]
        self.rightforwardlower =  [fence.maxx, fence.miny, fence.minz]

        self.extremes = [('left', self.leftmost), 
                         ('forward', self.forwardmost), 
                         ('lower', self.lowermost), 
                         ('right', self.rightmost), 
                         ('upper', self.uppermost), 
                        ]
        self.corner_extremes = [
                               ('leftforwardupper', self.leftforwardupper), 
                               ('leftforwardlower', self.leftforwardlower), 
                               ('leftbackwardlower', self.leftbackwardlower), 
                               ('leftbackwardupper', self.leftbackwardupper), 
                               ('rightbackwardupper', self.rightbackwardupper), 
                               ('rightbackwardlower', self.rightbackwardlower), 
                               ('rightforwardlower', self.rightforwardlower), 
                               ('rightforwardupper', self.rightforwardupper), ]
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

    def send_position(self, position, orientation, relative):
        self.service_step('POSE', relative, 'mdeg', position+orientation)

    def check_fence_extremes(self):
        for (label, pos) in self.extremes:
            print('moving to extreme {}:, {}'.format(label, pos))
            self.send_position(position=pos, orientation=[0,0,0], relative=False)

    def check_fence_corners(self):
        for (label, pos) in self.corner_extremes:
            print('moving to corner {}:, {}'.format(label, pos))
            self.send_position(position=pos, orientation=[0,0,0], relative=False)
            time.sleep(3)


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
        self.service_step('POSE', relative, 'mdeg', pos)
        time.sleep(.1)







if __name__ == '__main__':
    testp = TestPose()
    testp.check_fence_extremes()
    testp.check_fence_corners()
