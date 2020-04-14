import matplotlib
matplotlib.use("Agg")
import numpy as np
import os
import sys
import fence
import matplotlib.pyplot as plt
from IPython import embed
import rospy
from ros_interface.srv import reset, step, home, get_state

def scale_sketch_to_workspace(trace, wxmin, wxmax, wymin, wymax):
    xworkspace = wxmax-wxmin
    yworkspace = wymax-wymin
    trace = trace.astype(np.float64)
    maxx=0; minx=0; maxy=0; miny=0
    absx=0; absy=0
    for i in range(len(trace)):
        absx+=trace[i,0]
        absy+=trace[i,1]
        minx = min(minx, absx)
        miny = min(miny, absy)
        maxx = max(maxx, absx)
        maxy = max(maxy, absy)
    xdiff = maxx-minx
    ydiff = maxy-miny
    # scale xdiff/ydiff to be in range of workspace
    print('diffs', xdiff, ydiff)
    if xdiff > xworkspace:
        xscale = xworkspace/float(xdiff)
    else:
        xscale = xdiff/float(xworkspace)
    if ydiff > yworkspace:
        yscale = yworkspace/float(ydiff)
    else:
        yscale = ydiff/float(yworkspace)
    # scale so all strokes are within range
    #scale = min([xscale, yscale])
    scale = yscale
    print('scale', scale)
    trace[:,0]*=scale
    trace[:,1]*=scale
    minx*=scale
    maxx*=scale
    miny*=scale
    maxy*=scale
    print('ydiff', maxy-miny)
    print('yworkspace', yworkspace)
    #trace[:,0]+=wxmin
    #trace[:,1]+=wymin
    startx = wxmin+(trace[0,0]-minx)
    starty = wymin+(trace[0,1]-miny)
    return trace, startx, starty

def plot_trace(trace, xstart, ystart, trace_plot_filepath='fig1.png'):
    plt.figure()
    xs = trace[:,0]
    ys = trace[:,1]
    lift_pen = trace[:,2]
    xi = xstart; yi = ystart
    lifted = lift_pen[0]
    for stroke in range(len(trace)-1):
        xii = xi + xs[stroke]
        yii = yi + ys[stroke]
        if not lifted:
            plt.plot([xi,xii], [yi,yii])
        else:
            print('not plotting', xi, xii)
        lifted = lift_pen[stroke]
        xi = xii
        yi = yii
    plt.savefig(trace_plot_filepath)
    plt.close()

class JacoDraw():
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

    def draw_trace(self, trace, xstart, ystart, draw_plane=.35):
        # orientation with hand pointed down like holding pen
        draw_orientation = [.84, .51, .129, .09]
        xs = trace[:,0]
        ys = trace[:,1]
        lift_pen = trace[:,2]
        lifted = lift_pen[0] 
        # go to initial point
        self.service_step('POSE', False, 'mq', [xstart, ystart, draw_plane]+draw_orientation)
        for stroke in range(1, len(trace)):
            # sent relative step
            self.service_step('POSE', True, 'mdeg', [xs[stroke], ys[stroke], 0]+[0,0,0])
            # raise pen
            if lift_pen[stroke]:
                print("lifting pen")
                self.service_step('POSE', True, 'mdeg', [xs[stroke], ys[stroke], .1]+[0,0,0])
                if stroke+1 < len(trace):
                    self.service_step('POSE', True, 'mdeg', [xs[stroke+1], ys[stroke+1], -.1]+[0,0,0])
                print("putting down pen")

 
if __name__ == '__main__':
    # data is train/test/valid of shape deltax, deltay, pen state (up/down)
    gitpath = 'https://github.com/hardmaru/sketch-rnn-datasets'
    datadir = 'sketch-rnn-datasets/aaron_sheep/aaron_sheep.npz'
    if not os.path.exists(datadir):
        print('cloning sketchrnn dataset')
        os.system('git clone {}'.format(gitpath))
    jd = JacoDraw()
    data = np.load(datadir)
    train_data = data['train']
    for ii in range(len(train_data)):
        trace = train_data[ii]
        print(fence.minx, fence.miny)
        print(fence.maxx, fence.maxy)

        print("starting new trace {} - sending home".format(ii))
        jd.service_home()
        trace,startx,starty = scale_sketch_to_workspace(trace, fence.minx*.8, fence.maxx*.8, fence.miny*.8, fence.maxy*.8)
        jd.draw_trace(trace, startx, starty)

        
        



