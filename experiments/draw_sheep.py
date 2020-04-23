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

def scale_sketch_to_workspace(trace, work_xmin, work_xmax, work_ymin, work_ymax, pen_down, pen_up):
    n = trace.shape[0]
    delta_xs = trace[:,0]
    delta_ys = trace[:,1]
    true_xs = []
    true_ys = []
    x = 0
    y = 0
    # find position at each point given delta assuming zero start
    pens = []
    for i in range(n):
        true_xs.append(x)
        true_ys.append(y)
        x+=delta_xs[i]
        y+=delta_ys[i]
        if trace[i,2]:
            pens.append(pen_down)
        else:
            pens.append(pen_up)
    true_xs = np.array(true_xs)
    true_ys = np.array(true_ys)
    xwork_avail = (work_xmax-work_xmin)
    ywork_avail = (work_ymax-work_ymin)
    xwork_sketch = max(true_xs)-min(true_xs)
    ywork_sketch = max(true_ys)-min(true_ys)
    # find scale for each dimension
    if xwork_sketch > xwork_avail:
        xscale = xwork_avail/float(xwork_sketch)
    else:
        xscale = xwork_sketch/float(xwork_avail)
    if ywork_sketch > ywork_avail:
        yscale = ywork_avail/float(ywork_sketch)
    else:
        yscale = ywork_sketch/float(ywork_avail)
    scaled_xs = true_xs*xscale
    scaled_ys = true_ys*yscale
    minx = scaled_xs.min()
    maxx = scaled_xs.max()
    miny = scaled_ys.min()
    maxy = scaled_ys.max()
    startx = work_xmin-minx
    starty = work_ymin-miny
    scaled_xs += startx
    scaled_ys += starty
    outtrace = np.array([scaled_xs, scaled_ys, pens]).T
    assert round(outtrace[:,0].min(), 3) >= work_xmin
    assert round(outtrace[:,0].max(), 3) <= work_xmax
    assert round(outtrace[:,1].min(), 3) >= work_ymin
    assert round(outtrace[:,1].max(), 3) <= work_ymax
    return outtrace

def plot_trace(trace, trace_plot_filepath='fig1.png'):
    plt.figure()
    xs = trace[:,0]
    ys = trace[:,1]
    lift_pen = trace[:,2]
    lifted = lift_pen[0]
    xi = xs[0]
    yi = ys[0]
    lifted = 0
    for stroke in range(1,len(trace)):
        xii = xs[stroke]
        yii = ys[stroke]
        #if not lifted:
        plt.plot([xi,xii], [yi,yii])
        #else:
        #    print('not plotting', xi, xii)
        lifted = lift_pen[stroke-1]
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

    def draw_trace(self, trace):
        jd.service_home()
        steps = [] 
        goals = []
        # orientation with hand pointed down like holding pen
        draw_orientation = [.84, .51, .129, .09]
        # go to initial point
        for stroke in range(len(trace)):
            x = trace[stroke,0]
            y = trace[stroke,1]
            z = trace[stroke,2]
            goal = [x, y, z]+draw_orientation
            goals.append(goal)
            steps.append(self.service_step('POSE', False, 'mq', goal))
        return goals, steps

 
if __name__ == '__main__':
    # data is train/test/valid of shape deltax, deltay, pen state (up/down)
    gitpath = 'https://github.com/hardmaru/sketch-rnn-datasets'
    datadir = 'sketch-rnn-datasets/aaron_sheep/aaron_sheep.npz'
    if not os.path.exists(datadir):
        print('cloning sketchrnn dataset')
        os.system('git clone {}'.format(gitpath))
    data = np.load(datadir)
    train_data = data['train']
    random_state = np.random.RandomState(22)
    axes = np.array(['x','y','z'])
    axes_extents = {
               'x':(fence.minx, fence.maxx),
               'y':(fence.miny, fence.maxy), 
               'z':(fence.minz, fence.maxz)
               }

    datadir = 'drawings'
    shuffle_inds = True
    if not os.path.exists(datadir):
        os.makedirs(datadir)
    jd = JacoDraw()
    trial_num = 1
    for ii in range(len(train_data)):
        random_state.shuffle(axes)
        trace = train_data[ii]
        print("starting new trace {} - sending home".format(ii))
        print(axes)
        axes = list(axes)
        tx = axes[0]
        ty = axes[1]
        tz = axes[2]
        pen_down = ((axes_extents[tz][1]-axes_extents[tz][0])/2.0)+axes_extents[tz][0]
        pen_up = ((axes_extents[tz][1]-axes_extents[tz][0])/2.0)+axes_extents[tz][0]
        # order in x,y,z
        trace = scale_sketch_to_workspace(trace, 
                                          axes_extents[tx][0]+.01, axes_extents[tx][1]-.01, 
                                          axes_extents[ty][0]+.01, axes_extents[ty][1]-.01, 
                                          pen_down, pen_up)
        inds = np.arange(len(trace))
        if shuffle_inds:
            random_state.shuffle(inds)

        bpath = os.path.join(datadir, 'T%02d_%04d'%(trial_num, ii))
        plot_trace(trace, trace_plot_filepath=bpath+'_pts.png')
        plot_trace(trace[inds], trace_plot_filepath=bpath+'_shuffled_pts.png')
        # send to intended axis
        arm_ind = [axes.index('x'), axes.index('y'), axes.index('z')]
        arm_trace = trace[inds][:,arm_ind] 
        goals, steps = jd.draw_trace(arm_trace)
        np.savez(bpath, axes=axes, inds=inds, trace=trace, steps=steps)
        if ii > 2:
            sys.exit()

        
        



