import math
import numpy as np

def convert_to_degree(angle):
    """
    Converts the input angle to degree.
    :param angle: input angle in radian
    :type angle: np.array or float
    :return: converted angle to degrees
    :rtype: np.array or float
    """
    return 180. * angle / np.pi

def wrap_to_pi(angles):
    """
    Wraps a list of angles to [-pi, pi].
    :param angles: list of angles to be wrapped
    :type angles: list or np.array
    :return: list of wrapped angles
    :rtype: np.array
    """
    # convert angles to numpy array
    angles = np.array(angles)

    # wrap the angles to [-pi, pi] phases
    phases = (angles + np.pi) % (2 * np.pi) - np.pi

    return phases


def compute_idyn_torque(n_joints, q, qdot, qddot):
    """
    Computes the inverse dynamics torques using pybullet library. This function is very fast and can be used in an
    online scheme. Note that this method only computes a single entry.
    :param q: joint positions
    :type q: np.array
    :param qdot: joint velocities
    :type qdot: np.array
    :param qddot: joint accelerations
    :type qddot: np.array
    :return: required torques to do the motion
    :rtype: np.array
    """
    # concatenate finger data because the model in PyBullet requires data of fingers as the robot states
    finger_data = np.zeros(3)
    q = np.concatenate((q, finger_data))
    qdot = np.concatenate((qdot, finger_data))
    qddot = np.concatenate((qddot, finger_data))

    # convert states to list
    q = list(q)
    qdot = list(qdot)
    qddot = list(qddot)

    # compute id torques (returns a [n_joints + 3] list)
    torque = pybullet.calculateInverseDynamics(self.pybullet_robot, q, qdot, qddot)

    # convert the torque to np.array and remove finger data
    torque = np.array(torque)[0:n_joints]

    return torque

def compute_ik(pybullet_robot, n_joints, position, orientation, euler_flag=False):
    """
    Quick IK solver implemented in PyBullet. This implementation is not working very well. Use MoveIt! instead!
    :param position: target position of the end-effector in world frame
    :type position: list or np.array
    :param orientation: target orientation of the end-effector in world frame
    :type orientation: list or np.array
    :param euler_flag: a flag to indicate whether the orientation is in euler or quaternions
    :type euler_flag: bool
    :return: joint positions
    :rtype: np.array
    """
    # convert orientation euler angles to quaternions if the optional flag is set to True
    if euler_flag:
        orientation = tf.transformations.quaternion_from_euler(*orientation)

    position = list(position)
    orientation = list(orientation)

    if n_joints == 6:
        end_effector_link_id = 8
    else:
        end_effector_link_id = 9
    q = pybullet.calculateInverseKinematics(pybullet_robot, end_effector_link_id, position, targetOrientation=orientation)

    q = np.array(q)

    return pybullet_robot, q[:n_joints]

def compute_jacobian(pybullet_robot, n_joints, q):
    """
    Computes the geometric Jacobian of the arm with respect to the end effector. Note that this method only computes
    a single entry.
    :param q: joint positions
    :type q: np.array
    :return: Jacobian [linear jacobian; angular jacobian] shape: [6, n_joints + 3]
    :rtype: np.array
    """
    # concatenate finger data because the model in PyBullet requires data of fingers as the robot states
    finger_data = np.zeros(3)
    q = np.concatenate((q, finger_data))
    qdot = np.zeros(q.shape)
    qddot = np.zeros(q.shape)

    # convert states to list
    q = list(q)
    qdot = list(qdot)
    qddot = list(qddot)

    # compute Jacobian
    if n_joints == 6:
        end_effector_link_id = 8
    else:
        end_effector_link_id = 9
    local_position = [0., 0., 0.]

    linear_jacobian, angular_jacobian = pybullet.calculateJacobian(pybullet_robot, end_effector_link_id,
                                                                   local_position, q, qdot, qddot)

    jacobian = np.concatenate((np.array(linear_jacobian), np.array(angular_jacobian)), axis=0)

    # remove fingers from the jacobian
    jacobian = jacobian[:, 0:self.n_joints]

    return pybullet_robot, jacobian

# from kinova demo
def QuaternionNorm(Q_raw):
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    if np.abs(qnorm) > 0:
        qx_ = qx_temp/qnorm
        qy_ = qy_temp/qnorm
        qz_ = qz_temp/qnorm
        qw_ = qw_temp/qnorm
        Q_normed_ = [qx_, qy_, qz_, qw_]
    else:
        print("unable to use 0 qnorm")
        Q_normed_ = Q_raw[0:4] 
    return Q_normed_


# from kinova demo
def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_


# from kinova demo
def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_


