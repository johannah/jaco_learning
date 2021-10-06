import math
import numpy as np


def convert_to_degrees(unit, angle):
    """
    Converts the input angle to degree if unit is radians
    :param unit 'deg' or 'rad' indicating the type of input angle
    :param angle: array of input angles in radian or degrees
    :return: converted angle to degrees if unit==rad otherise return unchanged
    :rtype: np.array or float
    """
    if unit == 'rad':
        return np.rad2deg(angle)
    else:
        return angle


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


# from kinova demo
def QuaternionNorm(Q_raw):
    qx_temp, qy_temp, qz_temp, qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp * qx_temp + qy_temp * qy_temp +
                      qz_temp * qz_temp + qw_temp * qw_temp)
    if np.abs(qnorm) > 0:
        qx_ = qx_temp / qnorm
        qy_ = qy_temp / qnorm
        qz_ = qz_temp / qnorm
        qw_ = qw_temp / qnorm
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

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_),
                     (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_),
                     (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_, ty_, tz_]
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


def convert_tool_pose(current_tool_pose, unit, relative, position,
                      orientation):
    """
    unit: describes the unit of the command - mq:quaternian, mrad:radians, or mdeg:degrees. If mq, 3 position+4 quaternians data are required, otherwise, 3 position + 3 orientation data are required
    is_relative: bool indicative whether or not the pose command is relative to the current position or absolute
    position: relative or absolute position and orientation values for XYZ 
    """
    assert unit in ['mq', 'mrad', 'mdeg']
    if unit == 'mq':
        assert (len(orientation) == 4)
        "end effector pose in quaternions requires 3 position & 4 orientation values - received {}".format(
            len(orientation))
    else:
        assert (len(orientation) == 3)
        "end effector pose in rad/deg requires 3 position & 3 orientation values - received {}".format(
            len(orientation))

    last_position = [
        current_tool_pose.pose.position.x, current_tool_pose.pose.position.y,
        current_tool_pose.pose.position.z
    ]
    last_orientation_q = [
        current_tool_pose.pose.orientation.x,
        current_tool_pose.pose.orientation.y,
        current_tool_pose.pose.orientation.z,
        current_tool_pose.pose.orientation.w
    ]

    if relative:
        # get current orientation
        # note - in the example, they reference /driver/out/cartesian_command - so it is the last command, not the last position as we are doing here
        # last_orientation is in radians
        print("rev relative position", position)
        last_orientation_rad = Quaternion2EulerXYZ(last_orientation_q)
        position = [last_position[i] + position[i] for i in range(3)]
    if unit == 'mq':
        if relative:
            # current orientation is mq
            orientation_rad = [
                last_orientation_rad[i] + Quaternion2EulerXYZ(orientation)[i]
                for i in range(3)
            ]
            orientation_q = EulerXYZ2Quaternion(orientation_rad)
        else:
            orientation_q = orientation
        orientation_rad = Quaternion2EulerXYZ(orientation_q)
        orientation_deg = [math.degrees(orientation_rad[i]) for i in range(3)]
    if unit == 'mrad':
        if relative:
            orientation_rad = [
                last_orientation_rad[i] + orientation[i] for i in range(3)
            ]
        else:
            orientation_rad = orientation
        orientation_q = EulerXYZ2Quaternion(orientation_rad)
        orientation_deg = [math.degrees(orientation_rad[i]) for i in range(3)]
    if unit == 'mdeg':
        if relative:
            orientation_deg = [
                math.degrees(last_orientation_rad[i]) + orientation[i]
                for i in range(3)
            ]
        else:
            orientation_deg = orientation
        orientation_rad = [math.radians(orientation_deg[i]) for i in range(3)]
        orientation_q = EulerXYZ2Quaternion(orientation_rad)
    return position, orientation_q, orientation_rad, orientation_deg


def convert_joint_angles(current_joint_angle_radians, unit, relative,
                         target_joint_position):
    """
    unit: describes the unit of the command - must be 'deg' or 'rad' 
    is_relative: bool indicative whether or not the pose command is relative to the current position or absolute
    target_joint_position: relative or absolute joint position in degrees or radians. TODO size of input
    """

    assert unit in ['mdeg', 'mrad']
    # current joint estimate is in degrees
    if relative:
        current_joint_angle_degrees = [
            np.rad2deg(current_joint_angle_radians[i])
            for i in range(len(current_joint_angle_radians))
        ]
    if unit == 'mdeg':
        # get absolute value
        if relative:
            target_joint_degree = [
                target_joint_position[i] + current_joint_angle_degrees[i]
                for i in range(len(target_joint_position))
            ]
        else:
            target_joint_degree = target_joint_position
        target_joint_radian = list(map(math.radians, target_joint_degree))
    elif unit == 'mrad':
        if relative:
            # get absolute value
            target_joint_degree = [
                math.degrees(target_joint_position[i]) +
                current_joint_angle_degrees[i]
                for i in range(len(target_joint_position))
            ]
        else:
            target_joint_degree = list(map(math.degrees,
                                           target_joint_position))
        target_joint_radian = list(map(math.radians, target_joint_degree))
    print(target_joint_radian)
    return target_joint_degree, target_joint_radian


def convert_finger_pose(current_finger_pose, unit, relative, finger_value):


    current_finger_pose = [current_finger_pose.finger1,
                           current_finger_pose.finger2,
                           current_finger_pose.finger3]

    # Max distance for one finger in meter
    finger_maxDist = 18.9 / 2 / 1000
    # Max thread turn for one finger
    finger_maxTurn = 6800
    # transform between units
    if unit == 'turn':
        # get absolute value
        if relative:
            finger_turn_absolute_ = [
                finger_value[i] + current_finger_pose[i]
                for i in range(0, len(finger_value))
            ]
        else:
            finger_turn_absolute_ = finger_value

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [
            x * finger_maxDist / finger_maxTurn for x in finger_turn_
        ]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]

    elif unit == 'mm':
        # get absolute value
        finger_turn_command = [
            x / 1000 * finger_maxTurn / finger_maxDist for x in finger_value
        ]
        if relative:
            finger_turn_absolute_ = [
                finger_turn_command[i] + current_finger_pose[i]
                for i in range(0, len(finger_value))
            ]
        else:
            finger_turn_absolute_ = finger_turn_command

        finger_turn = finger_turn_absolute_
        finger_meter = [
            x * finger_maxDist / finger_maxTurn for x in finger_turn
        ]
        finger_percent = [x / finger_maxTurn * 100.0 for x in finger_turn]
    elif unit == 'percent':
        # get absolute value
        finger_turn_command = [
            x / 100.0 * finger_maxTurn for x in finger_value
        ]
        if relative:
            finger_turn_absolute_ = [
                finger_turn_command[i] + current_finger_pose[i]
                for i in range(0, len(finger_value))
            ]
        else:
            finger_turn_absolute_ = finger_turn_command

        finger_turn = finger_turn_absolute_
        finger_meter = [
            x * finger_maxDist / finger_maxTurn for x in finger_turn
        ]
        finger_percent = [x / finger_maxTurn * 100.0 for x in finger_turn]
    else:
        raise Exception("Finger value have to be in turn, mm or percent")

    return finger_turn, finger_meter, finger_percent


def trim_target_pose_safety(position, minx, maxx, miny, maxy, minz, maxz):
    """
    take in a position list [x,y,z] and ensure it doesn't violate the defined fence
    """
    x, y, z = position
    fence_result = ''
    if maxx < x:
        print('HIT FENCE: maxx of {} is < x of {}'.format(maxx, x))
        x = maxx
        fence_result += '+MAXFENCEX'
    if x < minx:
        print('HIT FENCE: x of {} < miny {}'.format(x, minx))
        x = minx
        fence_result += '+MINFENCEX'
    if maxy < y:
        print('HIT FENCE: maxy of {} is < y {}'.format(maxy, y))
        y = maxy
        fence_result += '+MAXFENCEY'
    if y < miny:
        print('HIT FENCE: y of {} is  miny of {}'.format(y, miny))
        y = miny
        fence_result += 'MINFENCEY'
    if maxz < z:
        print('HIT FENCE: maxz of {} is < z of {}'.format(maxz, z))
        z = maxz
        fence_result += 'MAXFENCEZ'
    if z < minz:
        print('HIT FENCE: z of {} < minz of {}'.format(z, minz))
        z = minz
        fence_result += 'MINFENCEZ'
    return [x, y, z], fence_result


def trim_target_pose_safety(position, minx, maxx, miny, maxy, minz, maxz):
    """
    take in a position list [x,y,z] and ensure it doesn't violate the defined fence
    """
    x, y, z = position
    fence_result = ''
    if maxx < x:
        print('HIT FENCE: maxx of {} is < x of {}'.format(maxx, x))
        x = maxx
        fence_result += '+MAXFENCEX'
    if x < minx:
        print('HIT FENCE: x of {} < miny {}'.format(x, minx))
        x = minx
        fence_result += '+MINFENCEX'
    if maxy < y:
        print('HIT FENCE: maxy of {} is < y {}'.format(maxy, y))
        y = maxy
        fence_result += '+MAXFENCEY'
    if y < miny:
        print('HIT FENCE: y of {} is  miny of {}'.format(y, miny))
        y = miny
        fence_result += 'MINFENCEY'
    if maxz < z:
        print('HIT FENCE: maxz of {} is < z of {}'.format(maxz, z))
        z = maxz
        fence_result += 'MAXFENCEZ'
    if z < minz:
        print('HIT FENCE: z of {} < minz of {}'.format(z, minz))
        z = minz
        fence_result += 'MINFENCEZ'
    return [x, y, z], fence_result
