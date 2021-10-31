#!/usr/bin/env python3
import rospy
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import time

from rnrt_msgs.msg import Markers
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply
import math as m
import numpy as np
import pickle as pkl
from scipy.spatial.transform import Rotation
from threading import Thread, Lock


def setPose(x, y, z, quat):
    result = Pose()
    result.position.x = x
    result.position.y = y
    result.position.z = z
    result.orientation = quat
    return result


class Controller:
    _mutex = Lock()
    _offset = None
    _batch_size = 10
    _batch_counter = 0
    _output_data = list()

    _cm_z = [0.5, 0.5, 0.5, 0.5]

    _base_dist = 0.5

    _cm_angles = [0.0, 12.5, 25.0, 37.5]
    _cm_angles_rad = np.radians(_cm_angles)

    # _cm_x = [
    #     -_base_dist*m.cos(_cm_angles_rad[0]),
    #     -_base_dist*m.cos(_cm_angles_rad[1]),
    #     -_base_dist*m.cos(_cm_angles_rad[2]),
    #     -_base_dist*m.cos(_cm_angles_rad[3])
    # ]

    _cm_x = [
        -_base_dist,
        -_base_dist,
        -_base_dist,
        -_base_dist
    ]

    _cm_y = [
        _base_dist*m.tan(_cm_angles_rad[0]),
        _base_dist*m.tan(_cm_angles_rad[1]),
        _base_dist*m.tan(_cm_angles_rad[2]),
        _base_dist*m.tan(_cm_angles_rad[3])
    ]

    _angle_boundary = 30

    def process_data(self, data):

        output_lin = list()
        output_ang = list()

        for i in range(0, len(data)):
            xyz = [data[i].position.x,
                   data[i].position.y,
                   data[i].position.z]

            rot_msg = Rotation.from_quat([data[i].orientation.x,
                                          data[i].orientation.y,
                                          data[i].orientation.z,
                                          data[i].orientation.w])

            z_vec = rot_msg.as_dcm()[:, 2]

            distance_err = np.sqrt(
                pow(xyz[0] - self._cm_y[i], 2)
                + pow(xyz[1], 2)
                + pow(xyz[2] + self._cm_x[i], 2)
            )

            rot = Rotation.from_euler(
                'y', 180 - self._cm_angles[i] + self._offset, degrees=True)

            rotated_z = rot.apply(z_vec)

            angle_err = np.degrees(np.arccos(rotated_z[2]))

            angle_err = abs(angle_err)

            output_lin.append(distance_err)
            output_ang.append(angle_err)

        return output_lin, output_ang

    def call_detector(self, msg):

        if(self._mutex.locked()):
            return "dump"

        self._mutex.acquire()

        data = dict(
            sorted(zip(msg.marker_ids, msg.poses), key=lambda x: x[0]))

        if(len(data) < 4):
            print("*** Goes to dump ***")
            return "dump"

        if(self._batch_counter < self._batch_size):
            self._batch_counter += 1
            lin, ang = self.process_data(data)
            self._linear_err_list_current = [
                *self._linear_err_list_current, *lin]
            self._angular_err_list_current = [
                *self._angular_err_list_current, *ang]

        self._mutex.release()


    def pose_pub(self, q_rot, model_name, m_pose):
        state_msg = ModelState()
        state_msg.model_name = model_name
        state_msg.pose.position = m_pose.position

        state_msg.pose.orientation = Quaternion(
            *quaternion_multiply(q_rot, m_pose.orientation))

        rospy.wait_for_service("/gazebo/set_model_state")
        try:
            set_state = rospy.ServiceProxy(
                "/gazebo/set_model_state", SetModelState)
            resp = set_state(state_msg)
            # print(resp)

        except rospy.ServiceException as err:
            print("Service call failed: %s", err)

        time.sleep(0.0167)

    def __init__(self, batch_in=10, deg_in=30, distance_to_cm=1.0):
        self._mutex.acquire()
        rospack = rospkg.RosPack()
        self._package_path = rospack.get_path('ast_controller')

        self._batch_size = batch_in

        rospy.Subscriber("/detected_markers", Markers, self.call_detector)

        cm3_pose = setPose(self._cm_x[0], self._cm_y[0], self._cm_z[0],
                           quaternion_from_euler(m.pi, m.pi/2, m.pi - self._cm_angles_rad[0]))

        cm4_pose = setPose(self._cm_x[1], self._cm_y[1], self._cm_z[1],
                           quaternion_from_euler(m.pi, m.pi/2, m.pi - self._cm_angles_rad[1]))

        cm5_pose = setPose(self._cm_x[2], self._cm_y[2], self._cm_z[2],
                           quaternion_from_euler(m.pi, m.pi/2, m.pi - self._cm_angles_rad[2]))

        cm6_pose = setPose(self._cm_x[3], self._cm_y[3], self._cm_z[3],
                           quaternion_from_euler(m.pi, m.pi/2, m.pi - self._cm_angles_rad[3]))

        quat = quaternion_from_euler(0, 0, 0)
        self.pose_pub(quat, "cuboid_marker3", cm3_pose)
        self.pose_pub(quat, "cuboid_marker4", cm4_pose)
        self.pose_pub(quat, "cuboid_marker5", cm5_pose)
        self.pose_pub(quat, "cuboid_marker6", cm6_pose)

        self._angle_boundary = deg_in
        self._offset = -self._angle_boundary

        self._offset_list = list()
        self._linear_err_list = list()
        self._angular_err_list = list()
        self._linear_err_list_current = list()
        self._angular_err_list_current = list()

        self._file = open(self._package_path
                          + '/saved_data/'
                          + 'out_data_' + str(self._base_dist) + '.pkl', 'wb')
        counter = 0
        self._mutex.release()

        while(not rospy.is_shutdown()):

            self._mutex.acquire()

            quat = quaternion_from_euler(0, 0, np.radians(self._offset))
            time.sleep(0.02)

            self.pose_pub(quat, "cuboid_marker3", cm3_pose)
            self.pose_pub(quat, "cuboid_marker4", cm4_pose)
            self.pose_pub(quat, "cuboid_marker5", cm5_pose)
            self.pose_pub(quat, "cuboid_marker6", cm6_pose)

            time.sleep(0.017)
            # self._flag = 1
            self._mutex.release()

            while(self._batch_counter < self._batch_size):
                time.sleep(0.1)

            if(self._batch_counter == self._batch_size):
                self._mutex.acquire()
                self._offset_list.append(self._offset)
                self._linear_err_list.append(self._linear_err_list_current)
                self._angular_err_list.append(self._angular_err_list_current)

                self._linear_err_list_current = list()
                self._angular_err_list_current = list()
                self._batch_counter = 0
                self._mutex.release()

            self._offset = self._offset + 1.0
            print("Another batch! #" + str(counter))
            counter += 1

            if(self._offset > self._angle_boundary):
                output = [
                    self._offset_list,
                    self._linear_err_list,
                    self._angular_err_list
                ]

                # print(self._angular_err_list)

                pkl.dump(output, self._file)

                self._file.close()

                rospy.signal_shutdown("It time to shutdown...")


# ============================== MAIN goes here ========================================


if __name__ == '__main__':
    rospy.init_node("test_controller")

    try:
        controller = Controller(50, 25, 1.0)
    except rospy.ROSInterruptException:
        pass
