# Copyright 2023 Bonn-Rhein-Sieg University
#
# Author: Shubham Shinde.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Optional

import os
import rclpy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from ultralytics import YOLO
import torch

# Node, State and Publisher are aliases for LifecycleNode, LifecycleState and LifecyclePublisher
# respectively.
# In case of ambiguity, the more explicit names can be imported.
from rclpy.action import ActionServer
from rclpy.node import Node

from mir_interfaces.action import ObjectDetection
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from std_msgs.msg import ColorRGBA, String
from sensor_msgs.msg import Image, RegionOfInterest, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from mir_interfaces.msg import ImageList, Object, ObjectList, Cavity
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_share_directory 
import rclpy.qos as qos
import message_filters

import sensor_msgs_py.point_cloud2 as point_cloud2
import ros2_numpy
from sklearn.decomposition import PCA
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf_transformations as tr
from scipy.spatial.transform import Rotation as R
import time
import threading

class PPT_LifecycleTalker(Node):

    def __init__(self, node_name, **kwargs):
        """Construct the node."""
        self.object_name = None
        self.cvbridge = None
        self.debug = False
        self.weights = None
        self.pub_debug = None
        self.pub_pose = None
        self.pub_cropped_pc = None

        self.sub_img = None
        self.sub_point_cloud = None
        self.image = None
        self.pointcloud = None
        self.ts = None
        self.tf_buffer = None
        self.cavity_pose_sent = False
        
        self.model = None
        self._action_server = None
        self.tf_listener = None
        self.goal_handle = None
        self.data_lock_ = None

        super().__init__(node_name, **kwargs)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested.

        on_configure callback is being called when the lifecycle node
        enters the "configuring" state.

        :return: The state machine either invokes a transition to the "inactive" state or stays
            in "unconfigured" depending on the return value.
            TransitionCallbackReturn.SUCCESS transitions to "inactive".
            TransitionCallbackReturn.FAILURE transitions to "unconfigured".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.get_logger().info('on_configure() is called.')
        
        self.object_name = "S40_40_H"
        self.cvbridge = CvBridge()
        self.debug = True

        self.weights = os.path.join(get_package_share_directory("mir_rgb_object_recognition_models"),
                            'common', 'models', "yolov8", "robocup_2023_dataset", "cavity.pt")

        self.pub_debug = self.create_lifecycle_publisher(Image, "/mir_perception/mir_ppt_detection/rgb/output/ppt_debug_image", qos_profile=qos.qos_profile_sensor_data)
        self.pub_pose = self.create_lifecycle_publisher(PoseStamped, "/mir_perception/mir_ppt_detection/cavity_pose", qos_profile=qos.qos_profile_system_default)
        self.pub_cropped_pc = self.create_lifecycle_publisher(PointCloud2, "/mir_perception/mir_ppt_detection/pc/output/cropped_pc", qos_profile=qos.qos_profile_sensor_data)
        
        # Initialize
        self.device = torch.device("cpu")
        # Load model
        self.model = YOLO(self.weights)    
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10))    
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._action_server = ActionServer(self, ObjectDetection, 'ppt_detection', 
                                           self.execute_callback, 
                                           goal_callback=self.goal_callback,
                                           handle_accepted_callback=self.handle_accepted_callback)
        self.data_lock_ = threading.Lock()
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Differently to rclcpp, a lifecycle publisher transitions automatically between the
        # inactive and enabled state and viceversa.
        # For that reason, we only need to write an on_configure() and on_cleanup() callbacks,
        # and we don't need to write on_activate()/on_deactivate() callbacks.

        # Log, only for demo purposes
        self.get_logger().info('on_activate() is called.')

        # Subscribe to image topic
        self.sub_img = message_filters.Subscriber(self, Image, "/camera/color/image_raw", qos_profile=qos.qos_profile_sensor_data)

        # subscribe to point cloud topic
        self.sub_point_cloud = message_filters.Subscriber(self, PointCloud2, "/camera/depth/color/points", qos_profile=qos.qos_profile_sensor_data)
        
        # synchronize image and point cloud
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_img, self.sub_point_cloud], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.time_sync_callback)

        # The default LifecycleNode callback is the one transitioning
        # LifecyclePublisher entities from inactive to enabled.
        # If you override on_activate(), don't forget to call the parent class method as well!!
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Log, only for demo purposes
        self.get_logger().info('on_deactivate() is called.')
        # Same reasong here that for on_activate().
        # These are the two only cases where you need to call the parent method.
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Cleanup the node, after a cleaning-up transition is requested.

        on_cleanup callback is being called when the lifecycle node
        enters the "cleaning up" state.

        :return: The state machine either invokes a transition to the "unconfigured" state or stays
            in "inactive" depending on the return value.
            TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
            TransitionCallbackReturn.FAILURE transitions to "inactive".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.get_logger().info('on_cleanup() is called.')
        self.destroy_subscription(self.sub_img.sub)
        self.destroy_subscription(self.sub_point_cloud.sub)
        self.destroy_publisher(self.pub_debug)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Shutdown the node, after a shutting-down transition is requested.

        on_shutdown callback is being called when the lifecycle node
        enters the "shutting down" state.

        :return: The state machine either invokes a transition to the "finalized" state or stays
            in the current state depending on the return value.
            TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
            TransitionCallbackReturn.FAILURE transitions to "inactive".
            TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.get_logger().info('on_shutdown() is called.')
        self.destroy_subscription(self.sub_img.sub)
        self.destroy_subscription(self.sub_point_cloud.sub)
        self.destroy_publisher(self.pub_debug)

        return TransitionCallbackReturn.SUCCESS
    
    def execute_callback(self, goal_handle):
        # acquire the lock
        self.data_lock_.acquire()
        result = ObjectDetection.Result()

        if self.image:
            try:
                cv_img = self.cvbridge.imgmsg_to_cv2(self.image, "bgr8")
                bboxes, probs, labels, predictions = self.yolo_detect(cv_img)
                # Capatilize labels
                labels = [label.upper() for label in labels]
                print("labels: ", labels) 
                
                for bbox, prob, label in zip(bboxes, probs, labels):
                    # TODO: Do this for all cavities. Use message type Cavity.
                    if label == self.object_name:
                        cavity_pose = self.get_cavity_pose(self.pointcloud, bbox)
                        self.pub_pose.publish(cavity_pose)
                        self.cavity_pose_sent = True
                    else:
                        continue
                if self.debug:
                    # Draw bounding boxes and labels of detections
                    debug_img = predictions[0].plot()
                    # publish bbox and label
                    self.publish_debug_img(debug_img)
                
                goal_handle.succeed()
                self.get_logger().info('Goal succeeded!')
                
            except CvBridgeError as e:
                self.get_logger().error(e)
        else:
            self.get_logger().warn("No image received")
            goal_handle.abort()
        
        # release the lock
        self.data_lock_.release()
        result.result = bool(self.cavity_pose_sent)
        return result

    
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        if self.data_lock_.locked():
            self.get_logger().info('Goal request rejected - action already in progress')
            return GoalResponse.REJECT
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle):
        """Provide a handle to an accepted goal."""
        self.get_logger().info('Execution of goal accepted')
        self.goal_handle = goal_handle      
        self.goal_handle.execute()

    def time_sync_callback(self, image, pointcloud: PointCloud2):
        """
        image: image data of current frame with Image data type
        pointcloud: pointcloud data of current frame with PointCloud2 data type
        """
        if not self.data_lock_.locked():
            self.image = image
            self.pointcloud = pointcloud
    
    def yolo_detect(self, cv_img):
        predictions = self.model.predict(source=cv_img,
                                            conf=0.8, iou=0.45,
                                            device='cpu', verbose=False)
        # convert results to numpy array
        predictions_np = predictions[0].boxes.numpy()
        class_ids = predictions_np.cls
        class_names = predictions[0].names
        class_labels = [class_names[i] for i in class_ids]
        class_scores = predictions_np.conf
        class_bboxes = predictions_np.xyxy # x, y, w, h

        return class_bboxes, class_scores, class_labels, predictions
    
    def get_cavity_pose(self, point_cloud, bounding_box):
        if point_cloud is None or bounding_box is None:
            return
        
        # Extract point cloud data within the bounding box
        cropped_cloud = self.crop_point_cloud(point_cloud, bounding_box)

        # Calculate the centroid (mean point) of the point cloud
        centroid = np.mean(cropped_cloud, axis=0)

        # normalize pointcloud with centroid
        cropped_cloud = cropped_cloud - centroid

        # Apply Principal Component Analysis (PCA)
        pca = PCA(n_components=3)
        pca.fit(cropped_cloud)

        # Get the eigenvalues and eigenvectors
        # eigenvalues = pca.explained_variance_
        eigenvectors = pca.components_
        # print("eigenvalues: ", eigenvalues, "\neigenvectors: \n", eigenvectors)
        # Calculate the pose using PCA results
        pose = self.calculate_pose(point_cloud, centroid, eigenvectors)
        return pose

    def crop_point_cloud(self, point_cloud, bounding_box):
        # Get the pointcloud data in numpy array format
        
        pc_data = list(point_cloud2.read_points(point_cloud, skip_nans=False, field_names=("x", "y", "z")))
        pc = np.array([np.array([x, y, z]) for x, y, z in pc_data], dtype=np.float32)
        pc = pc.reshape((480,640,3))

        # crop the point cloud data within the bounding box
        cropped_cloud = pc[int(bounding_box[1]):int(bounding_box[3]), int(bounding_box[0]):int(bounding_box[2])]
        cropped_cloud_shape = cropped_cloud.shape

        # Ensure the cropped_cloud is a 2D array
        cropped_cloud = np.reshape(cropped_cloud, (-1, 3))        
        
        # convert cropped_cloud data to ros pointcloud2 data type
        crp_cloud = np.core.records.fromarrays([cropped_cloud[:,0], cropped_cloud[:,1], cropped_cloud[:,2]], names='x, y, z')
        crp_pc = point_cloud2.create_cloud_xyz32(point_cloud.header, crp_cloud)
        
        try:
            trans = self.tf_buffer.lookup_transform("base_link", point_cloud.header.frame_id, rclpy.time.Time())   
        except tf2_ros.TransformException as ex:
            self.get_logger().info(
                f'Could not transform {"base_link"} to {point_cloud.header.frame_id}: {ex}')
            return
        
        cropped_cloud_wrt_base_link = do_transform_cloud(crp_pc, trans)

        cropped_cloud = list(point_cloud2.read_points(cropped_cloud_wrt_base_link, skip_nans=False, field_names=("x", "y", "z")))
        cropped_cloud = np.array([np.array([x, y, z]) for x, y, z in cropped_cloud], dtype=np.float32)
        cropped_cloud = cropped_cloud.reshape(cropped_cloud_shape)
        
        # get the min value of z in point cloud
        min_z = np.nanmin(cropped_cloud[:,:,2])
        max_z = np.nanmax(cropped_cloud[:,:,2])
        
        # crop the z of cropped_cloud to remove the ground
        # cropped_cloud = cropped_cloud[cropped_cloud[:,:,2] > min_z + 0.003]        
        cropped_cloud = cropped_cloud[cropped_cloud[:,:,2] < max_z - 0.003]        

        # remove nan values
        cropped_cloud = cropped_cloud[~np.isnan(cropped_cloud)]

        # Check if the cropped_cloud array is empty
        if cropped_cloud.size == 0:
            return None

        # Ensure the cropped_cloud is a 2D array
        cropped_cloud = np.reshape(cropped_cloud, (-1, 3))
        
        if self.debug:
            # Publish the cropped point cloud data
            crp_cloud = np.core.records.fromarrays([cropped_cloud[:,0], cropped_cloud[:,1], cropped_cloud[:,2]], names='x, y, z')
            crp_pc = point_cloud2.create_cloud_xyz32(point_cloud.header, crp_cloud)
            self.pub_cropped_pc.publish(crp_pc)

        # Return the cropped point cloud data as a numpy array
        return np.array(cropped_cloud)

    def calculate_pose(self, pointcloud, centroid, eigenvectors):
        pose = PoseStamped()
        pose.header.stamp = pointcloud.header.stamp
        pose.header.frame_id = "base_link"

        # Set the position of the object
        pose.pose.position.x = float(centroid[0])
        pose.pose.position.y = float(centroid[1])
        pose.pose.position.z = float(centroid[2])

        # Swap largest and second largest eigenvectors
        # eigenvectors[:, [0, 2]] = eigenvectors[:, [2, 0]]

        # Compute the cross product of the second largest and largest eigenvectors
        # cross_product = np.cross(eigenvectors[:, 2], eigenvectors[:, 0])

        # Assign the cross product to the second largest eigenvector
        # eigenvectors[:, 1] = cross_product

        # pdb.set_trace()
        ## This is incorrect
        # rot_mat = np.identity(4)
        # rot_mat[0:3, 0:3] = eigenvectors.T
        # rot_mat[:3, 3] = -(rot_mat[:3, :3] @ centroid[:3])
        # quaternion = tr.quaternion_from_matrix(rot_mat)
        rot = R.from_matrix(eigenvectors.T)
        quaternion = rot.as_quat()
        # eular = rot.as_euler('ZYX', degrees=True)
        # print(eular)
        
        roll, pitch, yaw = tr.euler_from_quaternion(quaternion)
        orientation = tr.quaternion_from_euler(0, 0, yaw)

        pose.pose.orientation.x = float(orientation[0])
        pose.pose.orientation.y = float(orientation[1])
        pose.pose.orientation.z = float(orientation[2])
        pose.pose.orientation.w = float(orientation[3])
        
        return pose
    
    def publish_debug_img(self, debug_img):
        debug_img = np.array(debug_img, dtype=np.uint8)
        if debug_img is not None and self.pub_debug is not None:
            debug_img = self.cvbridge.cv2_to_imgmsg(debug_img, "bgr8")
            self.pub_debug.publish(debug_img)

def main():
    print('Hi from mir_ppt_detection.')
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    ppt_detector = PPT_LifecycleTalker('ppt_detector')
    executor.add_node(ppt_detector)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        ppt_detector.destroy_node()

if __name__ == '__main__':
    main()
