#!/usr/bin/env python3
# -- coding: utf-8 --
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import sys
import os
import cubic_spline_planner as cubic_spline_planner 

from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, ColorRGBA, Float32
from geometry_msgs.msg import Point, Vector3
from enum import IntEnum

# 시작 전
# - rosrun lidar_cpp lane

class k_city_MODE(IntEnum):
    Pre = 0
    Final = 1

class Location(IntEnum):
    out_toolgate = 0
    in_toolgate = 1

class ChangeLocation(IntEnum):
    out_toolgate = 1
    out2in = 2
    in2out = 3

class LidarLaneDetection:
    def __init__(self):
        # self.lidar_sub = rospy.Subscriber('/minmax_ransac', PointCloud2, self.lidar_callback, queue_size=1)
        self.z_max_sub = rospy.Subscriber('/ask_to_sebin', Float32, self.z_max_callback,queue_size=1)

        self.left_lane_pub = rospy.Publisher('/left_lane_points', PointCloud2, queue_size=1)
        self.far_left_lane_pub = rospy.Publisher('/far_left_lane_points', PointCloud2, queue_size=1)
        self.right_lane_pub = rospy.Publisher('/right_lane_points', PointCloud2, queue_size=1)
        
        self.left_vizpath_pub = rospy.Publisher('/left_lane_vizpath', Marker, queue_size=1)
        self.far_left_vizpath_pub = rospy.Publisher('/far_left_lane_vizpath', Marker, queue_size=1)
        self.right_vizpath_pub = rospy.Publisher('/right_lane_vizpath', Marker, queue_size=1)
        self.center_vizpath_pub = rospy.Publisher('/center_lane_vizpath', Marker, queue_size=1)
        
        self.MODE = k_city_MODE.Pre # False: 예선, True: 본선
        self.points = np.array([])
        self.z_max_value = None
        
        self.obs_stop = False   
        self.prev_location = None
        self.location = None
        self.out_cnt = 0
        self.in_cnt = 0
        self.change_location = 0

        self.centroid_points = []
        self.prev_left_waypoints = np.array([])
        self.prev_far_left_waypoints = np.array([])
        self.prev_right_waypoints = np.array([])
        
    def z_max_callback(self, msg: Float32):
        self.z_max_value = msg.data
        
    def lidar_callback(self, lidar_msg: PointCloud2, threshold: int = 45, z_max_thres: int = 2.65, left_offset: int = 1.5, right_offset: int = 2.0):
        points = list(pc2.read_points(lidar_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        z_max = self.z_max_value

        center_waypoints = np.array([])
        right_waypoints = np.array([])
        left_waypoints = np.array([])
        
        if len(points) > 0 and z_max is not None:
            
            self.points = np.array(points)


            # ====================================================
            # toolgate 들어가기 전, 들어감, 들어가고 난 후 -> 3가지
            self.prev_location = self.location
                        
            if z_max < z_max_thres:
                self.out_cnt += 1
                # 값 범위 overflow 방지
                if self.out_cnt >= 10000:
                    self.out_cnt *= 0.1
                
                if self.out_cnt >= threshold:
                    self.location = Location.out_toolgate
                    self.in_cnt = 0
            else:
                self.in_cnt += 1
                if self.in_cnt >= 10000:
                    self.in_cnt *= 0.1
                
                if self.in_cnt >= threshold:
                    self.location = Location.in_toolgate
                    self.out_cnt = 0
            
            # (None, out)값이 있어서 +1이 초기값
            if self.prev_location != self.location:
                self.change_location += 1
                
                if self.change_location > 3:
                    self.change_location = 3
                
            # ====================================================
            # print("MODE: ", self.MODE) # False: 예선, True: 본선
            # print('max: ', z_max)
            # print('location: ', self.change_location)
            # print('out_cnt: ', self.out_cnt)
            # print('in_cnt: ', self.in_cnt)
            # print('*'*20)
            
            # 예선전
            if not self.MODE:
                # 오른쪽 차선만 봐야함: out 상태
                if self.change_location == int(ChangeLocation.out_toolgate):
                    right_roi_pointcloud = self.apply_roi(self.points, left=False, right=True, far_left=False)
                    right_centroid_pointcloud = self.apply_dbscan(right_roi_pointcloud)
                    
                    if len(right_centroid_pointcloud) >= 2:
                        right_waypoints = self.right_spline_interpolation(right_centroid_pointcloud)
                        if right_waypoints is not None:
                            center_waypoints = right_waypoints.copy()
                            center_waypoints[:, 1] = right_waypoints[:, 1] + right_offset
                            
                            # self.right_visualize_path(right_waypoints)
                            # self.center_visualize_path(center_waypoints)
                        
                        # self.publish_right_points(right_centroid_pointcloud)
                
                # 양쪽 차선만 봐야함: in 상태
                elif self.change_location == int(ChangeLocation.out2in):
                    right_roi_pointcloud = self.apply_roi(self.points, left=False, right=True, far_left=False)
                    right_centroid_pointcloud = self.apply_dbscan(right_roi_pointcloud)
                    
                    left_roi_pointcloud = self.apply_roi(self.points, left=True, right=False, far_left=False)
                    left_centroid_pointcloud = self.apply_dbscan(left_roi_pointcloud)
                    
                    far_left_roi_pointcloud = self.apply_roi(self.points, left=False, right=False, far_left=True)
                    far_left_centroid_pointcloud = self.apply_dbscan(far_left_roi_pointcloud, far_left=True)
                    
                    if len(right_centroid_pointcloud) >= 2: # 오른쪽 차선
                        right_waypoints = self.right_spline_interpolation(right_centroid_pointcloud)
                        if right_waypoints is not None:
                            # self.right_visualize_path(right_waypoints)
                            pass
                        
                    if len(left_centroid_pointcloud) >= 2: # 왼쪽 차선
                        left_waypoints = self.left_spline_interpolation(left_centroid_pointcloud)
                        if left_waypoints is not None:
                            # self.left_visualize_path(left_waypoints)
                            pass
                    
                    if len(far_left_centroid_pointcloud) >= 2: # 맨 왼쪽 차선
                        # far_left_waypoints =  self.far_left_spline_interpolation(far_left_centroid_pointcloud)
                        # self.far_left_visualize_path(far_left_waypoints)
                        pass
                
                    if left_waypoints is not None and right_waypoints is not None:
                        if len(left_waypoints) > 10 or len(right_waypoints) > 10: # 왼/오른쪽 차선만 보고, 중앙선 추종
                            if len(left_waypoints) > 10 and len(right_waypoints) > 10:
                                min_length = min(len(left_waypoints), len(right_waypoints))
                                center_list = []
                                for index in range(min_length):
                                    center_list.append((left_waypoints[index] + right_waypoints[index]) / 2)
                                
                                center_waypoints = np.array(center_list)
                            
                            # 왼쪽 차선만 보일 때
                            elif len(left_waypoints) > 10:
                                center_waypoints = left_waypoints.copy()
                                center_waypoints[:, 1] = left_waypoints[:, 1] - left_offset
                        
                            # 오른쪽 차선만 보일 때
                            elif len(right_waypoints) > 10:
                                center_waypoints = right_waypoints.copy()
                                center_waypoints[:, 1] = right_waypoints[:, 1] + right_offset
                            
                            # self.center_visualize_path(center_waypoints)    
                        
                        # self.publish_right_points(right_centroid_pointcloud)
                        # self.publish_left_points(left_centroid_pointcloud)
                        # self.publish_far_left_points(far_left_centroid_pointcloud)
                    
                # 왼쪽 차선만 봐야함: in->out상태
                elif self.change_location == int(ChangeLocation.in2out):
                    left_roi_pointcloud = self.apply_roi(self.points, left=True, right=False, far_left=False)
                    left_centroid_pointcloud = self.apply_dbscan(left_roi_pointcloud)
                    
                    if len(left_centroid_pointcloud) >= 2:
                        left_waypoints = self.left_spline_interpolation(left_centroid_pointcloud)
                        center_waypoints = left_waypoints.copy()
                        center_waypoints[:, 1] = left_waypoints[:, 1] - left_offset
                        
                        # self.left_visualize_path(left_waypoints)
                        # self.center_visualize_path(center_waypoints)
                
                    # self.publish_left_points(left_centroid_pointcloud)

                return center_waypoints ,left_waypoints, right_waypoints
            
            # 본선전 -> 왼/오른쪽 차선
            if self.MODE:
                right_roi_pointcloud = self.apply_roi(self.points, left=False, right=True, far_left=False)
                right_centroid_pointcloud = self.apply_dbscan(right_roi_pointcloud)
                
                left_roi_pointcloud = self.apply_roi(self.points, left=True, right=False, far_left=False)
                left_centroid_pointcloud = self.apply_dbscan(left_roi_pointcloud)
                
                if len(right_centroid_pointcloud) >= 2: # 오른쪽 차선
                    right_waypoints = self.right_spline_interpolation(right_centroid_pointcloud)
                    if right_waypoints is not None:
                        self.right_visualize_path(right_waypoints)
                    
                if len(left_centroid_pointcloud) >= 2: # 왼쪽 차선
                    left_waypoints = self.left_spline_interpolation(left_centroid_pointcloud)
                    if left_waypoints is not None:
                        self.left_visualize_path(left_waypoints)
                
                if left_waypoints is not None and right_waypoints is not None:
                    if len(left_waypoints) > 10 or len(right_waypoints) > 10: # 왼/오른쪽 차선만 보고, 중앙선 추종
                        if len(left_waypoints) > 10 and len(right_waypoints) > 10:
                            min_length = min(len(left_waypoints), len(right_waypoints))
                            center_list = []
                            for index in range(min_length):
                                center_list.append((left_waypoints[index] + right_waypoints[index]) / 2)
                            
                            center_waypoints = np.array(center_list)
                        
                        # 왼쪽 차선만 보일 때
                        elif len(left_waypoints) > 10:
                            center_waypoints = left_waypoints.copy()
                            center_waypoints[:, 1] = left_waypoints[:, 1] - left_offset
                    
                        # 오른쪽 차선만 보일 때
                        elif len(right_waypoints) > 10:
                            center_waypoints = right_waypoints.copy()
                            center_waypoints[:, 1] = right_waypoints[:, 1] + right_offset
                        # self.center_visualize_path(center_waypoints)    
                    
                    self.publish_right_points(right_centroid_pointcloud)
                    self.publish_left_points(left_centroid_pointcloud)

                return center_waypoints ,left_waypoints, right_waypoints


    def apply_roi(self, points: PointCloud2, left: bool = False, right: bool = False, far_left: bool = False,
                  min_x: int = 0.5, max_x = 10, far_left_min_y: int = 3.25, left_min_y: int = 2.75, right_min_y: int = -4.0, right_max_y: int = 0.0, intensity_thres: int = 50):
        x_values = points[:, 0]
        y_values = points[:, 1]
        intensity_values = points[:, 3]
        
        x_mask = (x_values >= min_x) & (x_values <= max_x)
        
        right_y_mask = (y_values <= right_max_y) & (y_values >= right_min_y) # right lane
        left_y_mask = (y_values >= 0) &(y_values <= left_min_y) # left lane
        far_left_y_mask = (y_values > far_left_min_y) # far left lane
        intensity_mask = (intensity_values >= intensity_thres)
        
        # 오른쪽 차선
        if right and not left:
            right_mask = intensity_mask & x_mask & right_y_mask
            points = points[right_mask]
            
            return points
        
        # 왼쪽 차선
        elif left and not right:
            left_mask = intensity_mask & x_mask & left_y_mask
            points = points[left_mask]
        
            return points

        # 톨게이트 안 맨 왼쪽 차선
        elif not left and not right and far_left:
            far_left_mask = intensity_mask & x_mask & far_left_y_mask
            points = points[far_left_mask]
            
            return points

    def apply_dbscan(self, point_cloud: np.ndarray, far_left: bool = False, eps: float = 0.05, min_points: int = 3) -> np.ndarray:
        o3d_point_cloud = o3d.geometry.PointCloud()
        o3d_point_cloud.points = o3d.utility.Vector3dVector(point_cloud[:, :3])
        
        clusters = {}
        if o3d_point_cloud.points:
            labels = np.array(o3d_point_cloud.cluster_dbscan(eps=eps, min_points=min_points))
            o3d_point_cloud_np = np.asarray(o3d_point_cloud.points)
            for label in set(labels):
                if label == -1:
                    continue
                cluster_points = o3d_point_cloud_np[labels == label]
                clusters[label] = cluster_points

        centroids = []
        for _, points in clusters.items():
            centroid = np.mean(points, axis=0)
            centroids.append(centroid)
        
        sorted_result = sorted(centroids, key=lambda item: item[0])
        
        if len(sorted_result) >  5:
            sorted_result = sorted_result[:5]
        
        elif len(sorted_result) < 3 and not far_left:
            return np.array([])
        
        return np.array(sorted_result)

    def right_spline_interpolation(self, centroids: np.ndarray, curvature_threshold: int = 0.3):
        if self.change_location == int(ChangeLocation.out2in):
            curvature_threshold = 0.05
            
        waypoint = np.empty((0,3))
        
        path_x = [point[0] for point in centroids]
        path_y = [point[1] for point in centroids]
        
        cx, cy, cyaw, ck, _, _ = cubic_spline_planner.calc_spline_course(path_x, path_y, ds=0.1)
        for i in range(len(cx)):
            point=[cx[i],cy[i],cyaw[i]]
            waypoint = np.vstack((waypoint, np.array(point)))
        
        ck_np = np.array(ck)
        ck_np = np.abs(ck_np)

        for index, ck_np_value in enumerate(ck_np):
            if ck_np_value == 0:
                ck_np[index] += 0.00001
                
        non_zero_curvatures = ck_np[ck_np != 0]
        non_zero_max_curvature = max(non_zero_curvatures)
        
        if non_zero_max_curvature < curvature_threshold:
            self.prev_right_waypoints = waypoint
            return waypoint
        
        elif self.obs_stop:
            return self.prev_right_waypoints

        elif non_zero_max_curvature >= curvature_threshold and len(self.prev_right_waypoints) > 0 and not self.obs_stop:
            return self.prev_right_waypoints
        
    def right_visualize_path(self, waypoints):
        rviz_msg_path=Marker(
            header=Header(frame_id='velodyne', stamp=rospy.get_rostime()),
            ns="parkings_path",
            id=190,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.1,0.0,0.0),
            color=ColorRGBA(r=1.0,g=0.0,b=1.0,a=0.8)
        )
        for waypoint in waypoints[:-1]:
            p = Point()
            p.x = waypoint[0]
            p.y = waypoint[1]
            p.z = 0.1
            rviz_msg_path.points.append(p)
        self.right_vizpath_pub.publish(rviz_msg_path)

    def left_spline_interpolation(self, centroids: np.ndarray, curvature_threshold: int = 0.3):
        if self.change_location == int(ChangeLocation.out2in):
            curvature_threshold = 0.05

        waypoint=np.empty((0,3))
        
        path_x = [point[0] for point in centroids]
        path_y = [point[1] for point in centroids]
        
        cx, cy, cyaw, ck, _, _ = cubic_spline_planner.calc_spline_course(path_x, path_y, ds=0.1)
        for i in range(len(cx)):
            point=[cx[i],cy[i],cyaw[i]]
            waypoint = np.vstack((waypoint, np.array(point)))
        
        ck_np = np.array(ck)
        ck_np = np.abs(ck_np)

        for index, ck_np_value in enumerate(ck_np):
            if ck_np_value == 0:
                ck_np[index] += 0.00001
                
        non_zero_curvatures = ck_np[ck_np != 0]
        non_zero_max_curvature = max(non_zero_curvatures)
        

        if non_zero_max_curvature < curvature_threshold:
            self.prev_left_waypoints = waypoint
            return waypoint
        
        elif self.obs_stop:
            return self.prev_left_waypoints
            
        elif non_zero_max_curvature >= curvature_threshold and len(self.prev_left_waypoints) > 0 and not self.obs_stop:
            return self.prev_left_waypoints
        
        
    def left_visualize_path(self, waypoints):
        rviz_msg_path=Marker(
            header=Header(frame_id='velodyne', stamp=rospy.get_rostime()),
            ns="parkings_path",
            id=190,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.1,0.0,0.0),
            color=ColorRGBA(r=0.0,g=1.0,b=1.0,a=0.8)
        )
        for waypoint in waypoints[:-1]:
            p = Point()
            p.x = waypoint[0]
            p.y = waypoint[1]
            p.z = 0.1
            rviz_msg_path.points.append(p)
        self.left_vizpath_pub.publish(rviz_msg_path)

    def far_left_spline_interpolation(self, centroids: np.ndarray, curvature_threshold: int = 0.3):
        waypoint=np.empty((0,3))
        
        path_x = [point[0] for point in centroids]
        path_y = [point[1] for point in centroids]
        
        cx, cy, cyaw, ck, _, _ = cubic_spline_planner.calc_spline_course(path_x, path_y, ds=0.1)
        for i in range(len(cx)):
            point=[cx[i],cy[i],cyaw[i]]
            waypoint = np.vstack((waypoint, np.array(point)))
        
        ck_np = np.array(ck)
        ck_np = np.abs(ck_np)
        
        for index, ck_np_value in enumerate(ck_np):
            if ck_np_value == 0:
                ck_np[index] += 0.00001
        
        non_zero_curvatures = ck_np[ck_np != 0]
        non_zero_max_curvature = max(non_zero_curvatures)
        
        if non_zero_max_curvature < curvature_threshold:
            self.prev_far_left_waypoints = waypoint
            return waypoint
        
        elif non_zero_max_curvature >= curvature_threshold and len(self.prev_far_left_waypoints) > 0:
            return self.prev_far_left_waypoints
        
    def far_left_visualize_path(self, waypoints):
        rviz_msg_path=Marker(
            header=Header(frame_id='velodyne', stamp=rospy.get_rostime()),
            ns="parkings_path",
            id=190,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.1,0.0,0.0),
            color=ColorRGBA(r=0.5,g=0.5,b=0.5,a=0.8)
        )
        for waypoint in waypoints[:-1]:
            p = Point()
            p.x = waypoint[0]
            p.y = waypoint[1]
            p.z = 0.1
            rviz_msg_path.points.append(p)
        self.far_left_vizpath_pub.publish(rviz_msg_path)

    def center_visualize_path(self, waypoints):
        rviz_msg_path=Marker(
            header=Header(frame_id='velodyne', stamp=rospy.get_rostime()),
            ns="parkings_path",
            id=182,
            type=Marker.LINE_STRIP,
            lifetime=rospy.Duration(0.5),
            action=Marker.ADD,
            scale=Vector3(0.1,0.0,0.0),
            color=ColorRGBA(r=0.0,g=0.0,b=1.0,a=0.8)
        )
        
        for waypoint in waypoints[:-1]:
            p = Point()
            p.x = waypoint[0]
            p.y = waypoint[1]
            p.z = 0.1
            rviz_msg_path.points.append(p)
        self.center_vizpath_pub.publish(rviz_msg_path)
        
    def publish_right_points(self, result_pointcloud: np.ndarray):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'velodyne'  

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        right_cloud = pc2.create_cloud(header, fields, result_pointcloud)

        self.right_lane_pub.publish(right_cloud)
    
    def publish_left_points(self, result_pointcloud: np.ndarray):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'velodyne'  

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        left_cloud  = pc2.create_cloud(header, fields, result_pointcloud)

        self.left_lane_pub.publish(left_cloud)

    def publish_far_left_points(self, result_pointcloud: np.ndarray):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'velodyne'  

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        far_left_cloud  = pc2.create_cloud(header, fields, result_pointcloud)

        self.far_left_lane_pub.publish(far_left_cloud)
        
# def main():
#     rospy.init_node("lidar_lane_detection", anonymous=True)
#     lidar = LidarLaneDetection()
#     rospy.spin()

# if __name__ == "__main__":
#     main()