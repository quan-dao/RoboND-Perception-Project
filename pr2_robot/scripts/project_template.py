#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = str(arm_name.data)
    yaml_dict["object_name"] = str(object_name.data)
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    
    # TODO: Statistical Outlier Filtering
    statiscal_filter = cloud.make_statistical_outlier_filter()  # create a filter object
    statiscal_filter.set_mean_k(10)  # set number of neighbor points involving in analyzing a given points
    x = 0.5  # mean distance threshold's scale factor
    # Any points have mean distance larger than threshold + x * standard deviation are considered outlier
    statiscal_filter.set_std_dev_mul_thresh(x)  
    cloud_filtered = statiscal_filter.filter()  # call the filter
    # statiscal_cloud = cloud_filtered  # for writeup 
    
    # TODO: Voxel Grid Downsampling
    voxel_filter = cloud_filtered.make_voxel_grid_filter()
    leaf_size = 0.01  # set size of volume element
    voxel_filter.set_leaf_size(leaf_size, leaf_size, leaf_size)
    cloud_filtered = voxel_filter.filter()

    # TODO: PassThrough Filter
    pass_through_z = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    pass_through_z.set_filter_field_name(filter_axis)
    axis_min = 0.62
    axis_max = 1.0
    pass_through_z.set_filter_limits(axis_min, axis_max)
    cloud_filtered = pass_through_z.filter()

    pass_through_y = cloud_filtered.make_passthrough_filter()
    filter_axis_2 = 'y'
    pass_through_y.set_filter_field_name(filter_axis_2)
    axis_min_2 = -0.55
    axis_max_2 = 0.55
    pass_through_y.set_filter_limits(axis_min_2, axis_max_2)
    cloud_filtered = pass_through_y.filter()

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)  # set segmentation model
    seg.set_method_type(pcl.SAC_RANSAC)  # set segmentation method
    max_distance = 0.005  # max distance for a point to be considered in the model
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()  # call the segment func to get inlier indices

    # TODO: Extract inliers and outliers
    ransac_objects = cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(ransac_objects)  # create spatial point cloud to prepare for k-d tree construction
    tree = white_cloud.make_kdtree()
    extracted_cluster = white_cloud.make_EuclideanClusterExtraction()  # create a cluster extraction object
    # set tolerances for distacnce threshold
    extracted_cluster.set_ClusterTolerance(0.025)
    extracted_cluster.set_MinClusterSize(50)
    extracted_cluster.set_MaxClusterSize(1000)
    extracted_cluster.set_SearchMethod(tree)  # search the k-d tree for cluster
    cluster_indices = extracted_cluster.Extract()  # extract list of points for each cluster. 

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()  # create a new cloud to store all the clusters, each with unique color
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    # ros_cloud = pcl_to_ros(yz_pass_through_cloud)
    ros_cluster_clould = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    # ransac_pub.publish(ros_cloud)
    cluster_pub.publish(ros_cluster_clould)

# Exercise-3 TODOs:
	# Create empty lists to receive object lables and object point cloud
    detected_objects = []
    detected_objects_labels = []
    for index, pts_list in enumerate(cluster_indices):
		# Grab the points for the cluster from the extracted outliers (ransac_objects)
        pcl_cluster = ransac_objects.extract(pts_list)
        # Convert pcl to ROS
        ros_cluster = pcl_to_ros(pcl_cluster)
        # Extract histogram feature
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        # Make the prediction, retrieve the label for the cluster and add this label to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
    	# Publish a label into RViz
    	label_pos = list(white_cloud[pts_list[0]])
    	label_pos[2] += .4
    	object_markers_pub.publish(make_label(label,label_pos, index))
    	# Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Print out the number and names of detected object
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)
   
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # 3 individual objects >>> check the number of object detected >>> check the duplication of objects' label
    
    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    num_objects = len(object_list_param)

    if len(detected_objects) == num_objects:
    	if len(set(detected_objects_labels)) == num_objects:  # use set to eleminate duplicate versions of any labels
    		# no coincident lables happened
    		# call pr2_mover()
    		try:
        		pr2_mover(detected_objects, object_list_param, dropbox_param)
    		except rospy.ROSInterruptException:
        		pass
        else:
        	pass
    else:
    	pass

 
# function to load parameters and request PickPlace service
def pr2_mover(object_list, object_list_param, dropbox_param):

    # TODO: Initialize variables
    dict_list = []

    # TODO: Parse parameters into individual variables
    sence_object_name = [object_list_param[i]['name'] for i in range(len(object_list_param))]
    sence_object_group = [object_list_param[i]['group'] for i in range(len(object_list_param))]

    # TODO: Rotate PR2 in place to capture side tables for the collision map
    for detected_object in object_list:   	
    	# TODO: Get the PointCloud for a given object and obtain it's centroid
    	points_arr = ros_to_pcl(detected_object.cloud).to_array()  # convert PC data to array
    	object_centroid = np.mean(points_arr, axis=0)[:3]  # centroid = mean 
    	# create the pick pose
    	pick_pose = Pose()
    	pick_pose.position.x = np.asscalar(object_centroid[0])
    	pick_pose.position.y = np.asscalar(object_centroid[1])
    	pick_pose.position.z = np.asscalar(object_centroid[2])

    	# TODO: Create 'place_pose' for the object
    	# find index of sence object having the same label as this detected object
    	sence_object_ind = sence_object_name.index(detected_object.label)
    	# check object's group to determine which dropbox to place this object
    	if sence_object_group[sence_object_ind] == 'green':
    		dropbox_ind = 1
    	else:
    		dropbox_ind = 0
    	# extract the choosen dropbox's position
    	dropbox_position = dropbox_param[dropbox_ind]['position']
    	place_pose = Pose()
    	place_pose.position.x = dropbox_position[0]
    	place_pose.position.y = dropbox_position[1]
    	place_pose.position.z = dropbox_position[2]

    	# TODO: Assign the arm to be used for pick_place
    	arm_name = String()
    	arm_name.data = dropbox_param[dropbox_ind]['name']

    	# TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
    	test_scene_num = Int32()
    	test_scene_num.data = 1
    	object_name = String()
    	object_name.data = detected_object.label 
    	yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
    	dict_list.append(yaml_dict)

    	# # Wait for 'pick_place_routine' service to come up
     #    rospy.wait_for_service('pick_place_routine')

     #    try:
     #        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

     #        # TODO: Insert your message variables to be sent as a service request
     #        resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

     #        print ("Response: ",resp.success)

     #    except rospy.ServiceException, e:
     #        print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    send_to_yaml('output_1.yaml', dict_list)




if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('perception_pipeline')

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    ransac_pub = rospy.Publisher("/ransac_segmentation", pc2.PointCloud2, queue_size=1)
    cluster_pub = rospy.Publisher("/extracted_cluster", pc2.PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model_reg_scene_1.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Get dropbox parameters
    dropbox_param = rospy.get_param('/dropbox')

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
    	rospy.spin()
