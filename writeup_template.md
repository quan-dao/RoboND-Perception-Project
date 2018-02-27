## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

# [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
## Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
## Writeup / README

### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented

[//]: # (Image References)
[img1]: ./misc/original_cloud.PNG
[img2]: ./misc/statistical_filtering.PNG
[img3]: ./misc/z_passthrough_only.PNG
[img4]: ./misc/yz_passthrough.PNG
[img5]: ./misc/seg_scene_1.PNG
[img6]: ./misc/cluster_scene_1.PNG
[img7]: ./misc/model_scene_1_confusion_matrix.PNG
[img8]: ./misc/model_scene_2_confusion_matrix.PNG
[img10]: ./misc/classifi_scene_1.PNG
[img11]: ./misc/classifi_scene_2.PNG

## 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
### 1.1 Statistical Outlier Filtering
The original cloud recieved by subscribing to the topic `/pr2/world/points` is shown in `Fig. 1`. 

![alt text] [img1]
*Fig.1 The cloud publised by `/pr2/world/points`*

As can be seen in this figure, this cloud contained a lot of sparse outliers (marked by the black ellipse) which can negatively effect geometry features of the cloud such as surface normal vectors. These noisy data are removed thanks to the Statistical Outlier filter. The idea of this filter is that any point which has its mean distance to its neighbors exceeding a threshold is considered an outlier. Such threshold is defined by the global distance mean plus a standard deviation assuming the set of every point in the cloud has the Gaussian distribution.

The Statistical Outlier filter is implemented as following
```
	statiscal_filter = cloud.make_statistical_outlier_filter()  # create a filter object
    statiscal_filter.set_mean_k(10)  # set number of neighbor points involving in calculating the mean distance of a particular point
    x = 0.5  # standard deviation scale factor
    statiscal_filter.set_std_dev_mul_thresh(x)  
    cloud_filtered = statiscal_filter.filter()  # call the filter
```
In the implementation above, the number of points involving in deriving the mean distance of any single point is set to 10 which means the neighbor of an arbitrary point is defined by 10 points being closest to it. In addition, the set's standard deviation is scaled by a factor of `x` (set to 0.5) prior to being added to the global distance mean.

The point cloud of the first world outputed by this filter is displayed in `Fig.2`.

![alt text] [img2]
*Fig.2 The cloud filtered by Statistical Outlier filter*

Obviously, the sparse outliers in `Fig.1` are removed.

### 1.2 Voxel Filtering
Accessing the `size` attribute of point cloud returned by the Statistical Outlier filter, the number of data point in this cloud is 492299 which is relatively large. This number should be reduced to speed up the perception pipeline. Keep in mind that a dense point cloud is made of many data points which contain overlapping information, compared to their neighbors. Therefore, the point cloud of our interest can be made less dense by eleminating those overlapping data points. This can be done by the Voxel filter which represents all data points in a volume element (a.k.a a leaf) by a single point. The information of this point is the mean of evey point it replaces. The size of the filtered point could is controlled by the size of a volume element.

The implemenetation of this filter is
```
	voxel_filter = cloud_filtered.make_voxel_grid_filter()
    leaf_size = 0.01  # set the length of a single size of volume element
    voxel_filter.set_leaf_size(leaf_size, leaf_size, leaf_size)  # set the size of the volume element
    cloud_filtered = voxel_filter.filter()
```
As implied in the code above, the Voxel filter has one parameter which is the lenght of one size of a volume element. However, a volume element is not necessary a cubic, so this filter can take 3 parameters to define the shape of a volume element. The truly important point of this filter is the balance between the reduction of data points and the preservation point cloud key information (e.g. the overall shape and color of objects in the scene). With the cubic leaf having size of `0.01m`, the size of the cloud is reduced to 95647 (19% of the original size).

### 1.3 Pass Through Filtering
In segmatation for tablle top objects which will be carried out in the next section, everything beneat the top fo the table is unnecessary. Therefore, the data points beneat the table top can be eleminate to save the computation effort later in the segmentation. To this end, a Pass Through is carried out as following.
```
	pass_through_z = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    pass_through_z.set_filter_field_name(filter_axis)  # set the axis along which the filtering is performed
    axis_min = 0.62
    axis_max = 1.0
    pass_through_z.set_filter_limits(axis_min, axis_max)  # set the interval in which data points are allowed to pass
    cloud_filtered = pass_through_z.filter()
```
The Pass Through filter has 3 parameters, namely the filter's axis, the minimum and maximum value of the coordinate which is filtered. The result of z-axis Pass Through filter is shown in `Fig.3`. 

![alt text] [img3]
*Fig.3 Result of z-axis pass through filter*

In this figure, there are the presence of part of two dropboxes (the red and green dots at the tip of the two dropboxes) beside a thin layer of the table's top and scene objects. This dropboxes part is not the objects of interest and can not be eleminated the plane segmentation. As a result, it will cause the mal function of the clustering. For this reason, the second Pass Through which is performed along y-axis is called in to filter this dropboxes part out. Because the dropboxes' center are placed at `0.71` and `-0.71` along y-axis (these value is given by `dropbox.yaml` in folder `/pr2_robot/config`), the interval of y-axis Pass Through filter is set to `[-0.55, 0.55]`. The filtered point cloud shown in `Fig.4` now no longer contained the dropboxes part.

![alt text] [img4]
*Fig.4 Dropboxes' tip eleminated by adding an y-axis pass thorugh filter* 

### 1.4 Plane fitting
To isolate the objects on the top of the table, the table top's surface needs to be identified first. A segmentation using RANSAC method for segmenting the `plane` contained in the filtered cloud is construted below
```
    seg = cloud_filtered.make_segmenter()  # create segmentation object
    seg.set_model_type(pcl.SACMODEL_PLANE)  # set the model for segmentation
    seg.set_method_type(pcl.SAC_RANSAC)  # set segmentation method
    max_distance = 0.005  # max distance for a point to be considered in the model
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()  # call the segment func to get inlier indices
```  
The outputs of this segmentation process are the points in the model for segemtation (the plane representing the table top in this case) and the coefficient of the polinomial expressing this model. Because the outliers in this plane segmentation are what represent the table top objects, the cloud cotains only these objects are extracted from the filtered cloud by
```
ransac_objects = cloud_filtered.extract(inliers, negative=True)
```
This cloud of objects is displayed in `Fig.5`.   

![alt text] [img5]
*Fig.5 Objects segmented by RANSAC algorithm*

## 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented. 
After being filterd and segmented, the resulted cloud now contains only data points of table top objects. However, these data points currently does not represent any individual object because they are still mixed together. On the other word, those data points of the same object must be seperated from others. This is done by the Euclidean clustering algorithm. For the algorithm to be able to execute, the feature-rich objects cloud is converted to a white cloud which contains only spatial coordinates, then rearranged into a k-d tree.
```
    white_cloud = XYZRGB_to_XYZ(ransac_objects)  # create spatial point cloud
    tree = white_cloud.make_kdtree()  # k-d tree construction
```    
Next, the Euclidean algorithm's paramters are set.
```
    extracted_cluster.set_ClusterTolerance(0.025)
    extracted_cluster.set_MinClusterSize(50)
    extracted_cluster.set_MaxClusterSize(1000)
    extracted_cluster.set_SearchMethod(tree)
```
It is worth to notice that the Cluster Tolerance plays a critical role in the accuracy of the clustering process. If it is too small, large object (like the buiscuit) can be divided into many clusters. On the other hand, if the Cluster Tolerance is too big, near by objects can be mixed in one cluster. Finally, the clustering alogrithm is executed.
```
    cluster_indices = extracted_cluster.Extract()  # extract lists of points for each cluster.
```
The command above returns a list. This list itself contains a number of lists, each of which associate with a cluster. The clustering result in displayed in `Fig.6`.

![alt text] [img6]
*Fig.6 Clusters of table top objects in scene 1*

## 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
### 3.1 Compute histogram
The features of the clustered cloud taken into account by the classification process are objects' color histogram and normal direction. These two features are extracted from the cloud by `compute_color_histograms()` and `compute_normal_histograms()` which are implemented below
```
def compute_color_histograms(cloud, using_hsv=False):

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])
    
    # TODO: Compute histograms
    nbins = 64  
    channel_1_hist = np.histogram(channel_1_vals, nbins) # 1-D nbins-elements array
    channel_2_hist = np.histogram(channel_2_vals, nbins)
    channel_3_hist = np.histogram(channel_3_vals, nbins)

    # TODO: Concatenate and normalize the histograms
    color_hist = np.concatenate((channel_1_hist[0], channel_2_hist[0], channel_3_hist[0])).astype(np.float64)  # 1-D 96-elements array
    color_hist = color_hist / np.sum(color_hist)  # normalize

    return color_hist 


def compute_normal_histograms(normal_cloud):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # TODO: Compute histograms of normal values (just like with color)
    nbins = 64
    norm_x_hist = np.histogram(norm_x_vals, nbins)
    norm_y_hist = np.histogram(norm_y_vals, nbins)
    norm_z_hist = np.histogram(norm_z_vals, nbins)

    # TODO: Concatenate and normalize the histograms
    normal_hist = np.concatenate((norm_x_hist[0], norm_y_hist[0], norm_z_hist[0])).astype(np.float64)
    normal_hist = normal_hist / np.sum(normal_hist)

    return normal_hist
```
The histograms in this section are created by `numpy` library's `histogram()`. The most important parameter of this function is the number of bins (`nbins`) which is the number of intervals that the range of value of interested feature is divided into. The larger this number is, the more detail the interested feature is. However, the increase of bins number leads to the increase of the data stored in feature histogram, hence the increase of computing effort.

### 3.2 Train the SVM
Using the objects' features prepared by the previous section (the color and normal direction histogram), a Support Vector Machine is employed to classify objects on the table top based on these features. The SVM's kernel is chosen to be `linear` and the number of folds of cross validation is set to `25`. For each scene, the SVM is trained with the training set consituted by `70` samples of each object in the scene. The confusion matrix of classification model using in each scene is plotted `Fig.7-9`.      

![alt text] [img7]
*Fig.7 Confusion matrixes of Scene 1's model* 

![alt text] [img8]
*Fig.8 Confusion matrixes of Scene 2's model*

The results of objects recognition process performed by the models used in each scene are displayed in `Fig.10-12`.

![alt text] [img10]
*Fig.10 Objects recognized by Scene 1's model*

![alt text] [img11]
*Fig.11 Objects recognized by Scene 2's model*

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



