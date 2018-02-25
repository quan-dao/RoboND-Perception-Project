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

## 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.
### 1.1 Statistical Outlier Filtering
The original cloud recieved by subscribing to the topic `/pr2/world/points` is shown in `Fig. 1`. As can be seen in this figure, this cloud contained a lot of sparse outliers (marked by the black ellipse) which can negatively effect geometry features of the cloud such as surface normal vectors. These noisy data are removed thanks to the Statistical Outlier filter. The idea of this filter is that any point which has its mean distance to its neighbors exceeding a threshold is considered an outlier. Such threshold is defined by the global distance mean plus a standard deviation assuming the set of every point in the cloud has the Gaussian distribution.

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
In segmatation for tablle top objects, everything beneat the top fo the table is unnecessary. Therefore, the data points beneat the table top can be eleminate to save the computation effort later in the segmentation. To this end, a Pass Through is carried out as following.
```
	pass_through_z = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    pass_through_z.set_filter_field_name(filter_axis)  # set the axis along which the filtering is performed
    axis_min = 0.62
    axis_max = 1.0
    pass_through_z.set_filter_limits(axis_min, axis_max)  # set the interval in which data points are allowed to pass
    cloud_filtered = pass_through_z.filter()
```
The Pass Through filter has 3 parameters, namely the filter's axis, the minimum and maximum value of the coordinate which is filtered. The result of z-axis Pass Through filter is shown in `Fig.3`. In this figure, there are the presence of part of two dropboxes beside a thin layer of the table's top and scene objects. This dropboxes part is not the objects of interest and can not be eleminated the plane segmentation. As a result, it will cause the mal function of the clustering. For this reason, the second Pass Through which is performed along y-axis is called in to filter this dropboxes part out. Because the dropboxes' center are placed at `0.71` and `-0.71` along y-axis (these value is given by `dropbox.yaml` in folder `/pr2_robot/config`), the interval of y-axis Pass Through filter is set to `[-0.55, 0.55]`. The filtered point cloud shown in `Fig.4` now no longer contained the dropboxes part. 


## 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

## 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



