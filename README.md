# Project: 3D Perception Project
---

[//]: # (Image References)
[img1]: ./misc/original_cloud.PNG
[img2]: ./misc/statistical_filtering.PNG
[img3]: ./misc/z_passthrough_only.PNG
[img4]: ./misc/yz_passthrough.PNG
[img5]: ./misc/seg_scene_1.PNG
[img6]: ./misc/cluster_scene_1.PNG
[img7]: ./misc/model_scene_1_confusion_matrix.PNG
[img8]: ./misc/model_scene_2_confusion_matrix.PNG
[img9]: ./misc/model_scene_3_confusion_matrix_nfold_5.PNG
[img10]: ./misc/classifi_scene_1.PNG
[img11]: ./misc/classifi_scene_2.PNG
[img12]: ./misc/classifi_scene_3.PNG

This project is to develop an object detection pipeline to recognize objects placed on a table from incoming pointcloud (obtained by RGB-D camera of PR2 robot). This pipeline is made of 

* Statistical Outlier Filter

* Voxel Filter

* Passthrough Filter

* Plane detection using RANSAC

* Clustering
 
* Object detector made of Support Vector Machine (SVM)

This pipeline is implemented in the function `pcl_callback()` in the file `./pr2_robot/scripts/project_template.py`

To install this project, you can follow `Installation.md`.

## 1. Filtering and RANSAC plane fitting
### 1.1 Statistical Outlier Filtering
The original cloud recieved by subscribing to the topic `/pr2/world/points` is shown in Fig.1. 

![alt text][img1]

*Fig.1 The cloud publised by `/pr2/world/points`*

As can be seen in this figure, this cloud contained a lot of sparse outliers (marked by the black ellipse) which can negatively effect geometry features of the cloud such as surface normal vectors. These noisy data are removed thanks to the **Statistical Outlier filter**. The _idea of this filter_ is that any point which has its mean distance to its neighbors exceeding a threshold is considered an outlier. Such threshold is defined by the global distance mean plus a standard deviation assuming the set of every point in the cloud has the Gaussian distribution.

In the current implementation, the number of points involving in deriving the mean distance of any single point is set to 10 which means the neighbor of an arbitrary point is defined by 10 points being closest to it. In addition, the set's standard deviation is scaled by a factor of `x` (set to 0.5) prior to being added to the global distance mean.

The point cloud of the first world outputed by this filter is displayed in Fig.2.
![alt text][img2]

*Fig.2 The cloud filtered by Statistical Outlier filter*

Obviously, the sparse outliers in Fig.1 are removed.

### 1.2 Voxel Filtering
Accessing the `size` attribute of point cloud returned by the Statistical Outlier filter, the number of data point in this cloud is _492 299_ which is relatively large. This number should be reduced to speed up the perception pipeline. Keep in mind that a dense point cloud is made of many data points which contain overlapping information, compared to their neighbors. Therefore, the point cloud of our interest can be made less dense by eleminating those overlapping data points. This can be done by the **Voxel filter** which represents all data points in a volume element (a.k.a a leaf) by a single point. The information of this point is the mean of evey point it replaces. The size of the filtered point could is controlled by the size of a volume element.

The Voxel filter has one parameter which is the lenght of one size of a volume element. However, a volume element is not necessary a cubic, so this filter can take 3 parameters to define the shape of a volume element. The truly _important point_ of this filter is the balance between the reduction of data points and the preservation point cloud key information (e.g. the overall shape and color of objects in the scene). With the cubic leaf having size of `0.01m`, the size of the cloud is reduced to 95647 (19% of the original size).

### 1.3 Pass Through Filtering
In segmatation for table top objects which will be carried out in the next section, everything beneat the top fo the table is unnecessary. Therefore, the data points beneat the table top can be eleminate to save the computation effort later in the segmentation. To this end, a **Passthrough Filter** is used.

The Pass Through filter has 3 parameters, namely the filter's axis, the minimum and maximum value of the coordinate which is filtered. The result of z-axis Pass Through filter is shown in Fig.3. 

![alt text][img3]

*Fig.3 Result of z-axis pass through filter*

In this figure, there are the presence of part of two dropboxes (the red and green dots at the tip of the two dropboxes) beside a thin layer of the table's top and scene objects. This dropboxes part is not the objects of interest and can not be eleminated the plane segmentation. As a result, it will cause the mal function of the clustering. For this reason, the second Pass Through which is performed along y-axis is called in to filter this dropboxes part out. Because the dropboxes' center are placed at `0.71` and `-0.71` along y-axis (these value is given by `dropbox.yaml` in folder `/pr2_robot/config`), the interval of y-axis Pass Through filter is set to `[-0.55, 0.55]`. The filtered point cloud shown in Fig.4 now no longer contained the dropboxes part.

![alt text][img4]

*Fig.4 Dropboxes' tip eleminated by adding an y-axis pass thorugh filter* 

### 1.4 Plane fitting
To isolate the objects on the top of the table, the table top's surface needs to be identified first. A segmentation using **RANSAC** method for segmenting the `plane` contained in the filtered cloud is construted. 

The outputs of this segmentation process are the points in the model for segemtation (the plane representing the table top in this case) and the coefficient of the polinomial expressing this model. Because the outliers in this plane segmentation are what represent the table top objects, the cloud cotains only these objects are extracted from the filtered cloud by
```
ransac_objects = cloud_filtered.extract(inliers, negative=True)
```
This cloud of objects is displayed in Fig.5.   

![alt text][img5]

*Fig.5 Objects segmented by RANSAC algorithm*

## 2. Clustering for segmentation  
After being filterd and segmented, the resulted cloud now contains only data points of table top objects. However, these data points currently does not represent any individual object because they are still mixed together. On the other word, those data points of the same object must be seperated from others. This is done by the Euclidean clustering algorithm. For the algorithm to be able to execute, the feature-rich objects cloud is converted to a white cloud which contains only spatial coordinates, then rearranged into a k-d tree.

Next, the Euclidean algorithm's paramters are set.
```
    extracted_cluster.set_ClusterTolerance(0.025)
    extracted_cluster.set_MinClusterSize(50)
    extracted_cluster.set_MaxClusterSize(1000)
    extracted_cluster.set_SearchMethod(tree)
```
It is worth to notice that the _Cluster Tolerance_ plays a critical role in the accuracy of the clustering process. If it is too small, large object (like the buiscuit) can be divided into many clusters. On the other hand, if the Cluster Tolerance is too big, near by objects can be mixed in one cluster. 

Finally, the clustering alogrithm is executed.
```
    cluster_indices = extracted_cluster.Extract()  # extract lists of points for each cluster.
```
The command above returns a list. This list itself contains a number of lists, each of which associate with a cluster. The clustering result in displayed in Fig.6.

![alt text][img6]

*Fig.6 Clusters of table top objects in scene 1*

## 3. Features extraction and  training SVM 
### 3.1 Features extraction
The features of the clustered cloud taken into account by the classification process are **objects' color histogram** and **normal direction histogram**. These two features are extracted from the cloud by `compute_color_histograms()` and `compute_normal_histograms()` in `./pr2_robot/scripts/features.py`

The resulted histograms are created by `numpy` library's `histogram()`. The most important parameter of this function is the number of bins (`nbins`) which is the number of intervals that the range of value of interested feature is divided into. The larger this number is, the more detail the interested feature is. However, the increase of bins number leads to the increase of the data stored in feature histogram, hence the increase of computing effort.

### 3.2 Train the SVM
Using the objects' features prepared by the previous section (the color and normal direction histogram), a Support Vector Machine is employed to classify objects on the table top based on these features. The SVM's kernel is chosen to be `linear` and the number of folds of cross validation is set to `25` for the first two scenes and `5` for scene 3. For each scene, the SVM is trained with the training set consituted by `70` samples of each object in the scene. The confusion matrix of classification model using in each scene is plotted Fig.7-9.      

![alt text][img7]

*Fig.7 Confusion matrixes of Scene 1's model* 

![alt text][img8]

*Fig.8 Confusion matrixes of Scene 2's model*

![alt text][img9]

*Fig.9 Confusion matrixes of Scene 3's model*

The results of objects recognition process performed by the models used in each scene are displayed in Fig.10-12.

![alt text][img10]

*Fig.10 Objects recognized by Scene 1's model*

![alt text][img11]

*Fig.11 Objects recognized by Scene 2's model*

![alt text][img12]

*Fig.12 Objects recognized by Scene 2's model*
