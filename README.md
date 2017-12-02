# Projet-Inf552

Academic project as part of the [INF552 course at École Polytechnique](http://www.enseignement.polytechnique.fr/informatique/INF552/).

The goal of the project is to apply computer vision tools to the [cityscapes dataset](https://www.cityscapes-dataset.com/), which consists in stereo images from cameras mounted on a car.

From the left image, disparity, and the camera matrix, we reconstruct a 3d point cloud of the street.

Using the RANSAC algorithm, we detect the main plane of the 3d point cloud i.e. the road.

We then use a modified version of the RANSAC algorithm to detect vertical objects.
