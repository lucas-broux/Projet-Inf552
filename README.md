# Projet-Inf552

Academic project as part of the [INF552 course at École Polytechnique](http://www.enseignement.polytechnique.fr/informatique/INF552/).

The goal of the project is to apply computer vision tools to the [cityscapes dataset](https://www.cityscapes-dataset.com/), which consists in stereo images from cameras mounted on a car.

We start with the left image, disparity and camera matrix.

![Left image](https://raw.githubusercontent.com/lucas-broux/Projet-Inf552/clean_code/reports/images/aachen_000029_000019_leftImg8bit.png "Left image")

![Disparity](https://raw.githubusercontent.com/lucas-broux/Projet-Inf552/clean_code/reports/images/aachen_000029_000019_disparity.png "Disparity")

From the left image, disparity, and the camera matrix, we reconstruct a 3d point cloud of the street.

![3d point cloud](https://raw.githubusercontent.com/lucas-broux/Projet-Inf552/clean_code/reports/images/3dpointcloud02.png "3d point cloud")

Using the RANSAC algorithm, we detect the main plane of the 3d point cloud i.e. the road.

![Road](https://raw.githubusercontent.com/lucas-broux/Projet-Inf552/clean_code/reports/images/Result_image_road.jpg "Road")

We then use a modified version of the RANSAC algorithm to detect vertical objects.

![Vertical objects](https://raw.githubusercontent.com/lucas-broux/Projet-Inf552/clean_code/reports/images/Result_image_vertical.jpg "Vertical objects")
