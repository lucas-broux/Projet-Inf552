#pragma once

const double MIN_DISPARITY = 10; //The minimum disparity to be treated in projectData::pointCloudFromData().
const double MIN_COSINE = 0.8; //The minimun cosine between the lines and the plane in ransac::fit3dLine.
