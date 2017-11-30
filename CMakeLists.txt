CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

FIND_PACKAGE(OpenCV REQUIRED)

PROJECT(Projet-Inf552)

ADD_EXECUTABLE(projet projet.cpp ransac.cpp plane.cpp point3dCloud.cpp point3d.cpp line3d.cpp product.cpp line3dCloud.cpp)
TARGET_LINK_LIBRARIES(projet ${OpenCV_LIBS})
