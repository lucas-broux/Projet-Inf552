CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

FIND_PACKAGE(OpenCV REQUIRED)

PROJECT(Projet-Inf552)

INCLUDE_DIRECTORIES(project/include)

SET(HEADER_FILES
	project/include/line3d.hpp
	project/include/line3dCloud.hpp
	project/include/parameters.hpp
	project/include/plane.hpp
	project/include/point3d.hpp
	project/include/point3dCloud.hpp
	project/include/product.hpp
	project/include/projectData.hpp
	project/include/ransac.hpp)

SET(SOURCE_FILES
	project/src/line3d.cpp
	project/src/line3dCloud.cpp
	project/src/plane.cpp
	project/src/point3d.cpp
	project/src/point3dCloud.cpp
	project/src/product.cpp
	project/src/projectData.cpp
	project/src/ransac.cpp)

ADD_EXECUTABLE(main project/main.cpp ${SOURCE_FILES} ${HEADER_FILES})
TARGET_LINK_LIBRARIES(main ${OpenCV_LIBS})

ADD_EXECUTABLE(performance_evaluation project/performance_evaluation.cpp ${SOURCE_FILES} ${HEADER_FILES})
TARGET_LINK_LIBRARIES(performance_evaluation ${OpenCV_LIBS})
