cmake_minimum_required(VERSION 3.5.1)
project(skratch_description)

find_package(ament_cmake REQUIRED)


install(DIRECTORY meshes gazebo launch urdf config description
	DESTINATION share/${PROJECT_NAME}/)

ament_package()
