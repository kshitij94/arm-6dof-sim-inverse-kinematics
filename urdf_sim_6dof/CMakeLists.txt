cmake_minimum_required(VERSION 3.5)

project(urdf_sim_6dof LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

ament_package()


foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION share/urdf_sim_6dof/${dir})
endforeach(dir)
