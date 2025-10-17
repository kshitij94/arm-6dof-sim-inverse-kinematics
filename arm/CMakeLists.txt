cmake_minimum_required(VERSION 3.5)

project(arm LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

ament_package()


foreach(dir config meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION share/arm/${dir})
endforeach(dir)
