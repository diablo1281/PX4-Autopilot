px4_add_module(
	MODULE drivers__distance_sensor__vl53l8_distro
	MAIN vl53l8_distro
	SRCS
		vl53l8_distro.cpp
		vl53l8_distro.hpp
		vl53l8_distro_main.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		drivers_rangefinder
		px4_work_queue
	)
