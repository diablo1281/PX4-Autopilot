px4_add_module(
	MODULE modules__shaft_kf
	MAIN shaft_kf

	SRCS
		ShaftKF.cpp
		ShaftKFModule.cpp

	MODULE_CONFIG
		module.yaml

	DEPENDS
		mathlib
		px4_work_queue
)
