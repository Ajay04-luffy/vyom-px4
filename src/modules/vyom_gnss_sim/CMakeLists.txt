px4_add_module(
	MODULE modules__vyom_gnss_sim
	MAIN vyom_gnss_sim
	SRCS
		VyomGnssSim.cpp
		VyomGnssSim.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		geo
	)
