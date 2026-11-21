px4_add_library(vyom_gnss_guard_core
	GnssGuard.cpp
	GnssGuard.hpp
)

px4_add_module(
	MODULE modules__vyom_gnss_guard
	MAIN vyom_gnss_guard
	SRCS
		VyomGnssGuard.cpp
		VyomGnssGuard.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		vyom_gnss_guard_core
		geo
	)

px4_add_unit_gtest(SRC GnssGuardTest.cpp LINKLIBS vyom_gnss_guard_core)
