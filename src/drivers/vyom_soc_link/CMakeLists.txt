px4_add_library(vyom_soc_link_core
	SocLinkCore.cpp
	SocLinkCore.hpp
	proto/vyom_link_v0.h
)

px4_add_module(
	MODULE drivers__vyom_soc_link
	MAIN vyom_soc_link
	SRCS
		VyomSocLink.cpp
		VyomSocLink.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		vyom_soc_link_core
		timesync
	)

px4_add_unit_gtest(SRC SocLinkCoreTest.cpp LINKLIBS vyom_soc_link_core)
