px4_add_module(
	MODULE drivers__vyom_wdt
	MAIN vyom_wdt
	SRCS
		VyomWdt.cpp
		VyomWdt.hpp
		WdtPolicy.hpp
	MODULE_CONFIG
		module.yaml
	)

px4_add_unit_gtest(SRC WdtPolicyTest.cpp)
