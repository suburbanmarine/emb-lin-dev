add_library(emb-lin-dev
	CFA835.cpp
	HEP_1000.cpp
	I2C_bus_base.cpp
	I2C_dev_base.cpp
	INA226.cpp
	LTC4151.cpp
	M24C02_DRE.cpp
	M24C64_DRE.cpp
	MCP23017.cpp
	MCP23X17.cpp
	PCA9544A.cpp
	PCA9544A_bus.cpp
)

target_link_libraries(emb-lin-dev
	Threads::Threads

	Boost::boost
	Boost::filesystem

	nlohmann_json::nlohmann_json

	spdlog::spdlog

	i2c
)

INSTALL(
	TARGETS
		emb-lin-dev
	DESTINATION
		lib
)
