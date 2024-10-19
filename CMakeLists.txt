add_library(emb-lin-dev
	src/BIC_2200.cpp
	src/CFA835.cpp
	src/HEP_1000.cpp
	src/I2C_bus_base.cpp
	src/I2C_dev_base.cpp
	src/INA226.cpp
	src/LTC4151.cpp
	src/M24C02_DRE.cpp
	src/M24C64_DRE.cpp
	src/MCP23017.cpp
	src/MCP23X17.cpp
	src/PCA9544A.cpp
	src/PCA9544A_bus.cpp
)

target_link_libraries(emb-lin-dev
	Threads::Threads

	Boost::boost
	Boost::filesystem

	nlohmann_json::nlohmann_json

	spdlog::spdlog

	i2c
)

target_include_directories(emb-lin-dev
	PUBLIC
		include
)

INSTALL(
	TARGETS
		emb-lin-dev
	DESTINATION
		lib
)
