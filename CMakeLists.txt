cmake_minimum_required(VERSION 3.18)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

project(emb-lin-dev
	VERSION 0.0.1
	DESCRIPTION "Small device drivers for embedded linux userland"
	LANGUAGES C CXX
)

set (THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package (Threads REQUIRED)

find_package(Boost
	1.65
	REQUIRED
		filesystem
)

configure_file(./include/emb-lin-dev/version.hpp.in version.hpp)

add_library(emb-lin-dev
	src/BIC_2200.cpp
	src/CFA835.cpp
	src/CFA039.cpp
	src/CFA_base.cpp
	src/HEP_1000.cpp
	src/I2C_bus_base.cpp
	src/I2C_dev_base.cpp
	src/INA226.cpp
	src/LTC4151.cpp
	src/M24C02_DRE.cpp
	src/M24C64_DRE.cpp
	src/M24XXX_DRE_base.cpp
	src/MCP23017.cpp
	src/MCP23X17.cpp
	src/PCA9544A.cpp
	src/PCA9544A_bus.cpp

	src/max38615_spidev.cpp
	src/RTD_util.cpp
	src/MCP23S17.cpp

	src/gpio_base.cpp
	src/gpiod_base.cpp

	src/SPIdev.cpp
	src/SPIdev_gpiod_cs.cpp

	# TODO: make crypto optional
	src/crypto/ATECC608_botan.cpp
	src/crypto/ATECC608_iface.cpp
	src/crypto/ATECC608_TNGTLS_iface.cpp
	src/crypto/Certificate_store.cpp
)

target_link_libraries(emb-lin-dev
	Threads::Threads

	Boost::boost
	Boost::filesystem

	nlohmann_json::nlohmann_json

	spdlog::spdlog

	i2c
	gpiod

	# TODO: make crypto optional
	cryptoauth
	botan-2

	emb-lin-util
)

target_include_directories(emb-lin-dev
	PUBLIC
		include

		# TODO: make crypto optional
		/usr/include/botan-2
)

set_target_properties(emb-lin-dev
PROPERTIES
	VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}
	SOVERSION ${PROJECT_VERSION_MAJOR}
)

INSTALL(
	TARGETS
		emb-lin-dev
	DESTINATION
		lib
)
