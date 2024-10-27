/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#pragma once 

#include <string>
#include <map>
#include <vector>

#include <cstdint>

class Victron_modbus_tcp
{
public:
	Victron_modbus_tcp();
	~Victron_modbus_tcp();

	enum class FUNCTION_CODE : uint8_t
	{
		READ_HOLDING_REGISTERS   = 0x01U,
		READ_INPUT_REGISTERS     = 0x02U,
		WRITE_SINGLE_REGISTER    = 0x03U,
		WRITE_MULTIPLE_REGISTERS = 0x0AU
	};

	enum class ERROR : uint8_t
	{
		ILLEGAL_FUNCTION                        = 0x01U,
		ILLEGAL_DATA_ADDRESS                    = 0x02U,
		ILLEGAL_DATA_VALUE                      = 0x03U,
		GATEWAY_PATH_UNAVAILABLE                = 0x0AU,
		GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0BU
	};

	enum class RegisterType
	{
		INT8,
		INT16,
		INT32,
		UINT8,
		UINT16,
		UINT32,
		STRING
	};

	struct VictronModbusTcpRegister
	{
		std::string path;
		int address;
		RegisterType type;
		std::pair<int, int> scalefactor; // Power of 10
		bool writable;
	};

	// enum class REGISTER_ID : uint8_t
	// {
	// 	Serial                 = 0,
	// 	AcPowerSetpointL1      = 37,
	// 	AcPowerSetpointL2      = 40,
	// 	AcPowerSetpointL3      = 41,
	// 	DisableCharge          = 38,
	// 	DisableFeedIn          = 39,
	// 	DoNotFeedInOvervoltage = 65,
	// 	MaxFeedInPowerL1       = 66,
	// 	MaxFeedInPowerL2       = 67,
	// 	MaxFeedInPowerL3       = 68,
	// 	TargetPowerIsMaxFeedIn = 71,
	// 	FixSolarOffsetTo100mV  = 72
	// 	ESSMode
	// };

	constexpr static uint16_t TCP_PORT = 502U;

	class Modbus_tcp_frame
	{
	public:
		Modbus_tcp_frame()
		{
			protocol_id = 0;
		}
		~Modbus_tcp_frame();

		// MODBUS Application Protocol (MBAP) header
		uint16_t trx_id; // will be returned by the server, increment it
		uint16_t protocol_id;
		// uint16_t length; // length of unit_id + func_code + reg_id + payload
		uint8_t unit_id;
		// Protocol Data Unit (PDU)
		uint8_t func_code;
		uint16_t reg_id;
		std::vector<uint8_t> payload;

		bool serialize(std::vector<uint8_t>* const out_frame);
		bool deserialize(const std::vector<uint8_t>& frame);

		constexpr static uint16_t MBAP_HDR_LEN        = 7;
		constexpr static uint16_t MBAP_PACKET_MIN_LEN = 10;
	};

	const static std::map<std::string, VictronModbusTcpRegister> VICTRON_REG_MAP;
};
