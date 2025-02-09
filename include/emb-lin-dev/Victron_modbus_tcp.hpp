/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * This software is distrubuted in the hope it will be useful, but without any warranty, including the implied warrranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE.txt for details.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024-2025 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#pragma once 

#include "emb-lin-dev/Modbus_tcp.hpp"

#include <nlohmann/json.hpp>

#include <unistd.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <cstdint>

class Victron_modbus_tcp : public Modbus_tcp_io
{
public:
	Victron_modbus_tcp();
	~Victron_modbus_tcp();

	enum class CERBO_GX_UNIT_ID : uint8_t
	{
		VECAN     = 100,
		VEDIRECT3 = 223,
		VEDIRECT2 = 224,
		BMSCAN    = 225,
		VEDIRECT1 = 226,
		VEBUS     = 227
	};

	struct VictronModbusTcpRegister
	{
		std::string path;
		int address;
		RegisterType type;
		size_t num_reg;
		std::pair<int, int> scalefactor;
		bool writable;
	};

	bool read_serial(std::string* const out_serial);

	// false on error, io occured if true
	// check out_resp->is_exception if returns true
	bool read_register(const std::string& register_name, Modbus_pdu_response_03* const out_resp);
	
	// false on error, io occured if true
	// check out_resp->is_exception if returns true
	bool write_register(const std::string& register_name, const double val, Modbus_pdu_response_16* const out_resp);
	bool write_register(const Modbus_pdu_request_16& val, Modbus_pdu_response_16* const out_resp);

	static bool get_register_metadata(const std::string& dbus_name, VictronModbusTcpRegister* const out_metadata)
	{
		auto it = VICTRON_REG_MAP.find(dbus_name);
		if(it == VICTRON_REG_MAP.end())
		{
			return false;
		}

		if(out_metadata)
		{
			*out_metadata = it->second;
		}

		return true;
	}

	static size_t get_regtype_payload_length(const RegisterType& regtype)
	{
		size_t ret = 0;
		switch(regtype)
		{
			case RegisterType::INT16:
			case RegisterType::UINT16:
			{
				ret = 1;
				break;
			}
			case RegisterType::INT32:
			case RegisterType::UINT32:
			{
				ret = 2;
				break;
			}
			default:
			{
				ret = 0;
				break;
			}
		}

		return ret;
	}
protected:

	const static std::map<std::string, VictronModbusTcpRegister> VICTRON_REG_MAP;

};
