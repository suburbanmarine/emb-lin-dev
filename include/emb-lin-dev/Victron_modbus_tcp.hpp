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

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <mutex>
#include <vector>

#include <cstdint>

class Victron_modbus_tcp
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

	class Socket_fd
	{
	public:
		Socket_fd(int fd) : m_fd(fd)
		{

		}
		~Socket_fd()
		{
			close();
		}

		bool close()
		{
			if(m_fd < 0)
			{
				return true;
			}

			const int ret = ::close(m_fd);
			
			m_fd = -1;

			return 0 == ret;
		}

		int get_fd() const
		{
			return m_fd;
		}

		bool is_open() const
		{
			return m_fd >= 0;
		}

	protected:
		int m_fd;
	};

	bool open(const std::string& server);
	bool close();

	bool read_serial(std::string* const out_serial);

	bool send_cmd_resp(const Modbus_tcp_frame& cmd, Modbus_tcp_frame* const out_resp);

	bool read_register(const std::string& register_name, Modbus_pdu_response_03* const out_resp);

	bool is_open() const
	{
		return m_fd && m_fd->is_open();
	}

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

protected:

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

	bool write_buf(const std::vector<uint8_t>& buf);
	bool read_modbus_frame(Modbus_tcp_frame* const buf);

	std::shared_ptr<Socket_fd> m_fd;

	std::atomic<uint16_t> req_id;

	constexpr static uint16_t TCP_PORT = 502U;
	const static std::map<std::string, VictronModbusTcpRegister> VICTRON_REG_MAP;
	const static std::chrono::milliseconds MAX_READ_WAIT_TIME;

	std::recursive_mutex m_mutex;
};
