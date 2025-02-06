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

#include <nlohmann/json.hpp>

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <cstdint>

class Victron_modbus_tcp
{
public:
	Victron_modbus_tcp();
	~Victron_modbus_tcp();

	enum class FUNCTION_CODE : uint8_t
	{
		READ_HOLDING_REGISTERS   = 0x03U,
		READ_INPUT_REGISTERS     = 0x04U, // for victron, codes 0x03 and 0x4 are the same
		WRITE_SINGLE_REGISTER    = 0x06U,
		WRITE_MULTIPLE_REGISTERS = 0x10U
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

	class Modbus_tcp_frame
	{
	public:
		Modbus_tcp_frame()
		{
			protocol_id = 0;
		}
		~Modbus_tcp_frame()
		{

		}

		// MODBUS Application Protocol (MBAP) header
		uint16_t trx_id; // will be returned by the server, increment it
		uint16_t protocol_id;
		// uint16_t length; // length of unit_id + func_code + reg_id + payload
		uint8_t unit_id;
		
		// Protocol Data Unit (PDU)
		uint8_t func_code;
		uint16_t reg_id;
		std::vector<uint8_t> payload;

		bool serialize(std::vector<uint8_t>* const out_frame) const;
		bool deserialize(const std::vector<uint8_t>& frame);

		bool is_frame_response_for(const Modbus_tcp_frame& other) const
		{
			return (trx_id == other.trx_id) && (protocol_id == other.protocol_id) && (unit_id == other.unit_id) && (reg_id == other.reg_id);
		}

		constexpr static uint16_t MBAP_HDR_LEN        = 7;
		constexpr static uint16_t MBAP_PACKET_MIN_LEN = 10;
	};

	bool open(const std::string& server);
	bool close();

	bool read_serial(std::string* const out_serial);

	bool read_register(const std::string& register_name, Modbus_tcp_frame* out_resp);

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
			case RegisterType::INT8:
			case RegisterType::UINT8:
			{
				ret = 1;
				break;
			}
			case RegisterType::INT16:
			case RegisterType::UINT16:
			{
				ret = 2;
				break;
			}
			case RegisterType::INT32:
			case RegisterType::UINT32:
			{
				ret = 4;
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
	bool read_buf(const size_t payload_len, std::vector<uint8_t>* const buf);

	std::shared_ptr<Socket_fd> m_fd;

	std::atomic<uint16_t> req_id;

	constexpr static uint16_t TCP_PORT = 502U;
	const static std::map<std::string, VictronModbusTcpRegister> VICTRON_REG_MAP;
};

void to_json(nlohmann::json& j, const Victron_modbus_tcp::Modbus_tcp_frame& val);
void from_json(const nlohmann::json& j, Victron_modbus_tcp::Modbus_tcp_frame& val);
