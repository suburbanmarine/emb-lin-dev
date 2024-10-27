/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#pragma once 

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

			return 0 == ::close(m_fd);
		}

		int get_fd() const
		{
			return m_fd;
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

		bool serialize(std::vector<uint8_t>* const out_frame);
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

protected:

	bool write_buf(const std::vector<uint8_t>& buf);
	bool read_buf(const size_t payload_len, std::vector<uint8_t>* const buf);

	std::shared_ptr<Socket_fd> m_fd;

	std::atomic<uint16_t> req_id;

	constexpr static uint16_t TCP_PORT = 502U;
	const static std::map<std::string, VictronModbusTcpRegister> VICTRON_REG_MAP;
};
