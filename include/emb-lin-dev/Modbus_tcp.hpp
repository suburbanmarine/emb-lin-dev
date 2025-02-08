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

#include <string>
#include <vector>
#include <optional>

#include <cstdint>

enum class RegisterType
{
	INT16,
	INT32,
	UINT16,
	UINT32,
	STRING
};

enum class FUNCTION_CODE : uint8_t
{
	READ_HOLDING_REGISTERS   = 0x03U,
	READ_INPUT_REGISTERS     = 0x04U,
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

class Modbus_pdu_response_base
{
public:
	uint8_t func_code;
	std::optional<uint8_t> exception_code;

	Modbus_pdu_response_base()
	{

	}
	virtual ~Modbus_pdu_response_base()
	{

	}

	// check exception bit
	bool is_exception() const
	{
		return func_code & 0x80U;
	}

	// mask off exception bit
	uint8_t base_func_code() const
	{
		return func_code & 0x7FU;
	}
};

class Modbus_pdu_request_03
{
public:
	uint8_t  func_code;
	uint16_t reg_start;
	uint16_t num_reg;

	bool serialize(std::vector<uint8_t>* const out_frame) const;
};

class Modbus_pdu_response_03 : public Modbus_pdu_response_base
{
public:
	uint8_t length;
	std::vector<uint8_t> payload;

	bool deserialize(const std::vector<uint8_t>& frame);
};

class Modbus_pdu_request_06
{
public:
	uint8_t  func_code;
	uint16_t reg_start;
	uint16_t reg_val;

	bool serialize(std::vector<uint8_t>* const out_frame) const;
};

class Modbus_pdu_response_06 : public Modbus_pdu_response_base
{
public:	
	uint16_t reg_start;
	uint16_t num_reg;

	bool deserialize(const std::vector<uint8_t>& frame);
};

class Modbus_pdu_request_16
{
public:
	uint8_t  func_code;
	uint16_t reg_start;
	uint16_t num_reg;
	uint8_t  length;
	std::vector<uint8_t> payload;

	bool serialize(std::vector<uint8_t>* const out_frame) const;
};

class Modbus_pdu_response_16 : public Modbus_pdu_response_base
{
public:
	uint16_t reg_start;
	uint16_t num_reg;

	bool deserialize(const std::vector<uint8_t>& frame);
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
	uint16_t length;
	uint8_t  unit_id;
	std::vector<uint8_t> pdu;

	bool serialize(std::vector<uint8_t>* const out_frame) const;
	bool deserialize(const std::vector<uint8_t>& frame);

	bool deserialize_header(const std::vector<uint8_t>& frame);

	bool is_frame_response_for(const Modbus_tcp_frame& other) const
	{
		return 
			(trx_id      == other.trx_id)      &&
			(protocol_id == other.protocol_id) &&
			(unit_id     == other.unit_id);
	}

	constexpr static uint16_t MBAP_HDR_LEN        = 7;
	constexpr static uint16_t MBAP_PACKET_MIN_LEN = 10;
};

void to_json(nlohmann::json& j, const Modbus_tcp_frame& val);
void from_json(const nlohmann::json& j, Modbus_tcp_frame& val);

void to_json(nlohmann::json& j, const Modbus_pdu_request_03& val);
void from_json(const nlohmann::json& j, Modbus_pdu_request_03& val);

void to_json(nlohmann::json& j, const Modbus_pdu_response_03& val);
void from_json(const nlohmann::json& j, Modbus_pdu_response_03& val);

void to_json(nlohmann::json& j, const Modbus_pdu_request_16& val);
void from_json(const nlohmann::json& j, Modbus_pdu_request_16& val);

void to_json(nlohmann::json& j, const Modbus_pdu_response_16& val);
void from_json(const nlohmann::json& j, Modbus_pdu_response_16& val);



