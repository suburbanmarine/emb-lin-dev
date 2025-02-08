#pragma once

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

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

class Modbus_pdu_request_03
{
public:
	uint8_t  func_code;
	uint16_t reg_start;
	uint16_t num_reg;

	bool serialize(std::vector<uint8_t>* const out_frame) const;
};

class Modbus_pdu_response_03
{
public:
	uint8_t  func_code;
	uint8_t  length;
	std::vector<uint8_t> payload;

	bool is_exception() const
	{
		return func_code & 0x80U;
	}

	uint8_t base_func_code() const
	{
		// mask off exception bit
		return func_code & 0x7FU;
	}

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

	const std::vector<uint8_t>& get_payload() const
	{
		return pdu;
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
