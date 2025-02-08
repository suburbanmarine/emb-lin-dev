#include "emb-lin-dev/Modbus_tcp.hpp"

#include <botan/base64.h>

#include <spdlog/spdlog.h>
#include <fmt/ranges.h>

void to_json(nlohmann::json& j, const Modbus_tcp_frame& val)
{
	j = nlohmann::json{
		{"trx_id",      val.trx_id},
		{"protocol_id", val.protocol_id},
		{"length",      val.length},
		{"unit_id",     val.unit_id},
		{"pdu",         Botan::base64_encode(val.pdu.data(), val.pdu.size())}
	};
}
void from_json(const nlohmann::json& j, Modbus_tcp_frame& val)
{
	j.at("trx_id").get_to(val.trx_id);
	j.at("protocol_id").get_to(val.protocol_id);
	j.at("length").get_to(val.length);
	j.at("unit_id").get_to(val.unit_id);
	val.pdu = Botan::unlock(Botan::base64_decode(j.at("pdu")));
}

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Modbus_pdu_request_03, func_code, reg_start, num_reg);

void to_json(nlohmann::json& j, const Modbus_pdu_response_03& val)
{
	j = nlohmann::json{
		{"func_code",       val.func_code},
		{"length",          val.length},
		{"payload",         Botan::base64_encode(val.payload.data(), val.payload.size())},
	};

	if(val.exception_code.has_value())
	{
		j["exception_code"] = val.exception_code.value();
	}
	else
	{
		j["exception_code"] = nullptr;
	}
}
void from_json(const nlohmann::json& j, Modbus_pdu_response_03& val)
{
	j.at("func_code").get_to(val.func_code);
	j.at("length").get_to(val.length);
	if(j.at("exception_code").is_null())
	{
		val.exception_code.reset();
	}
	else
	{
		uint8_t temp;
		j.at("exception_code").get_to(temp);
		val.exception_code = temp;
	}
	val.payload = Botan::unlock(Botan::base64_decode(j.at("payload")));
}

bool Modbus_pdu_request_03::serialize(std::vector<uint8_t>* const out_frame) const
{
	out_frame->resize(5);

	(*out_frame)[0]  = func_code;

	if(func_code != uint8_t(FUNCTION_CODE::READ_HOLDING_REGISTERS))
	{
		return false;
	}

	(*out_frame)[1]  = (reg_start & 0xFF00U) >> 8;
	(*out_frame)[2]  = (reg_start & 0x00FFU) >> 0;
	(*out_frame)[3]  = (num_reg   & 0xFF00U) >> 8;
	(*out_frame)[4]  = (num_reg   & 0x00FFU) >> 0;

	return true;
}

bool Modbus_pdu_response_03::deserialize(const std::vector<uint8_t>& frame)
{
	if(frame.size() < 2)
	{
		return false;
	}

	func_code = frame[0];

	if(base_func_code() != uint8_t(FUNCTION_CODE::READ_HOLDING_REGISTERS))
	{
		return false;
	}

	if(is_exception())
	{
		exception_code = frame[1];

		length = 0;
		payload.clear();
	}
	else
	{
		exception_code.reset();

		length = frame[1];

		if(frame.size() < (2U+length))
		{
			return false;
		}

		payload.assign(frame.data()+2, frame.data()+frame.size());
	}

	return true;
}

bool Modbus_tcp_frame::serialize(std::vector<uint8_t>* const out_frame) const
{
	size_t tmplength = 1 + pdu.size();
	out_frame->resize(7 + pdu.size());

	(*out_frame)[0]  = (trx_id      & 0xFF00U) >> 8;
	(*out_frame)[1]  = (trx_id      & 0x00FFU) >> 0;
	(*out_frame)[2]  = (protocol_id & 0xFF00U) >> 8;
	(*out_frame)[3]  = (protocol_id & 0x00FFU) >> 0;
	(*out_frame)[4]  = (tmplength   & 0xFF00U) >> 8;
	(*out_frame)[5]  = (tmplength   & 0x00FFU) >> 0;
	(*out_frame)[6]  = unit_id;

	memcpy(out_frame->data() + 7, pdu.data(), pdu.size());

	return true;
}

bool Modbus_tcp_frame::deserialize_header(const std::vector<uint8_t>& frame)
{
	if(frame.size() < MBAP_HDR_LEN)
	{
		SPDLOG_ERROR("Modbus_tcp_frame::deserialize_header runt frame");
		return false;
	}

	// MODBUS Application Protocol (MBAP) header
	trx_id          = (uint16_t(frame[0]) << 8) | (uint16_t(frame[1]) << 0);
	protocol_id     = (uint16_t(frame[2]) << 8) | (uint16_t(frame[3]) << 0);
	length          = (uint16_t(frame[4]) << 8) | (uint16_t(frame[5]) << 0);
	unit_id         = frame[6];

	if(protocol_id != 0)
	{
		SPDLOG_ERROR("Modbus_tcp_frame::deserialize_header protocol mismatch");
		return false;
	}

	return true;
}

bool Modbus_tcp_frame::deserialize(const std::vector<uint8_t>& frame)
{
	if( ! deserialize_header(frame) )
	{
		return false;
	}

	if(frame.size() != (length+6U))
	{
		return false;
	}

	pdu.assign(frame.data() + 7, frame.data() + frame.size());

	return true;
}

bool Modbus_pdu_request_06::serialize(std::vector<uint8_t>* const out_frame) const
{
	out_frame->resize(5);

	(*out_frame)[0]  = func_code;

	if(func_code != uint8_t(FUNCTION_CODE::WRITE_SINGLE_REGISTER))
	{
		return false;
	}

	(*out_frame)[1]  = (reg_start & 0xFF00U) >> 8;
	(*out_frame)[2]  = (reg_start & 0x00FFU) >> 0;
	(*out_frame)[3]  = (reg_val   & 0xFF00U) >> 8;
	(*out_frame)[4]  = (reg_val   & 0x00FFU) >> 0;

	return true;
}
bool Modbus_pdu_response_06::deserialize(const std::vector<uint8_t>& frame)
{
	if(frame.size() < 2)
	{
		return false;
	}

	func_code = frame[0];

	if(base_func_code() != uint8_t(FUNCTION_CODE::WRITE_SINGLE_REGISTER))
	{
		return false;
	}

	if(is_exception())
	{
		exception_code = frame[1];

		reg_start = 0;
		num_reg   = 0;
	}
	else
	{
		if(frame.size() != 5)
		{
			return false;
		}

		exception_code.reset();
	
		reg_start = (uint16_t(frame[1]) << 8) | (uint16_t(frame[2]) << 0);
		num_reg   = (uint16_t(frame[3]) << 8) | (uint16_t(frame[4]) << 0);
	}

	return true;
}
bool Modbus_pdu_request_16::serialize(std::vector<uint8_t>* const out_frame) const
{
	if(func_code != uint8_t(FUNCTION_CODE::WRITE_MULTIPLE_REGISTERS))
	{
		return false;
	}

	if(payload.size() > 255)
	{
		return false;
	}

	if((payload.size() % 2) != 0)
	{
		return false;
	}

	if(payload.size() != (num_reg*2))
	{
		return false;
	}

	out_frame->resize(6 + payload.size());

	(*out_frame)[0]  = func_code;
	(*out_frame)[1]  = (reg_start      & 0xFF00U) >> 8;
	(*out_frame)[2]  = (reg_start      & 0x00FFU) >> 0;
	(*out_frame)[3]  = (num_reg        & 0xFF00U) >> 8;
	(*out_frame)[4]  = (num_reg        & 0x00FFU) >> 0;
	(*out_frame)[5]  = (payload.size() & 0x00FFU) >> 0;

	memcpy(out_frame->data() + 6, payload.data(), payload.size());

	return true;
}
bool Modbus_pdu_response_16::deserialize(const std::vector<uint8_t>& frame)
{
	if(frame.size() < 2)
	{
		return false;
	}

	func_code = frame[0];

	if(base_func_code() != uint8_t(FUNCTION_CODE::WRITE_MULTIPLE_REGISTERS))
	{
		return false;
	}

	if(is_exception())
	{
		exception_code = frame[1];

		reg_start = 0;
		num_reg   = 0;
	}
	else
	{
		if(frame.size() != 5)
		{
			return false;
		}

		exception_code.reset();

		reg_start = (uint16_t(frame[1]) << 8) | (uint16_t(frame[2]) << 0);
		num_reg   = (uint16_t(frame[3]) << 8) | (uint16_t(frame[4]) << 0);
	}

	return true;
}