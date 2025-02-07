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

#include "emb-lin-dev/Victron_modbus_tcp.hpp"

#include <botan/base64.h>

#include <spdlog/spdlog.h>
#include <fmt/format.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#include <exception>

#include <cstring>

// TODO: maybe just directly read the spreadsheet, or have a script dump the spreadsheet here
// TODO: add length field so we can read strings easily
const std::map<std::string, Victron_modbus_tcp::VictronModbusTcpRegister> Victron_modbus_tcp::VICTRON_REG_MAP = 
{
	{ "/Serial",                      {.path = "/Serial",                      .address =   800, .type = Victron_modbus_tcp::RegisterType::STRING, .num_reg = 6, .scalefactor = std::make_pair(0, 0),   .writable = false} },
	{ "/Hub4/L1/AcPowerSetpoint",     {.path = "/Hub4/L1/AcPowerSetpoint",     .address =    37, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/L2/AcPowerSetpoint",     {.path = "/Hub4/L2/AcPowerSetpoint",     .address =    40, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/L3/AcPowerSetpoint",     {.path = "/Hub4/L3/AcPowerSetpoint",     .address =    41, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/DisableCharge",          {.path = "/Hub4/DisableCharge",          .address =    38, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/DisableFeedIn",          {.path = "/Hub4/DisableFeedIn",          .address =    39, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/DoNotFeedInOvervoltage", {.path = "/Hub4/DoNotFeedInOvervoltage", .address =    65, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/L1/MaxFeedInPower",      {.path = "/Hub4/L1/MaxFeedInPower",      .address =    66, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 100), .writable = true} },
	{ "/Hub4/L2/MaxFeedInPower",      {.path = "/Hub4/L2/MaxFeedInPower",      .address =    67, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 100), .writable = true} },
	{ "/Hub4/L3/MaxFeedInPower",      {.path = "/Hub4/L3/MaxFeedInPower",      .address =    68, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 100), .writable = true} },
	{ "/Hub4/TargetPowerIsMaxFeedIn", {.path = "/Hub4/TargetPowerIsMaxFeedIn", .address =    71, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/FixSolarOffsetTo100mV",  {.path = "/Hub4/FixSolarOffsetTo100mV",  .address =    72, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Settings/Ess/Mode",           {.path = "/Settings/Ess/Mode",           .address =  4921, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Ess/AcPowerSetpoint",         {.path = "/Ess/AcPowerSetpoint",         .address =  4922, .type = Victron_modbus_tcp::RegisterType::INT32,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Ess/DisableFeedIn",           {.path = "/Ess/DisableFeedIn",           .address =  4924, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	
	{ "/Dc/Battery/Voltage",          {.path = "/Dc/Battery/Voltage",          .address =   840, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1),  .writable = false} },
	{ "/Dc/Battery/Current",          {.path = "/Dc/Battery/Current",          .address =   841, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1),  .writable = false} },
	{ "/Dc/Battery/Power",            {.path = "/Dc/Battery/Power",            .address =   842, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = false} },
	{ "/Dc/Battery/Soc",              {.path = "/Dc/Battery/Soc",              .address =   843, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = false} },
	{ "/Dc/Battery/State",            {.path = "/Dc/Battery/State",            .address =   844, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = false} },
	{ "/Dc/Battery/ConsumedAmphours", {.path = "/Dc/Battery/ConsumedAmphours", .address =   845, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, -1), .writable = false} },
	{ "/Dc/Battery/TimeToGo",         {.path = "/Dc/Battery/TimeToGo",         .address =   846, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 100), .writable = false} },

	{ "/Ac/ActiveIn/L1/V",            {.path = "/Ac/ActiveIn/L1/V",            .address =     3, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L2/V",            {.path = "/Ac/ActiveIn/L2/V",            .address =     4, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L3/V",            {.path = "/Ac/ActiveIn/L3/V",            .address =     5, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L1/I",            {.path = "/Ac/ActiveIn/L1/I",            .address =     6, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L2/I",            {.path = "/Ac/ActiveIn/L2/I",            .address =     7, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L3/I",            {.path = "/Ac/ActiveIn/L3/I",            .address =     8, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L1/F",            {.path = "/Ac/ActiveIn/L1/F",            .address =     9, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/L2/F",            {.path = "/Ac/ActiveIn/L2/F",            .address =    10, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/L3/F",            {.path = "/Ac/ActiveIn/L3/F",            .address =    11, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/L1/P",            {.path = "/Ac/ActiveIn/L1/P",            .address =    12, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/ActiveIn/L2/P",            {.path = "/Ac/ActiveIn/L2/P",            .address =    13, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/ActiveIn/L3/P",            {.path = "/Ac/ActiveIn/L3/P",            .address =    14, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },

	{ "/Ac/Out/L1/V",                 {.path = "/Ac/Out/L1/V",                 .address =    15, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L2/V",                 {.path = "/Ac/Out/L2/V",                 .address =    16, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L3/V",                 {.path = "/Ac/Out/L3/V",                 .address =    17, .type = Victron_modbus_tcp::RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L1/I",                 {.path = "/Ac/Out/L1/I",                 .address =    18, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L2/I",                 {.path = "/Ac/Out/L2/I",                 .address =    19, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L3/I",                 {.path = "/Ac/Out/L3/I",                 .address =    20, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable = false} },
	{ "/Ac/Out/L1/F",                 {.path = "/Ac/Out/L1/F",                 .address =    21, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/CurrentLimit",    {.path = "/Ac/ActiveIn/CurrentLimit",    .address =    22, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable = false} },
	{ "/Ac/Out/L1/P",                 {.path = "/Ac/Out/L1/P",                 .address =    23, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/Out/L2/P",                 {.path = "/Ac/Out/L2/P",                 .address =    24, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/Out/L3/P",                 {.path = "/Ac/Out/L3/P",                 .address =    25, .type = Victron_modbus_tcp::RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} }
};

void to_json(nlohmann::json& j, const Victron_modbus_tcp::Modbus_tcp_frame& val)
{
	j = nlohmann::json{
		{"trx_id",      val.trx_id},
		{"protocol_id", val.protocol_id},
		{"length",      val.length},
		{"unit_id",     val.unit_id},
		{"pdu",         Botan::base64_encode(val.pdu.data(), val.pdu.size())}
	};
}
void from_json(const nlohmann::json& j, Victron_modbus_tcp::Modbus_tcp_frame& val)
{
	j.at("trx_id").get_to(val.trx_id);
	j.at("protocol_id").get_to(val.protocol_id);
	j.at("length").get_to(val.length);
	j.at("unit_id").get_to(val.unit_id);
	val.pdu = Botan::unlock(Botan::base64_decode(j.at("pdu")));
}

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Victron_modbus_tcp::Modbus_pdu_request_03, func_code, reg_start, num_reg);

void to_json(nlohmann::json& j, const Victron_modbus_tcp::Modbus_pdu_response_03& val)
{
	j = nlohmann::json{
		{"func_code", val.func_code},
		{"length",  val.length},
		{"payload",   Botan::base64_encode(val.payload.data(), val.payload.size())}
	};
}
void from_json(const nlohmann::json& j, Victron_modbus_tcp::Modbus_pdu_response_03& val)
{
	j.at("func_code").get_to(val.func_code);
	j.at("length").get_to(val.length);
	val.payload = Botan::unlock(Botan::base64_decode(j.at("payload")));
}

bool Victron_modbus_tcp::Modbus_pdu_request_03::serialize(std::vector<uint8_t>* const out_frame) const
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

bool Victron_modbus_tcp::Modbus_pdu_response_03::deserialize(const std::vector<uint8_t>& frame)
{
	if(frame.size() < 2)
	{
		return false;
	}

	func_code = frame[0];

	if(func_code != uint8_t(FUNCTION_CODE::READ_HOLDING_REGISTERS))
	{
		return false;
	}

	length = frame[1];

	if(frame.size() < (2U+length))
	{
		return false;
	}

	payload.assign(frame.data()+2, frame.data()+frame.size());

	return true;
}

bool Victron_modbus_tcp::Modbus_tcp_frame::serialize(std::vector<uint8_t>* const out_frame) const
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

bool Victron_modbus_tcp::Modbus_tcp_frame::deserialize_header(const std::vector<uint8_t>& frame)
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

bool Victron_modbus_tcp::Modbus_tcp_frame::deserialize(const std::vector<uint8_t>& frame)
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

Victron_modbus_tcp::Victron_modbus_tcp() : req_id(0)
{
	
}
Victron_modbus_tcp::~Victron_modbus_tcp()
{
	close();
}

bool Victron_modbus_tcp::open(const std::string& server)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);

	m_fd.reset();

	addrinfo* getaddrinfo_result;

	addrinfo hints;
	memset(&hints, 0, sizeof(hints));
	hints.ai_flags    = 0;
	hints.ai_family   = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	int ret = getaddrinfo(server.c_str(), fmt::format("{:d}", TCP_PORT).c_str(), &hints, &getaddrinfo_result);
	if(ret < 0)
	{
		return false;
	}

	for(addrinfo* a = getaddrinfo_result; a != nullptr; a = a->ai_next)
	{
		// m_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		// if(m_fd < 0)
		// {
		// 	return false;
		// }
		int temp_fd = socket(a->ai_family, a->ai_socktype, a->ai_protocol);
		if(m_fd < 0)
		{
			continue;
		}

		std::shared_ptr<Socket_fd> sock = std::make_shared<Socket_fd>(temp_fd);
		ret = connect(temp_fd, a->ai_addr, a->ai_addrlen);
		if(ret < 0)
		{
			continue;
		}
		else
		{
			m_fd = sock;
			break;
		}
	}

	return m_fd != nullptr;
}
bool Victron_modbus_tcp::close()
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
	    
	m_fd.reset();
	return true;
}

bool Victron_modbus_tcp::read_serial(std::string* const out_serial)
{
	Modbus_pdu_response_03 resp_pdu;
	if( ! read_register("/Serial", &resp_pdu) )
	{
		return false;
	}

	if(out_serial)
	{
		out_serial->clear();
		out_serial->insert(out_serial->end(), resp_pdu.payload.begin(), resp_pdu.payload.end());
	}

	return true;
}

bool Victron_modbus_tcp::send_cmd_resp(const Modbus_tcp_frame& cmd, Modbus_tcp_frame* const out_resp)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);

	if( ! out_resp )
	{
		return false;
	}

	std::vector<uint8_t> buf;
	if( ! cmd.serialize(&buf) )
	{
		return false;
	}

	if( ! write_buf(buf) )
	{
		return false;
	}

	if( ! read_modbus_frame(out_resp) )
	{
		SPDLOG_ERROR("send_cmd_resp read failed");
		return false;
	}

	if( ! out_resp->is_frame_response_for(cmd) )
	{
		SPDLOG_ERROR("send_cmd_resp response mismatch");
		return false;
	}

	return true;
}

bool Victron_modbus_tcp::read_register(const std::string& register_name, Modbus_pdu_response_03* const out_resp)
{
	auto reg_info = VICTRON_REG_MAP.find(register_name);
	if(reg_info == VICTRON_REG_MAP.end())
	{
		// add metadata about register to VICTRON_REG_MAP
		return false;
	}

	size_t num_reg = reg_info->second.num_reg;
	if( num_reg == 0)
	{
		num_reg = get_regtype_payload_length(reg_info->second.type);
	}

	if(num_reg == 0)
	{
		// unsupported by this api, eg a unknown length string
		return false;
	}

	Modbus_pdu_request_03 pdu;
	pdu.func_code = uint8_t(FUNCTION_CODE::READ_HOLDING_REGISTERS);
	pdu.reg_start = reg_info->second.address;
	pdu.num_reg   = num_reg;

	Modbus_tcp_frame cmd_frame;
	cmd_frame.trx_id  = req_id++;
	cmd_frame.unit_id = uint8_t(CERBO_GX_UNIT_ID::VECAN);
	if( ! pdu.serialize(&cmd_frame.pdu) )
	{
		return false;
	}

	Modbus_tcp_frame resp_frame;
	if( ! send_cmd_resp(cmd_frame, &resp_frame) )
	{
		return false;
	}

	Modbus_pdu_response_03 resp_pdu;
	if( ! out_resp->deserialize(resp_frame.pdu) )
	{
		return false;
	}

	return true;
}

bool Victron_modbus_tcp::write_buf(const std::vector<uint8_t>& buf)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);

	if( ! is_open() )
	{
		return false;
	}

	size_t num_written = 0;
	do
	{
		const ssize_t num_to_write = buf.size() - num_written;
		const ssize_t ret = write(m_fd->get_fd(), buf.data() + num_written, num_to_write);
		if( ret < 0 )
		{
			m_fd.reset();
			return false;
		}

		num_written += ret;
	} while(num_written != buf.size());
	
	return true;
}

bool Victron_modbus_tcp::read_modbus_frame(Victron_modbus_tcp::Modbus_tcp_frame* const buf)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);

	if( ! is_open() )
	{
		return false;
	}

	// TODO timeouts

	const ssize_t HDR_LEN = 7;
	std::vector<uint8_t> header_buf;
	header_buf.resize(HDR_LEN);

	{
		ssize_t num_read = 0;
		do
		{
			const ssize_t num_to_read = HDR_LEN - num_read;
			const ssize_t ret = read(m_fd->get_fd(), header_buf.data() + num_read, num_to_read);
			if( ret < 0 )
			{
				m_fd.reset();
				return false;
			}

			num_read += ret;
		} while(num_read != HDR_LEN);
	}

	if( ! buf->deserialize_header(header_buf) )
	{
		SPDLOG_ERROR("deserialize_header failed");
		return false;
	}

	buf->pdu.resize(buf->length - 1);
	{
		ssize_t num_read = 0;
		do
		{
			const ssize_t num_to_read = buf->pdu.size() - num_read;
			const ssize_t ret = read(m_fd->get_fd(), buf->pdu.data() + num_read, num_to_read);
			if( ret < 0 )
			{
				m_fd.reset();
				return false;
			}

			num_read += ret;
		} while(size_t(num_read) != buf->pdu.size());	
	}

	return true;
}