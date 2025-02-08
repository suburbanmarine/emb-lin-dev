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

#include "emb-lin-util/Stopwatch.hpp"
#include "emb-lin-util/Timespec_util.hpp"

#include <spdlog/spdlog.h>
#include <fmt/format.h>

#include <netdb.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <exception>

#include <cstring>

// TODO: maybe just directly read the spreadsheet, or have a script dump the spreadsheet here
// TODO: add length field so we can read strings easily
// TODO: add min/max fields for bounds checking
const std::map<std::string, Victron_modbus_tcp::VictronModbusTcpRegister> Victron_modbus_tcp::VICTRON_REG_MAP = 
{
	{ "/Serial",                      {.path = "/Serial",                      .address =   800, .type = RegisterType::STRING, .num_reg = 6, .scalefactor = std::make_pair(0, 0),   .writable = false} },
	{ "/Hub4/L1/AcPowerSetpoint",     {.path = "/Hub4/L1/AcPowerSetpoint",     .address =    37, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/L2/AcPowerSetpoint",     {.path = "/Hub4/L2/AcPowerSetpoint",     .address =    40, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/L3/AcPowerSetpoint",     {.path = "/Hub4/L3/AcPowerSetpoint",     .address =    41, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/DisableCharge",          {.path = "/Hub4/DisableCharge",          .address =    38, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/DisableFeedIn",          {.path = "/Hub4/DisableFeedIn",          .address =    39, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/DoNotFeedInOvervoltage", {.path = "/Hub4/DoNotFeedInOvervoltage", .address =    65, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/L1/MaxFeedInPower",      {.path = "/Hub4/L1/MaxFeedInPower",      .address =    66, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 100), .writable = true} },
	{ "/Hub4/L2/MaxFeedInPower",      {.path = "/Hub4/L2/MaxFeedInPower",      .address =    67, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 100), .writable = true} },
	{ "/Hub4/L3/MaxFeedInPower",      {.path = "/Hub4/L3/MaxFeedInPower",      .address =    68, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 100), .writable = true} },
	{ "/Hub4/TargetPowerIsMaxFeedIn", {.path = "/Hub4/TargetPowerIsMaxFeedIn", .address =    71, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/FixSolarOffsetTo100mV",  {.path = "/Hub4/FixSolarOffsetTo100mV",  .address =    72, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Settings/Ess/Mode",           {.path = "/Settings/Ess/Mode",           .address =  4921, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Ess/AcPowerSetpoint",         {.path = "/Ess/AcPowerSetpoint",         .address =  4922, .type = RegisterType::INT32,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Ess/DisableFeedIn",           {.path = "/Ess/DisableFeedIn",           .address =  4924, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	
	{ "/Dc/Battery/Voltage",          {.path = "/Dc/Battery/Voltage",          .address =   840, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1),  .writable = false} },
	{ "/Dc/Battery/Current",          {.path = "/Dc/Battery/Current",          .address =   841, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1),  .writable = false} },
	{ "/Dc/Battery/Power",            {.path = "/Dc/Battery/Power",            .address =   842, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = false} },
	{ "/Dc/Battery/Soc",              {.path = "/Dc/Battery/Soc",              .address =   843, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = false} },
	{ "/Dc/Battery/State",            {.path = "/Dc/Battery/State",            .address =   844, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = false} },
	{ "/Dc/Battery/ConsumedAmphours", {.path = "/Dc/Battery/ConsumedAmphours", .address =   845, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, -1), .writable = false} },
	{ "/Dc/Battery/TimeToGo",         {.path = "/Dc/Battery/TimeToGo",         .address =   846, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 100), .writable = false} },

	{ "/Ac/ActiveIn/L1/V",            {.path = "/Ac/ActiveIn/L1/V",            .address =     3, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L2/V",            {.path = "/Ac/ActiveIn/L2/V",            .address =     4, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L3/V",            {.path = "/Ac/ActiveIn/L3/V",            .address =     5, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L1/I",            {.path = "/Ac/ActiveIn/L1/I",            .address =     6, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L2/I",            {.path = "/Ac/ActiveIn/L2/I",            .address =     7, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L3/I",            {.path = "/Ac/ActiveIn/L3/I",            .address =     8, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L1/F",            {.path = "/Ac/ActiveIn/L1/F",            .address =     9, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/L2/F",            {.path = "/Ac/ActiveIn/L2/F",            .address =    10, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/L3/F",            {.path = "/Ac/ActiveIn/L3/F",            .address =    11, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/L1/P",            {.path = "/Ac/ActiveIn/L1/P",            .address =    12, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/ActiveIn/L2/P",            {.path = "/Ac/ActiveIn/L2/P",            .address =    13, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/ActiveIn/L3/P",            {.path = "/Ac/ActiveIn/L3/P",            .address =    14, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },

	{ "/Ac/Out/L1/V",                 {.path = "/Ac/Out/L1/V",                 .address =    15, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L2/V",                 {.path = "/Ac/Out/L2/V",                 .address =    16, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L3/V",                 {.path = "/Ac/Out/L3/V",                 .address =    17, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L1/I",                 {.path = "/Ac/Out/L1/I",                 .address =    18, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L2/I",                 {.path = "/Ac/Out/L2/I",                 .address =    19, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L3/I",                 {.path = "/Ac/Out/L3/I",                 .address =    20, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable = false} },
	{ "/Ac/Out/L1/F",                 {.path = "/Ac/Out/L1/F",                 .address =    21, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/CurrentLimit",    {.path = "/Ac/ActiveIn/CurrentLimit",    .address =    22, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable = false} },
	{ "/Ac/Out/L1/P",                 {.path = "/Ac/Out/L1/P",                 .address =    23, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/Out/L2/P",                 {.path = "/Ac/Out/L2/P",                 .address =    24, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/Out/L3/P",                 {.path = "/Ac/Out/L3/P",                 .address =    25, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} }
};

const std::chrono::milliseconds Victron_modbus_tcp::MAX_READ_WAIT_TIME = std::chrono::milliseconds(2000);


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

	if( ! out_resp->deserialize(resp_frame.pdu) )
	{
		return false;
	}

	if(pdu.func_code != out_resp->base_func_code())
	{
		return false;
	}

	if(out_resp->is_exception())
	{
		//TODO handle exceptions
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

bool Victron_modbus_tcp::read_modbus_frame(Modbus_tcp_frame* const buf)
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);

	if( ! is_open() )
	{
		return false;
	}

	// TODO timeouts

	pollfd read_fds[] = {
		{.fd = m_fd->get_fd(), .events = POLLIN}
	};

	Stopwatch stopwatch;
	if( ! stopwatch.start() )
	{
		return false;
	}

	sigset_t sigmask;
	sigemptyset(&sigmask);

	const ssize_t HDR_LEN = 7;
	std::vector<uint8_t> header_buf;
	header_buf.resize(HDR_LEN);

	{
		ssize_t num_read = 0;
		do
		{
			std::chrono::nanoseconds t_now;
			if( ! stopwatch.get_time(&t_now) )
			{
				return false;
			}
			timespec max_wait_ts = Timespec_util::from_chrono(MAX_READ_WAIT_TIME - t_now);

			int ppoll_ret = ppoll(read_fds, sizeof(read_fds) / sizeof(pollfd), &max_wait_ts, &sigmask);
			if(ppoll_ret < 0)
			{
				// TODO: log errno
				m_fd.reset();
				return false;
			}
			else if(ppoll_ret == 0)
			{
				// no data
				return false;
			}

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
			std::chrono::nanoseconds t_now;
			if( ! stopwatch.get_time(&t_now) )
			{
				return false;
			}
			timespec max_wait_ts = Timespec_util::from_chrono(MAX_READ_WAIT_TIME - t_now);

			int ppoll_ret = ppoll(read_fds, sizeof(read_fds) / sizeof(pollfd), &max_wait_ts, &sigmask);
			if(ppoll_ret < 0)
			{
				// TODO: log errno
				m_fd.reset();
				return false;
			}
			else if(ppoll_ret == 0)
			{
				// no data
				return false;
			}

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