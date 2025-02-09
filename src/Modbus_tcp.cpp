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

#include "emb-lin-dev/Modbus_tcp.hpp"

#include "emb-lin-util/Stopwatch.hpp"
#include "emb-lin-util/Timespec_util.hpp"

#include <botan/base64.h>

#include <spdlog/spdlog.h>

#include <netdb.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>

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

void to_json(nlohmann::json& j, const Modbus_pdu_request_16& val)
{
	j = nlohmann::json{
		{"func_code", val.func_code},
		{"reg_start", val.reg_start},
		{"num_reg",   val.num_reg},
		{"length",    val.length},
		{"payload",   Botan::base64_encode(val.payload.data(), val.payload.size())}
	};
}
void from_json(const nlohmann::json& j, Modbus_pdu_request_16& val)
{
	j.at("func_code").get_to(val.func_code);
	j.at("reg_start").get_to(val.reg_start);
	j.at("num_reg").get_to(val.num_reg);
	j.at("length").get_to(val.length);
	val.payload = Botan::unlock(Botan::base64_decode(j.at("payload")));
}

void to_json(nlohmann::json& j, const Modbus_pdu_response_16& val)
{
	j = nlohmann::json{
		{"func_code", val.func_code},
		{"reg_start", val.reg_start},
		{"num_reg",   val.num_reg},
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
void from_json(const nlohmann::json& j, Modbus_pdu_response_16& val)
{
	j.at("func_code").get_to(val.func_code);
	j.at("reg_start").get_to(val.reg_start);
	j.at("num_reg").get_to(val.num_reg);
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

const std::chrono::milliseconds Modbus_tcp_io::MAX_READ_WAIT_TIME = std::chrono::milliseconds(2000);

Modbus_tcp_io::Modbus_tcp_io() : req_id(0)
{
	
}
Modbus_tcp_io::~Modbus_tcp_io()
{
	close();
}

bool Modbus_tcp_io::open(const std::string& server)
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
bool Modbus_tcp_io::close()
{
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
	    
	m_fd.reset();
	return true;
}

bool Modbus_tcp_io::write_modbus_frame(const std::vector<uint8_t>& buf)
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

bool Modbus_tcp_io::read_modbus_frame(Modbus_tcp_frame* const buf)
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

bool Modbus_tcp_io::send_cmd_resp(const Modbus_tcp_frame& cmd, Modbus_tcp_frame* const out_resp)
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

	if( ! write_modbus_frame(buf) )
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