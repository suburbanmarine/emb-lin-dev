/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#include "emb-lin-dev/CFA_base.hpp"

#include <boost/crc.hpp>

#include <spdlog/spdlog.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>


bool CFA_Packet::serialize(std::vector<uint8_t>* out_buf) const
{
	if(data.size() > 255)
	{
		SPDLOG_WARN("CFA_Packet::serialize: payload too large");
		return false;
	}

	out_buf->clear();
	out_buf->reserve(1+1+data.size()+2);
	out_buf->push_back(cmd);
	out_buf->push_back(data.size() & 0xFF);
	out_buf->insert(out_buf->end(), data.begin(), data.end());
	out_buf->push_back((crc >> 0) & 0xFFU);
	out_buf->push_back((crc >> 8) & 0xFFU);
	return true;
}
bool CFA_Packet::deserialize(const std::vector<uint8_t>& buf)
{
	if(buf.size() < 4)
	{
		SPDLOG_WARN("CFA_Packet::deserialize: buf too small");
		return false;
	}
	
	cmd        = buf[0];
	size_t len = buf[1];
	if(len != (buf.size()-4))
	{
		SPDLOG_WARN("CFA_Packet::deserialize: len mismatch");
		return false;
	}

	data.clear();
	data.reserve(len);
	data.insert(data.end(), std::next(buf.begin(), 2), std::prev(buf.end(), 2));
	
	crc = (uint16_t(buf[buf.size()-1]) << 8) | (uint16_t(buf[buf.size()-2]) << 0);

	if(calc_crc() != crc)
	{
		SPDLOG_WARN("CFA_base::wait_for_packet: crc mismatch");
		return false;
	}

	return true;
}

uint16_t CFA_Packet::calc_crc() const
{
	// crc_ccitt_true_t but with 0xFFFF initial value and output ones complement
	boost::crc_optimal<16, 0x1021, 0xFFFF, 0xFFFF, true, true> crcgen; 

	crcgen.process_byte(cmd);
	crcgen.process_byte(data.size() & 0xFF);
	crcgen.process_block(data.data(), data.data()+data.size());
	return crcgen.checksum();
}

CFA_base::CFA_base()
{
	m_fd = -1;
}
CFA_base::~CFA_base()
{
	close();
}

bool CFA_base::open(const std::string& path)
{
	errno = 0;

	int fd = ::open(path.c_str(), O_RDWR);
	if(fd < 0)
	{
		SPDLOG_ERROR("Failed to open {:s}, errno: {:d}", path, errno);
		return false;
	}
	
	m_fd = fd;

	termios options;
	memset(&options, 0, sizeof(options));

	tcgetattr(m_fd, &options);
	cfmakeraw(&options);
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	options.c_cc[VMIN]  = 0;  // return on first byte
	options.c_cc[VTIME] = 20; // 2 second timeout
	tcsetattr(m_fd, TCSANOW, &options);

	return true;
}
bool CFA_base::close()
{
	if(m_fd >= 0)
	{
		::close(m_fd);
		m_fd = -1;
	}
	return true;
}
bool CFA_base::sync()
{
	if(0 != tcflush(m_fd, TCIOFLUSH) )
	{
		return false;
	}

	CFA_Packet packet;
	while( wait_for_packet(&packet, PACKET_TIMEOUT) )
	{

	}

	return true;
}

bool CFA_base::send_ping(const std::string& msg)
{
	if(msg.size() > 124)
	{
		return false;
	}

	CFA_Packet packet;
	packet.cmd  = uint8_t(COMMON_OP_CODE::PING);
	packet.data.insert(packet.data.end(), msg.begin(), msg.end());
	packet.crc  = packet.calc_crc();
	
	if( ! send_packet(packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to send_packet");
		return false;
	}

	if( ! wait_for_packet(&packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to wait_for_packet");
		return false;
	}

	if( ! packet.is_ack_response_to(uint8_t(COMMON_OP_CODE::PING)) )
	{
		return false;
	}

	if( ! std::equal(msg.begin(), msg.end(), packet.data.begin(), packet.data.end()) )
	{
		return false;
	}

	return true;
}
bool CFA_base::get_module_info(const bool serial_nversion, std::string* const out_info)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(COMMON_OP_CODE::GET_MODULE_INFO);
	packet.data = {{uint8_t(serial_nversion)}};
	packet.crc  = packet.calc_crc();
	
	if( ! send_packet(packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to send_packet");
		return false;
	}

	if( ! wait_for_packet(&packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to wait_for_packet");
		return false;
	}

	if( ! packet.is_ack_response_to(uint8_t(COMMON_OP_CODE::GET_MODULE_INFO)) )
	{
		return false;
	}

	out_info->clear();
	out_info->insert(out_info->end(), packet.data.begin(), packet.data.end());

	return true;
}

bool CFA_base::restart_display(const RESTART_TYPE restart_type)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(COMMON_OP_CODE::RESTART);

	switch(restart_type)
	{
		case RESTART_TYPE::RELOAD_BOOT_SETTINGS:
		{
			packet.data = {{8, 18, 99}};
			break;
		}
		case RESTART_TYPE::RESTART_HOST:
		{
			packet.data = {{12, 28, 97}};
			break;
		}
		case RESTART_TYPE::POWER_OFF_HOST:
		{
			packet.data = {{3, 11, 95}};
			break;
		}
		case RESTART_TYPE::RESTART_DISPLAY:
		{
			packet.data = {{8, 25, 48}};
			break;
		}
		case RESTART_TYPE::RESTORE_DEFAULT_SETTINGS:
		{
			packet.data = {{10, 8, 98}};
			break;
		}
		case RESTART_TYPE::RESTART_DISPLAY_TO_BOOTLOADER:
		{
			packet.data = {{88, 207, 5}};
			break;
		}
		default:
		{
			return false;
		}
	}
	packet.crc  = packet.calc_crc();	

	if( ! send_packet(packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to send_packet");
		return false;
	}

	if( ! wait_for_packet(&packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to wait_for_packet");
		return false;
	}

	if( ! packet.is_ack_response_to(uint8_t(COMMON_OP_CODE::RESTART)) )
	{
		return false;
	}

	return true;
}

bool CFA_base::clear_display()
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(COMMON_OP_CODE::CLEAR_DISPLAY);
	packet.data.clear();
	packet.crc  = packet.calc_crc();
	
	if( ! send_packet(packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to send_packet");
		return false;
	}

	if( ! wait_for_packet(&packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to wait_for_packet");
		return false;
	}

	if( ! packet.is_ack_response_to(uint8_t(COMMON_OP_CODE::CLEAR_DISPLAY)) )
	{
		return false;
	}

	return true;
}

bool CFA_base::set_keypad_reporting_mask(const uint8_t press_mask, const uint8_t release_mask)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(COMMON_OP_CODE::KEYPAD_REPORTING);
	packet.data = {{press_mask, release_mask}};
	packet.crc  = packet.calc_crc();	

	if( ! send_packet(packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to send_packet");
		return false;
	}

	if( ! wait_for_packet(&packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to wait_for_packet");
		return false;
	}

	if( ! packet.is_ack_response_to(uint8_t(COMMON_OP_CODE::KEYPAD_REPORTING)) )
	{
		return false;
	}

	return true;
}

bool CFA_base::poll_keypad(uint8_t* const out_keys_down, uint8_t* const out_keys_pressed, uint8_t* const out_keys_released)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(COMMON_OP_CODE::READ_KEYPAD);
	packet.data.clear();
	packet.crc  = packet.calc_crc();
	
	if( ! send_packet(packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("CFA_base::poll_keypad: Failed to send_packet");
		return false;
	}

	if( ! wait_for_packet(&packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("CFA_base::poll_keypad: Failed to wait_for_packet");
		return false;
	}

	if( ! packet.is_ack_response_to(uint8_t(COMMON_OP_CODE::READ_KEYPAD)) )
	{
		return false;
	}

	if(packet.data.size() != 3)
	{
		SPDLOG_ERROR("CFA_base::poll_keypad: data len wrong: {:d}/{:d}", packet.cmd, packet.data.size());
		return false;
	}

	if(out_keys_down)
	{
		*out_keys_down     = packet.data[0];
	}
	if(out_keys_pressed)
	{
		*out_keys_pressed  = packet.data[1];
	}
	if(out_keys_released)
	{
		*out_keys_released = packet.data[2];
	}

	return true;
}

bool CFA_base::send_packet(const CFA_Packet& packet, const std::chrono::milliseconds& max_wait)
{
	std::vector<uint8_t> temp;

	if( ! packet.serialize(&temp) )
	{
		return false;
	}

	return send_buffer(temp, max_wait);
}

bool CFA_base::send_buffer(const std::vector<uint8_t>& buf, const std::chrono::milliseconds& max_wait)
{
	size_t num_written = 0;
	do
	{
		ssize_t ret = write(m_fd, buf.data() + num_written, buf.size() - num_written);
		if(ret < 0)
		{
			SPDLOG_WARN("CFA_base::send_buffer: errno: {:d}", errno);
			return false;
		}

		SPDLOG_DEBUG("CFA_base::send_buffer: {:d}", ret);

		num_written += size_t(ret);

	} while(num_written < buf.size());

	return num_written == buf.size();
}

bool CFA_base::wait_for_packet(CFA_Packet* out_packet, const std::chrono::milliseconds& max_wait)
{
	std::vector<uint8_t> temp;
	temp.resize(4+255);

	if( ! wait_for_read(temp.data(), 2, max_wait) ) // TODO update wait time as time elapses
	{
		return false;
	}

	if(temp[1])
	{
		if( ! wait_for_read(temp.data()+2, temp[1], max_wait) ) // TODO update wait time as time elapses
		{
			return false;
		}
	}

	if( ! wait_for_read(temp.data()+2+temp[1], 2, max_wait) ) // TODO update wait time as time elapses
	{
		return false;
	}

	temp.resize(4+temp[1]);

	if( ! out_packet->deserialize(temp) )
	{
		SPDLOG_WARN("CFA_base::wait_for_packet: deserialize failed");
		return false;
	}

	return true;

}

bool CFA_base::wait_for_read(uint8_t* out_data, size_t len, const std::chrono::milliseconds& max_wait)
{
	// TODO: update to use epoll and enforce max_wait

	size_t num_read = 0;
	do
	{
		ssize_t ret = read(m_fd, out_data + num_read, len - num_read);
		if(ret < 0)
		{
			SPDLOG_WARN("CFA_base::wait_for_read: errno: {:d}", errno);
			return false;
		}
		
		if(ret == 0)
		{
			return false;
		}

		SPDLOG_DEBUG("CFA_base::wait_for_read: {:d}", ret);

		num_read += size_t(ret);

	} while(num_read < len);

	return num_read == len;
}
