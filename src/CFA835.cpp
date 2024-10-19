/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
*/

#include "emb-lin-dev/CFA835.hpp"

#include <boost/crc.hpp>

#include <spdlog/spdlog.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <cstring>

CFA835::CFA835()
{
	m_fd = -1;
}

CFA835::~CFA835()
{
	close();
}

bool CFA835::CFA835_Packet::serialize(std::vector<uint8_t>* out_buf) const
{
	if(data.size() > 255)
	{
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
bool CFA835::CFA835_Packet::deserialize(const std::vector<uint8_t>& buf)
{
	if(buf.size() < 4)
	{
		SPDLOG_WARN("CFA835::wait_for_packet: buf too small");
		return false;
	}
	
	cmd        = buf[0];
	size_t len = buf[1];
	if(len != (buf.size()-4))
	{
		SPDLOG_WARN("CFA835::wait_for_packet: len mismatch");
		return false;
	}

	data.clear();
	data.reserve(len);
	data.insert(data.end(), std::next(buf.begin(), 2), std::prev(buf.end(), 2));
	
	crc = (uint16_t(buf[buf.size()-1]) << 8) | (uint16_t(buf[buf.size()-2]) << 0);

	if(calc_crc() != crc)
	{
		SPDLOG_WARN("CFA835::wait_for_packet: crc mismatch");
		return false;
	}

	return true;
}

// uint16_t CFA835::CFA835_Packet::calc_crc() const
// {
// 	// crc_ccitt_true_t but with 0xFFFF initial value and output ones complement
// 	boost::crc_optimal<16, 0x1021, 0xFFFF, 0x0000, true, true> crcgen; 

// 	crcgen.process_byte(cmd);
// 	crcgen.process_byte(data.size() & 0xFF);
// 	crcgen.process_block(data.data(), data.data()+data.size());
// 	return ~crcgen.checksum();
// }

uint16_t CFA835::CFA835_Packet::calc_crc() const
{
	// crc_ccitt_true_t but with 0xFFFF initial value and output ones complement
	boost::crc_optimal<16, 0x1021, 0xFFFF, 0xFFFF, true, true> crcgen; 

	crcgen.process_byte(cmd);
	crcgen.process_byte(data.size() & 0xFF);
	crcgen.process_block(data.data(), data.data()+data.size());
	return crcgen.checksum();
}

bool CFA835::open(const std::string& path)
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
bool CFA835::close()
{
	if(m_fd >= 0)
	{
		::close(m_fd);
		m_fd = -1;
	}
	return true;
}
bool CFA835::sync()
{
	if(0 != tcflush(m_fd, TCIOFLUSH) )
	{
		return false;
	}

	CFA835::CFA835_Packet packet;
	while( wait_for_packet(&packet, PACKET_TIMEOUT) )
	{

	}

	return true;
}

bool CFA835::send_ping(const std::string& msg)
{
	if(msg.size() > 124)
	{
		return false;
	}

	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::PING);
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

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::PING)) )
	{
		return false;
	}

	if( ! std::equal(msg.begin(), msg.end(), packet.data.begin(), packet.data.end()) )
	{
		return false;
	}

	return true;
}
bool CFA835::get_module_info(const bool serial_nversion, std::string* const out_info)
{
	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::GET_MODULE_INFO);
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

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::GET_MODULE_INFO)) )
	{
		return false;
	}

	out_info->clear();
	out_info->insert(out_info->end(), packet.data.begin(), packet.data.end());

	return true;
}

bool CFA835::set_brightness(const uint8_t display_percent, const uint8_t keypad_percent)
{
	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::DISPLAY_KEYPAD_BACKLIGHT);
	packet.data = {{display_percent, keypad_percent}};
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

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::DISPLAY_KEYPAD_BACKLIGHT)) )
	{
		return false;
	}

	return true;
}

bool CFA835::clear_display()
{
	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::CLEAR_DISPLAY);
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

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::CLEAR_DISPLAY)) )
	{
		return false;
	}

	return true;
}

bool CFA835::set_keypad_reporting_mask(const uint8_t press_mask, const uint8_t release_mask)
{
	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::KEYPAD_REPORTING);
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

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::KEYPAD_REPORTING)) )
	{
		return false;
	}

	return true;
}

bool CFA835::poll_keypad(uint8_t* const out_keys_down, uint8_t* const out_keys_pressed, uint8_t* const out_keys_released)
{
	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::READ_KEYPAD);
	packet.data.clear();
	packet.crc  = packet.calc_crc();
	
	if( ! send_packet(packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("CFA835::poll_keypad: Failed to send_packet");
		return false;
	}

	if( ! wait_for_packet(&packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("CFA835::poll_keypad: Failed to wait_for_packet");
		return false;
	}

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::READ_KEYPAD)) )
	{
		return false;
	}

	if(packet.data.size() != 3)
	{
		SPDLOG_ERROR("CFA835::poll_keypad: data len wrong: {:d}/{:d}", packet.cmd, packet.data.size());
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

bool CFA835::write_text(const int col, const int row, const std::string& str)
{
	if(col > 19)
	{
		return false;
	}
	if(row > 3)
	{
		return false;
	}
	if(str.size() > 20)
	{
		return false;
	}

	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::WRITE_TEXT);
	packet.data.push_back(col & 0xFF);
	packet.data.push_back(row & 0xFF);
	packet.data.insert(packet.data.end(), str.begin(), str.end());
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

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::WRITE_TEXT)) )
	{
		return false;
	}

	return true;
}

bool CFA835::set_cursor_position(const uint8_t col, const uint8_t row)
{
	if(col > 19)
	{
		return false;
	}
	if(row > 3)
	{
		return false;
	}

	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::DISPLAY_CURSOR_POSITION);
	packet.data = {{col, row}};
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

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::DISPLAY_CURSOR_POSITION)) )
	{
		return false;
	}

	return true;
}
bool CFA835::set_cursor_style(const CURSOR_STYLE style)
{
	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::CURSOR_STYLE);
	packet.data = {{uint8_t(style)}};
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

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::CURSOR_STYLE)) )
	{
		return false;
	}

	return true;
}

bool CFA835::get_cfa_fbscab_count(uint8_t* out_count)
{
	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::FBSCAB_CMD);
	packet.data = {{uint8_t(CFA835::FBSCAB_CMD_CODE::GET_MODULE_INFO)}};
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

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::FBSCAB_CMD), uint8_t(CFA835::FBSCAB_CMD_CODE::GET_MODULE_INFO)) )
	{
		return false;
	}

	if(out_count)
	{
		*out_count = packet.data[1];
	}

	return true;	
}
bool CFA835::get_cfa_fbscab_info(const uint8_t fbscab_idx, std::string* const out_info)
{
	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::FBSCAB_CMD);
	packet.data = {{uint8_t(CFA835::FBSCAB_CMD_CODE::GET_MODULE_INFO), fbscab_idx}};
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

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::FBSCAB_CMD), uint8_t(CFA835::FBSCAB_CMD_CODE::GET_MODULE_INFO)) )
	{
		return false;
	}

	if( packet.data[1] != fbscab_idx )
	{
		return false;
	}

	if(out_info)
	{
		out_info->clear();
		out_info->insert(out_info->end(), std::next(packet.data.begin(), 2), packet.data.end());
	}

	return true;
}

bool CFA835::get_cfa_fbscab_dow_info(const uint8_t fbscab_idx, std::vector<std::string>* const out_info)
{
	if( ! out_info )
	{
		return false;
	}
	out_info->clear();

	std::string dow_info;
	dow_info.reserve(16);

	for(uint8_t dow_idx = 0; dow_idx < 16; dow_idx++)
	{
		CFA835::CFA835_Packet packet;
		packet.cmd  = uint8_t(CFA835::OP_CODE::FBSCAB_CMD);
		packet.data = {{uint8_t(CFA835::FBSCAB_CMD_CODE::READ_DOW_INFO), fbscab_idx, dow_idx}};
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

		if( packet.is_error_response_to(uint8_t(CFA835::OP_CODE::FBSCAB_CMD), uint8_t(CFA835::FBSCAB_CMD_CODE::READ_DOW_INFO)) )
		{
			return false;
		}

		if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::FBSCAB_CMD), uint8_t(CFA835::FBSCAB_CMD_CODE::READ_DOW_INFO)) )
		{
			return false;
		}

		if(packet.data.size() != 11)
		{
			return false;
		}

		if( packet.data[1] != fbscab_idx )
		{
			return false;
		}

		if( packet.data[2] != dow_idx )
		{
			return false;
		}

		// All 0x00 sn is no sensor
		if( std::all_of(std::next(packet.data.begin(), 3), packet.data.end(), [](const auto& x) {return x == 0x00U;}) )
		{
			continue;
		}

		dow_info.clear();
		for(auto it = std::next(packet.data.begin(), 3); it != packet.data.end(); it++)
		{
			dow_info += fmt::format("{:02X}", *it);
		}
		out_info->push_back(dow_info);
	}
	return true;
}

bool CFA835::get_cfa_fbscab_dow_temp(const uint8_t fbscab_idx, const uint8_t dow_idx, uint16_t* out_temp)
{
	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::FBSCAB_CMD);
	packet.data = {{uint8_t(CFA835::FBSCAB_CMD_CODE::READ_DOW_TEMP), fbscab_idx, dow_idx}};
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

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::FBSCAB_CMD), uint8_t(CFA835::FBSCAB_CMD_CODE::READ_DOW_TEMP)) )
	{
		return false;
	}

	if(packet.data.size() != 5)
	{
		return false;
	}

	if( packet.data[1] != fbscab_idx )
	{
		return false;
	}

	if( packet.data[2] != dow_idx )
	{
		return false;
	}

	if(out_temp)
	{
		*out_temp = (uint16_t(packet.data[4]) << 8) | (uint16_t(packet.data[3]) << 0);
	}

	return true;
}

bool CFA835::set_graphic_option(const bool en_manual_buffer_flush, const bool en_gamma_correction)
{
	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::GRAPHIC_CMD);
	packet.data = {{uint8_t(CFA835::GRAPHIC_CMD_CODE::GRAPHIC_OPT), 0}};

	if(en_manual_buffer_flush)
	{
		packet.data[1] |= 0x01U;
	}

	if(en_gamma_correction)
	{
		packet.data[1] |= 0x02U;
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

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::GRAPHIC_CMD), uint8_t(CFA835::GRAPHIC_CMD_CODE::GRAPHIC_OPT)) )
	{
		return false;
	}

	return true;
}

bool CFA835::set_gpio(const ONBOARD_GPIO gpio, const uint8_t duty_percent, const uint8_t drive_mode)
{
	CFA835::CFA835_Packet packet;
	packet.cmd  = uint8_t(CFA835::OP_CODE::GPIO_CONFIG);
	packet.data = {{uint8_t(gpio), duty_percent, drive_mode}};
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

	if( ! packet.is_ack_response_to(uint8_t(CFA835::OP_CODE::GPIO_CONFIG)) )
	{
		return false;
	}

	return true;
}

bool CFA835::send_packet(const CFA835_Packet packet, const std::chrono::milliseconds& max_wait)
{
	std::vector<uint8_t> temp;

	if( ! packet.serialize(&temp) )
	{
		return false;
	}

	size_t num_written = 0;
	do
	{
		ssize_t ret = write(m_fd, temp.data(), temp.size());
		if(ret < 0)
		{
			return false;
		}

		SPDLOG_DEBUG("CFA835::send_packet: {:d}", ret);

		num_written += size_t(ret);

	} while(num_written < temp.size());

	return num_written == temp.size();
}

bool CFA835::wait_for_packet(CFA835_Packet* out_packet, const std::chrono::milliseconds& max_wait)
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
		SPDLOG_WARN("CFA835::wait_for_packet: deserialize failed");
		return false;
	}

	return true;

}

bool CFA835::wait_for_read(uint8_t* out_data, size_t len, const std::chrono::milliseconds& max_wait)
{
	// TODO: update to use epoll and enforce max_wait

	size_t num_read = 0;
	do
	{
		ssize_t ret = read(m_fd, out_data, len);
		if(ret < 0)
		{
			return false;
		}
		
		if(ret == 0)
		{
			return false;
		}

		SPDLOG_DEBUG("CFA835::wait_for_read: {:d}", ret);

		num_read += size_t(ret);

	} while(num_read < len);

	return num_read == len;
}
