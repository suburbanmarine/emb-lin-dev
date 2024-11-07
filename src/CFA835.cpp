/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * This software is distrubuted in the hope it will be useful, but without any warranty, including the implied warrranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE.txt for details.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#include "emb-lin-dev/CFA835.hpp"

#include <spdlog/spdlog.h>

#include <cstring>

CFA835::CFA835()
{
	
}

CFA835::~CFA835()
{
	
}

bool CFA835::send_ping(const std::string& msg)
{
	if(msg.size() > 124)
	{
		return false;
	}

	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::PING);
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::PING)) )
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
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::GET_MODULE_INFO);
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::GET_MODULE_INFO)) )
	{
		return false;
	}

	out_info->clear();
	out_info->insert(out_info->end(), packet.data.begin(), packet.data.end());

	return true;
}

bool CFA835::set_brightness(const uint8_t display_percent, const uint8_t keypad_percent)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::DISPLAY_KEYPAD_BACKLIGHT);
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::DISPLAY_KEYPAD_BACKLIGHT)) )
	{
		return false;
	}

	return true;
}

bool CFA835::write_user_flash(const std::vector<uint8_t>& data)
{
	if( data.empty() || (data.size() > 124) )
	{
		return false;
	}

	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::WRITE_USER_FLASH);
	packet.data = data;
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::WRITE_USER_FLASH)) )
	{
		return false;
	}

	return true;
}
bool CFA835::read_user_flash(const uint8_t num_to_read, std::vector<uint8_t>* const out_data)
{
	if( (num_to_read == 0) || (num_to_read > 124) )
	{
		return false;
	}

	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::READ_USER_FLASH);
	packet.data = {{num_to_read}};
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::WRITE_USER_FLASH)) )
	{
		return false;
	}

	if(out_data)
	{
		*out_data = std::move(packet.data);	
	}
	
	return true;
}

bool CFA835::clear_display()
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::CLEAR_DISPLAY);
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::CLEAR_DISPLAY)) )
	{
		return false;
	}

	return true;
}

bool CFA835::all_on_display()
{
	return draw_rectangle(0, 0, WIDTH-1, HEIGHT-1, uint8_t(LCD_SHADE::DARK), uint8_t(LCD_SHADE::DARK));
}

bool CFA835::restart_display(const RESTART_TYPE restart_type)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::RESTART);

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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::RESTART)) )
	{
		return false;
	}

	return true;
}

bool CFA835::set_keypad_reporting_mask(const uint8_t press_mask, const uint8_t release_mask)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::KEYPAD_REPORTING);
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::KEYPAD_REPORTING)) )
	{
		return false;
	}

	return true;
}

bool CFA835::poll_keypad(uint8_t* const out_keys_down, uint8_t* const out_keys_pressed, uint8_t* const out_keys_released)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::READ_KEYPAD);
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::READ_KEYPAD)) )
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

	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::WRITE_TEXT);
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::WRITE_TEXT)) )
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

	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::DISPLAY_CURSOR_POSITION);
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::DISPLAY_CURSOR_POSITION)) )
	{
		return false;
	}

	return true;
}
bool CFA835::set_cursor_style(const CURSOR_STYLE style)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::CURSOR_STYLE);
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::CURSOR_STYLE)) )
	{
		return false;
	}

	return true;
}

bool CFA835::get_cfa_fbscab_count(uint8_t* out_count)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::FBSCAB_CMD);
	packet.data = {{uint8_t(FBSCAB_CMD_CODE::GET_MODULE_INFO)}};
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::FBSCAB_CMD), uint8_t(FBSCAB_CMD_CODE::GET_MODULE_INFO)) )
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
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::FBSCAB_CMD);
	packet.data = {{uint8_t(FBSCAB_CMD_CODE::GET_MODULE_INFO), fbscab_idx}};
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::FBSCAB_CMD), uint8_t(FBSCAB_CMD_CODE::GET_MODULE_INFO)) )
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
		CFA_Packet packet;
		packet.cmd  = uint8_t(OP_CODE::FBSCAB_CMD);
		packet.data = {{uint8_t(FBSCAB_CMD_CODE::READ_DOW_INFO), fbscab_idx, dow_idx}};
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

		if( packet.is_error_response_to(uint8_t(OP_CODE::FBSCAB_CMD), uint8_t(FBSCAB_CMD_CODE::READ_DOW_INFO)) )
		{
			return false;
		}

		if( ! packet.is_ack_response_to(uint8_t(OP_CODE::FBSCAB_CMD), uint8_t(FBSCAB_CMD_CODE::READ_DOW_INFO)) )
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
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::FBSCAB_CMD);
	packet.data = {{uint8_t(FBSCAB_CMD_CODE::READ_DOW_TEMP), fbscab_idx, dow_idx}};
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::FBSCAB_CMD), uint8_t(FBSCAB_CMD_CODE::READ_DOW_TEMP)) )
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
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::GRAPHIC_CMD);
	packet.data = {{uint8_t(GRAPHIC_CMD_CODE::GRAPHIC_OPT), 0}};

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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::GRAPHIC_CMD), uint8_t(GRAPHIC_CMD_CODE::GRAPHIC_OPT)) )
	{
		return false;
	}

	return true;
}
bool CFA835::flush_graphics_buffer()
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::GRAPHIC_CMD);
	packet.data = {{uint8_t(GRAPHIC_CMD_CODE::BUFFER_FLUSH)}};
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::GRAPHIC_CMD), uint8_t(GRAPHIC_CMD_CODE::BUFFER_FLUSH)) )
	{
		return false;
	}

	return true;
}
bool CFA835::draw_buffer(const bool transparency, const bool invert, const uint8_t x_start, const uint8_t y_start, const uint8_t width, const uint8_t height, const std::vector<uint8_t>& buf)
{
	if(buf.empty() || (buf.size() > (WIDTH*HEIGHT)))
	{
		return false;
	}

	constexpr bool rle = true;
	std::vector<uint8_t> rle_buf;
	std::vector<uint8_t> const * buf_to_send;

	uint8_t opt = 0x00U;
	if(transparency)
	{
		opt |= (1U << 0);
	}
	if(invert)
	{
		opt |= (1U << 1);
	}
	if(rle)
	{
		opt |= (1U << 2);
	}

	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::GRAPHIC_CMD);
	packet.data = {{uint8_t(GRAPHIC_CMD_CODE::WRITE_BUFFER), opt, x_start, y_start, width, height}};
	packet.crc  = packet.calc_crc();

	if(rle)
	{
		rle_buf.reserve(buf.size());
		rle_compress(buf, &rle_buf);
		buf_to_send = &rle_buf;
	}
	else
	{
		buf_to_send = &buf;
	}

	if( ! send_packet(packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to send_packet");
		return false;
	}

	if( ! send_buffer(*buf_to_send, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to send_buffer");
		return false;
	}

	if( ! wait_for_packet(&packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to wait_for_packet");
		return false;
	}

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::GRAPHIC_CMD), uint8_t(GRAPHIC_CMD_CODE::WRITE_BUFFER)) )
	{
		return false;
	}

	return true;
}

bool CFA835::draw_pixel(const uint8_t x, const uint8_t y, const uint8_t shade)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::GRAPHIC_CMD);
	packet.data = {{uint8_t(GRAPHIC_CMD_CODE::DRAW_PIXEL), x, y, shade}};
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::GRAPHIC_CMD), uint8_t(GRAPHIC_CMD_CODE::DRAW_PIXEL)) )
	{
		return false;
	}

	return true;
}
bool CFA835::draw_line(const uint8_t x_start, const uint8_t y_start, const uint8_t x_end, const uint8_t y_end, const uint8_t line_shade)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::GRAPHIC_CMD);
	packet.data = {{uint8_t(GRAPHIC_CMD_CODE::DRAW_LINE), x_start, y_start, x_end, y_end, line_shade}};
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::GRAPHIC_CMD), uint8_t(GRAPHIC_CMD_CODE::DRAW_LINE)) )
	{
		return false;
	}

	return true;
}
bool CFA835::draw_rectangle(const uint8_t x_top_left, const uint8_t y_top_left, const uint8_t width, const uint8_t height, const uint8_t line_shade, const uint8_t fill_shade)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::GRAPHIC_CMD);
	packet.data = {{uint8_t(GRAPHIC_CMD_CODE::DRAW_RECTANGLE), x_top_left, y_top_left, width, height, line_shade, fill_shade}};
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::GRAPHIC_CMD), uint8_t(GRAPHIC_CMD_CODE::DRAW_RECTANGLE)) )
	{
		return false;
	}

	return true;
}
bool CFA835::draw_circle(const uint8_t x_center, const uint8_t y_center, const uint8_t radius, const uint8_t line_shade, const uint8_t fill_shade)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::GRAPHIC_CMD);
	packet.data = {{uint8_t(GRAPHIC_CMD_CODE::DRAW_CIRCLE), x_center, y_center, radius, line_shade, fill_shade}};
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::GRAPHIC_CMD), uint8_t(GRAPHIC_CMD_CODE::DRAW_CIRCLE)) )
	{
		return false;
	}

	return true;
}

bool CFA835::set_gpio(const ONBOARD_GPIO gpio, const uint8_t duty_percent, const uint8_t drive_mode)
{
	const uint8_t pinfun_drive = 0x08U | (drive_mode & 0x07U);
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::GPIO_CONFIG);
	packet.data = {{uint8_t(gpio), duty_percent, pinfun_drive}};
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::GPIO_CONFIG)) )
	{
		return false;
	}

	return true;
}

bool CFA835::set_all_gpio_led(const uint8_t val)
{
	bool ret = true;
	const std::array<CFA835::ONBOARD_GPIO, 8> all_leds = {{
		CFA835::ONBOARD_GPIO::LED0R, CFA835::ONBOARD_GPIO::LED0G,
		CFA835::ONBOARD_GPIO::LED1R, CFA835::ONBOARD_GPIO::LED1G,
		CFA835::ONBOARD_GPIO::LED2R, CFA835::ONBOARD_GPIO::LED2G,
		CFA835::ONBOARD_GPIO::LED3R, CFA835::ONBOARD_GPIO::LED3G
	}};

	for(const auto& x : all_leds)
	{
		if( ! set_gpio(x, val, 0) )
		{
			ret = false;
		}
	}

	return ret;
}

bool CFA835::set_all_green_gpio_led(const uint8_t val)
{
	bool ret = true;
	const std::array<CFA835::ONBOARD_GPIO, 4> all_leds = {{
		CFA835::ONBOARD_GPIO::LED0G,
		CFA835::ONBOARD_GPIO::LED1G,
		CFA835::ONBOARD_GPIO::LED2G,
		CFA835::ONBOARD_GPIO::LED3G
	}};

	for(const auto& x : all_leds)
	{
		if( ! set_gpio(x, val, 0) )
		{
			ret = false;
		}
	}

	return ret;
}

bool CFA835::set_all_red_gpio_led(const uint8_t val)
{
	bool ret = true;
	const std::array<CFA835::ONBOARD_GPIO, 4> all_leds = {{
		CFA835::ONBOARD_GPIO::LED0R,
		CFA835::ONBOARD_GPIO::LED1R,
		CFA835::ONBOARD_GPIO::LED2R,
		CFA835::ONBOARD_GPIO::LED3R
	}};

	for(const auto& x : all_leds)
	{
		if( ! set_gpio(x, val, 0) )
		{
			ret = false;
		}
	}

	return ret;
}

size_t CFA835::rle_compress(const std::vector<uint8_t>& in, std::vector<uint8_t>* const out)
{
	uint8_t const * ptr = in.data();
	uint8_t const * end = in.data() + in.size();

	// TODO check boundary condition and test...
	while(ptr != end)
	{
		// only high 8 bits matter, so drop the low 8 for better compression
		const uint8_t this_val = pixel_shade_mask(ptr[0]);

		if((ptr+1) == end)
		{
			if(this_val == 0x03)
			{
				// force encode 0x03 as a run

				// output run
				out->push_back(0x03);
				out->push_back(1);
				out->push_back(this_val);

				// consume input
				ptr++;
			}
			else
			{
				// not in a run
				out->push_back(this_val);
				ptr++;
			}
		}
		else
		{
			const uint8_t next_val = pixel_shade_mask(ptr[1]);

			if( this_val == next_val )
			{
				// find length of run
				size_t run_len = 2;
				while( (run_len < 255) && ((ptr+run_len) != end) && (this_val == pixel_shade_mask(ptr[run_len])) )
				{
					run_len++;
				}

				if(run_len > 2)
				{
					// output run
					out->push_back(0x03);
					out->push_back(run_len);
					out->push_back(this_val);
				}
				else
				{
					//2 is short to just output
					out->push_back(this_val);
					out->push_back(this_val);
				}

				// consume input
				ptr += run_len;
			}
			else if(this_val == 0x03)
			{
				// force encode 0x03 as a run

				// output run
				out->push_back(0x03);
				out->push_back(1);
				out->push_back(this_val);

				// consume input
				ptr++;
			}
			else
			{
				// not in a run
				out->push_back(this_val);
				ptr++;
			}
		}
	}

	return out->size();
}
