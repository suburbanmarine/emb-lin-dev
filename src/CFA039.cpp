/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#include "emb-lin-dev/CFA039.hpp"

#include <spdlog/spdlog.h>

CFA039::CFA039()
{

}

CFA039::~CFA039()
{
	
}

bool CFA039::set_led_color(const uint8_t led_mask, const uint8_t key_mask, const uint8_t r, const uint8_t g, const uint8_t b)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::LIGHTING);
	packet.data = {{
		led_mask,
		key_mask,
		r,
		g,
		b
	}};
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::LIGHTING)) )
	{
		return false;
	}

	return true;
}

bool CFA039::set_led_autodim(const uint8_t l1_timeout_s, const uint8_t l2_timeout_s, const uint8_t l1_dim_percent, const uint8_t l2_dim_percent)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::LIGHTING);
	packet.data = {{
		0x07, // lcd, keypad, indictor leds
		0x03, // wake on keypad or touchscreen, but not cmd packet I/O
		l1_timeout_s,
		l2_timeout_s,
		l1_dim_percent,
		l2_dim_percent
	}};
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

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::LIGHTING)) )
	{
		return false;
	}

	return true;
}

bool CFA039::send_image_argb8888(const uint16_t x, const uint16_t y, const uint8_t z, std::vector<uint8_t>& img_argb8888, const size_t width, const size_t height)
{
	CFA_Packet packet;
	packet.cmd  = uint8_t(OP_CODE::IMAGE_OBJ_CMD);
	packet.data = {{
		uint8_t(IMAGE_OBJ_CMD_CODE::HOST_LOAD),
		0,
		get_lsb(x), get_msb(x),
		get_lsb(y), get_msb(y),
		z,
		uint8_t(TOUCH_REPORTING_OPTION::OFF),
		uint8_t(IMAGE_FORMAT::ARGB8888),
		get_lsb(width), get_msb(width),
		get_lsb(height), get_msb(height),
		get_lsb(img_argb8888.size()), get_msb(img_argb8888.size()),
	}};
	packet.crc  = packet.calc_crc();

	if( ! send_packet(packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to send_packet");
		return false;
	}

	if( ! send_buffer(img_argb8888, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to send_buffer");
		return false;
	}

	if( ! wait_for_packet(&packet, PACKET_TIMEOUT) )
	{
		SPDLOG_ERROR("Failed to wait_for_packet");
		return false;
	}

	if( ! packet.is_ack_response_to(uint8_t(OP_CODE::IMAGE_OBJ_CMD), uint8_t(IMAGE_OBJ_CMD_CODE::HOST_LOAD)) )
	{
		return false;
	}

	return true;
}
