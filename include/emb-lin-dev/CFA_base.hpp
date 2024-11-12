/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <cstdint>

class CFA_Packet
{
public:
	CFA_Packet()
	{

	}
	~CFA_Packet()
	{

	}

	bool is_command() const
	{
		return (cmd & 0xC0U) == 0x00U;
	}

	bool is_response() const
	{
		return (cmd & 0xC0U) == 0x40U;
	}

	bool is_report() const
	{
		return (cmd & 0xC0U) == 0x80U;
	}

	bool is_error() const
	{
		return (cmd & 0xC0U) == 0xC0U;
	}

	uint8_t get_cmd() const
	{
		return cmd & 0x3FU;
	}

	// Check packet is a response to a particular command, and not eg an error packet
	bool is_ack_response_to(const uint8_t cmd) const
	{
		return is_response() && (get_cmd() == cmd);
	}

	// For nested commands like CFA835::OP_CODE::GRAPHIC_CMD
	bool is_ack_response_to(const uint8_t cmd, const uint8_t subcmd) const
	{
		return is_response() && (get_cmd() == cmd) && (data.size() >= 1) && (data[0] == subcmd);
	}

	bool is_error_response_to(const uint8_t cmd) const
	{
		return is_error() && (get_cmd() == cmd);
	}

	// For nested commands like CFA835::OP_CODE::GRAPHIC_CMD
	bool is_error_response_to(const uint8_t cmd, const uint8_t subcmd) const
	{
		return is_error() && (get_cmd() == cmd) && (data.size() >= 1) && (data[0] == subcmd);
	}

	bool is_cmd_error_eq(const uint8_t err) const
	{
		return data[1] == err;
	}

	bool is_subcmd_error_eq(const uint8_t err) const
	{
		return (data.size() >= 2) && (data[1] == err);
	}

	bool is_subcommand_error_eq(const uint8_t err) const
	{
		return (data.size() >= 3) && (data[2] == err);
	}

	bool serialize(std::vector<uint8_t>* out_buf) const;
	bool deserialize(const std::vector<uint8_t>& buf);
	uint16_t calc_crc() const;

	uint8_t cmd;
	std::vector<uint8_t> data;
	uint16_t crc;
};

class CFA_base
{
public:
		CFA_base();
		~CFA_base();
	
		bool open(const std::string& path);
		bool close();
		bool sync();

		static constexpr std::chrono::milliseconds PACKET_TIMEOUT = std::chrono::milliseconds(2000);
		static constexpr uint8_t get_msb(const uint16_t x)
		{
			return (x & 0xFF00U) >> 8;
		}
		static constexpr uint8_t get_lsb(const uint16_t x)
		{
			return (x & 0x00FFU) >> 0;
		}

		// These op-codes have fairly similar (but not always identical) operation among the CFA635 / CFA735 / CFA835 / CFA039A0-N-V family
		enum class COMMON_OP_CODE : uint8_t
		{
			PING                     = 0x00,
			GET_MODULE_INFO          = 0x01,
			RESTART                  = 0x05,
			CLEAR_DISPLAY            = 0x06,
			KEYPAD_REPORTING         = 0x17,
			READ_KEYPAD              = 0x18,
			DISPLAY_KEYPAD_BACKLIGHT = 0x0E
		};

		enum class RESTART_TYPE : uint8_t
		{
			RELOAD_BOOT_SETTINGS,
			RESTART_HOST,
			POWER_OFF_HOST,
			RESTART_DISPLAY,
			RESTORE_DEFAULT_SETTINGS,
			RESTART_DISPLAY_TO_BOOTLOADER // only CFA039A0-N-V
		};

		// Commands that are common in the CFA635 / CFA735 / CFA835 / CFA039A0-N-V family
		// info
		bool send_ping(const std::string& msg);
		bool get_module_info(const bool serial_nversion, std::string* const out_info);

		bool restart_display(const RESTART_TYPE restart_type);

		bool clear_display();

		bool set_brightness(const uint8_t display_percent);
		bool set_brightness(const uint8_t display_percent, const uint8_t keypad_percent);

		// keypad
		bool set_keypad_reporting_mask(const uint8_t press_mask, const uint8_t release_mask);
		bool poll_keypad(uint8_t* const out_keys_down, uint8_t* const out_keys_pressed, uint8_t* const out_keys_released);

		bool send_packet(const CFA_Packet& packet, const std::chrono::milliseconds& max_wait);
		bool send_buffer(const std::vector<uint8_t>& buf, const std::chrono::milliseconds& max_wait);
		bool wait_for_packet(CFA_Packet* out_packet, const std::chrono::milliseconds& max_wait);
		bool wait_for_read(uint8_t* out_data, size_t len, const std::chrono::milliseconds& max_wait);

protected:


	int m_fd;
};
