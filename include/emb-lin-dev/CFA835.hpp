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

#pragma once

#include <chrono>
#include <vector>
#include <string>

#include <cstdint>

class CFA835
{
public:
	CFA835();
	~CFA835();

	class CFA835_Packet
	{
	public:
		CFA835_Packet()
		{

		}
		~CFA835_Packet()
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

	static constexpr size_t WIDTH  = 244U;
	static constexpr size_t HEIGHT = 68U;
	static constexpr size_t NUM_PX = WIDTH*HEIGHT;

	enum class ERROR_CODE : uint8_t
	{
		UNKNOWN                    = 0x01,
		UNKNOWN_CMD                = 0x02,
		INVALID_LEN_OPT            = 0x03,
		FLASH_WRITE_FAILED         = 0x04,
		FLASH_READ_FAILED          = 0x05,
		CFA_FBSCAB_IDX_DNE         = 0x06,
		CFA_FBSCAB_NO_RESP         = 0x07,
		SDCARD_NOT_PRESENT         = 0x08,
		SDCARD_NOT_FORMATTED       = 0x09,
		SDCARD_FILE_NOT_FOUND      = 0x0A,
		SDCARD_UNKNOWN_ERROR       = 0x0B,
		SDCARD_FILE_READ_ERROR     = 0x0C,
		SDCARD_FILE_WRITE_ERROR    = 0x0D,
		SDCARD_FILE_HEADER_INVALID = 0x0E,
		SDCARD_FILE_ALREADY_OPEN   = 0x0F,
		SDCARD_FILE_OP_FAILED      = 0x10,
		SDCARD_FILE_NOT_OPEN       = 0x11,
		GFX_STREAM_ALREADY_STARTED = 0x12,
		GFX_OUT_OF_BOUNDS          = 0x13,
		VIDEO_NOT_OPEN_IN_SLOT     = 0x14,
		GFX_STREAM_TIMEOUT         = 0x15,
		GPIO_NOT_SET_ATX           = 0x16,
		IFACE_NOT_EN               = 0x17,
		IFACE_NA                   = 0x18
	};

	enum class OP_CODE : uint8_t
	{
		PING                     = 0x00,
		GET_MODULE_INFO          = 0x01,
		WRITE_USER_FLASH         = 0x02,
		READ_USER_FLASH          = 0x03,
		STORE_CURRENT_STATE      = 0x04,
		RESTART                  = 0x05,
		CLEAR_DISPLAY            = 0x06,
		DISPLAY_CURSOR_POSITION  = 0x0B,
		CURSOR_STYLE             = 0x0C,
		CONTRAST                 = 0x0D,
		DISPLAY_KEYPAD_BACKLIGHT = 0x0E,
		KEYPAD_REPORTING         = 0x17,
		READ_KEYPAD              = 0x18,
		WATCHDOG                 = 0x1D,
		WRITE_TEXT               = 0x1F,
		READ_TEXT                = 0x20,
		IFACE_OPT                = 0X21,
		GPIO_CONFIG              = 0X22,
		IFACE_BRIDGE             = 0x24,
		FBSCAB_CMD               = 0x25,
		SDCARD_CMD               = 0x27,
		GRAPHIC_CMD              = 0x28,
		VIDEO_CMD                = 0x29,
		RESERVED_DEBUGGING       = 0x3E
	};

	enum class FBSCAB_CMD_CODE : uint8_t
	{
		GET_MODULE_INFO    = 0x00,
		FAN_SETTINGS       = 0x01,
		READ_FAN_TACH      = 0x02,
		READ_DOW_INFO      = 0x03,
		READ_DOW_TEMP      = 0x04,
		GPIO_LEVEL         = 0x05,
		RESET_SEARCH       = 0x06,
		LIVE_FAN_TEMP_DISP = 0x07,
		AUTO_FAN_CTRL      = 0x08,
	};
	enum class SDCARD_CMD_CODE : uint8_t
	{
		FILE_OPEN_CLOSE = 0x00,
		FILE_SEEK       = 0x01,
		FILE_READ       = 0x02,
		FILE_WRITE      = 0x03,
		FILE_DEL        = 0x04
	};
	enum class GRAPHIC_CMD_CODE : uint8_t
	{
		GRAPHIC_OPT            = 0x00,
		BUFFER_FLUSH           = 0x01,
		WRITE_BUFFER           = 0x02,
		LOAD_SDCARD_IMAGE      = 0x03,
		SAVE_SCREENSHOT_SDCARD = 0x04,
		DRAW_PIXEL             = 0x05,
		DRAW_LINE              = 0x06,
		DRAW_RECTANGLE         = 0x07,
		DRAW_CIRCLE            = 0x08
	};
	enum class VIDEO_CMD_CODE : uint8_t
	{
		LOAD_VIDEO = 0x00,
		VIDEO_CTRL = 0x01,
	};

	enum class CURSOR_STYLE : uint8_t
	{
		HIDDEN                        = 0x00,
		BLINKING_BLOCK                = 0x01,
		UNDERSCORE                    = 0x02,
		BLINKING_BLOCK_AND_UNDERSCORE = 0x03,
		INVERTING_BLINKING_BLOCK      = 0x04
	};

	enum class ONBOARD_GPIO : uint8_t
	{
		GPIO0  = 0x00U, // H1 pin 11
		GPIO1  = 0x01U, // H1 pin 12, ATX Power Sense
		GPIO2  = 0x02U, // H1 pin 9, ATX Power Control
		GPIO3  = 0x03U, // H1 pin 10, ATX Reset Control
		GPIO4  = 0x04U, // H1 pin 13
		LED3G  = 0x05U,
		LED3R  = 0x06U,
		LED2G  = 0x07U,
		LED2R  = 0x08U,
		LED1G  = 0x09U,
		LED1R  = 0x0AU,
		LED0G  = 0x0BU,
		LED0R  = 0x0CU,
		GPIO5  = 0x0DU, // H1 pin 5, ADC0
		GPIO6  = 0x0EU, // H1 pin 6, ADC1
		GPIO7  = 0x0FU, // H1 pin 1, UART TX
		GPIO8  = 0x10U, // H1 pin 2, UART RX
		GPIO9  = 0x11U, // H1 pin 3
		GPIO10 = 0x12U, // H1 pin 4
		GPIO11 = 0x13U, // H1 pin 7
		GPIO12 = 0x14U  // H1 pin 8
	};

	enum class LCD_SHADE : uint8_t
	{
		LITE     = 0x00U,
		SHADE_1  = 0x11U,
		SHADE_2  = 0x22U,
		SHADE_3  = 0x33U,
		SHADE_4  = 0x44U,
		SHADE_5  = 0x55U,
		SHADE_6  = 0x66U,
		SHADE_7  = 0x77U,
		SHADE_8  = 0x88U,
		SHADE_9  = 0x99U,
		SHADE_10 = 0xAAU,
		SHADE_11 = 0xBBU,
		SHADE_12 = 0xCCU,
		SHADE_13 = 0xDDU,
		SHADE_14 = 0xEEU,
		DARK     = 0xFFU
	};
	static constexpr uint8_t pixel_shade_mask(const uint8_t val)
	{
		return val & 0xF0U;
	}

	// Drive strength options, ignored for onboard LEDs
	enum class GPIO_DRIVE_MODE : uint8_t
	{
		STRONG_UP_WEAK_DOWN   = 0x00U,
		// STRONG_UP_STRONG_DOWN = 0x01U,
		HIZ                   = 0x02U, // eg HI-Z input
		WEAK_UP_STRONG_DOWN   = 0x03U,
		STRONG_UP_HIZ_DOWN    = 0x04U, // eg open source
		STRONG_UP_STRONG_DOWN = 0x05U,
		HIZ_UP_STRONG_DOWN    = 0x07U  // eg open drain
	};

	enum class RESTART_TYPE : uint8_t
	{
		RELOAD_BOOT_SETTINGS,
		RESTART_HOST,
		POWER_OFF_HOST,
		RESTART_DISPLAY,
		RESTORE_DEFAULT_SETTINGS
	};

	bool open(const std::string& path);
	bool close();
	bool sync();

	// info
	bool send_ping(const std::string& msg);
	bool get_module_info(const bool serial_nversion, std::string* const out_info);

	bool set_brightness(const uint8_t display_percent, const uint8_t keypad_percent);
	bool set_gpio(const ONBOARD_GPIO gpio, const uint8_t duty_percent, const uint8_t drive_mode);

	// set all onboard indication leds
	// does not change backlight or keypad brightness
	bool set_all_gpio_led(const uint8_t val);
	bool set_all_green_gpio_led(const uint8_t val);
	bool set_all_red_gpio_led(const uint8_t val);

	bool write_user_flash(const std::vector<uint8_t>& data);
	bool read_user_flash(const uint8_t num_to_read, std::vector<uint8_t>* const out_data);

	bool clear_display();
	bool all_on_display();
	bool restart_display(const RESTART_TYPE restart_type);

	// keypad
	bool set_keypad_reporting_mask(const uint8_t press_mask, const uint8_t release_mask);
	bool poll_keypad(uint8_t* const out_keys_down, uint8_t* const out_keys_pressed, uint8_t* const out_keys_released);

	// text display
	bool write_text(const int col, const int row, const std::string& str);

	bool set_cursor_position(const uint8_t col, const uint8_t row);
	bool set_cursor_style(const CURSOR_STYLE style);

	// CFA-FBSCAB group
	bool get_cfa_fbscab_count(uint8_t* out_count);
	bool get_cfa_fbscab_info(const uint8_t fbscab_idx, std::string* const out_info);
	bool get_cfa_fbscab_dow_info(const uint8_t fbscab_idx, std::vector<std::string>* const out_info);
	bool get_cfa_fbscab_dow_temp(const uint8_t fbscab_idx, const uint8_t dow_idx, uint16_t* out_temp); // out_temp is a bitfield, TODO: add decode helpers

	// graphics options
	bool set_graphic_option(const bool en_manual_buffer_flush, const bool en_gamma_correction);
	
	// draw full screen buffer
	// if grayscale U8 or grayscale U8 + binary transparency
	// cv::Mat CV_8UC1, only top 4 bytes are significant for 16 shades of gray
	// send left to right, top to bottom
	// bool draw_screen(const cv::Mat& im, const bool invert_color, const bool rle);
	// As before, but with boolean transparency mask (both im and mask are CV_8UC1, but im is set to 0 everywhere mask is 0, and the display's transparency mode is enabled)
	// bool draw_screen(const cv::Mat& im, const cv::Mat& mask, const bool invert_color, const bool rle);
	bool flush_graphics_buffer();

	bool draw_buffer(const bool transparency, const bool invert, const uint8_t x_start, const uint8_t y_start, const uint8_t width, const uint8_t height, const std::vector<uint8_t>& buf);
	bool draw_pixel(const uint8_t x, const uint8_t y, const uint8_t shade);
	bool draw_line(const uint8_t x_start, const uint8_t y_start, const uint8_t x_end, const uint8_t y_end, const uint8_t line_shade);
	bool draw_rectangle(const uint8_t x_top_left, const uint8_t y_top_left, const uint8_t width, const uint8_t height, const uint8_t line_shade, const uint8_t fill_shade);
	bool draw_circle(const uint8_t x_center, const uint8_t y_center, const uint8_t radius, const uint8_t line_shade, const uint8_t fill_shade);

	bool send_packet(const CFA835_Packet& packet, const std::chrono::milliseconds& max_wait);
	bool send_buffer(const std::vector<uint8_t>& buf, const std::chrono::milliseconds& max_wait);
	bool wait_for_packet(CFA835_Packet* out_packet, const std::chrono::milliseconds& max_wait);
	bool wait_for_read(uint8_t* out_data, size_t len, const std::chrono::milliseconds& max_wait);

	// 0x03, len, value
	size_t rle_compress(const std::vector<uint8_t>& in, std::vector<uint8_t>* const out);

	static constexpr std::chrono::milliseconds PACKET_TIMEOUT = std::chrono::milliseconds(2000);

protected:
	int m_fd;
};
