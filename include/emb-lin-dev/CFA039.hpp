/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#pragma once

#include "emb-lin-dev/CFA_base.hpp"

class CFA039 : public CFA_base
{
public:
	CFA039();
	~CFA039();

	static constexpr size_t WIDTH  = 480U;
	static constexpr size_t HEIGHT = 128U;
	static constexpr size_t NUM_PX = WIDTH*HEIGHT;

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
		LIGHTING                 = 0x0E,
		KEYPAD_REPORTING         = 0x17,
		KEYPAD_POLL              = 0x18,
		TOUCHSCREEN_REPORTING    = 0x19,
		WATCHDOG                 = 0x1D,
		WRITE_TEXT               = 0x1F,
		READ_TEXT                = 0x20,
		IFACE_OPT                = 0x21,
		GPIO_CONFIG              = 0x22,
		IFACE_BRIDGE             = 0x24,
		FBSCAB_CMD               = 0x25,
		STORAGE_CMD              = 0x27,
		GRAPHIC_CMD              = 0x28,
		VIDEO_OBJ_CMD            = 0x29,
		IMAGE_OBJ_CMD            = 0x2A,
		SKETCH_SURFACE_CMD       = 0x2C,
		BUTTON_CMD               = 0x2F,
		SLIDER_CTRL_CMD          = 0x30,
		NUMBEREDIT_CTRL_CMD      = 0x31,
		CHECKBOX_CTRL_CMD        = 0x32,
		PROGRESS_BAR_CMD         = 0x33,
		RESERVED_DEBUGGING       = 0x3E
	};


	enum class STORAGE_CMD_CODE : uint8_t
	{
		FILE_OPEN_CLOSE      = 0x00U,
		FILE_SEEK            = 0x01U,
		FILE_READ            = 0x02U,
		FILE_WRITE           = 0x03U,
		FILE_DEL             = 0x04U,
		FILE_COPY            = 0x05U,
		FILE_SYSTEM_ERASE    = 0x06U,
		FILE_SYSTEM_CAPACITY = 0x07U
	};

	enum class GRAPHIC_CMD_CODE : uint8_t
	{
		GRAPHIC_OPT         = 0x00,
		BUFFER_FLUSH        = 0x01,
		BG_COLOR            = 0x02,
		REMOVE_OBJ          = 0x0A,
		MOVE_OBJ            = 0x0B,
		OBJ_TOUCH_REPORTING = 0x0C,
		OBJ_Z_IDX           = 0x0D
	};

	enum class IMAGE_OBJ_CMD_CODE : uint8_t
	{
		OBJ_LOAD      = 0x00,
		BTN_LOAD_DOWN = 0x01,
		BTN_LOAD_DIS  = 0x02,
		HOST_LOAD     = 0x03,
		TGL_BTN_STATE = 0x04
	};

	enum class IMAGE_FORMAT : uint8_t
	{
		MONO     = 0x00,
		GRAY     = 0x01,
		RGB565   = 0x02,
		ARGB1555 = 0x03,
		ARGB8888 = 0x04,
		FILE     = 0x05
	};

	enum class TOUCH_REPORTING_OPTION : uint8_t
	{
		OFF                       = 0x00,
		SINGLE_TOUCH_RELEASE      = 0x01,
		SINGLE_TOUCH_RELEASE_DRAG = 0x02,
		MULTIPLE                  = 0x03
	};

	bool send_image_argb8888(const uint16_t x, const uint16_t y, const uint8_t z, std::vector<uint8_t>& img_argb8888, const size_t width, const size_t height);
};
