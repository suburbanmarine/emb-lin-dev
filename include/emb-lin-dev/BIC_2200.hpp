/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#pragma once

#include <linux/can.h>

#include <chrono>
#include <string>

#include <cstdint>

class BIC_2200
{
public:

	enum class CMD_OPCODE: uint16_t
	{
		OPERATION            = 0x0000U, 
		VOUT_SET             = 0x0020U,
		IOUT_SET             = 0x0030U,
		FAULT_STATUS         = 0x0040U,
		READ_VIN             = 0x0050U,
		READ_VOUT            = 0x0060U,
		READ_IOUT            = 0x0061U,
		READ_TEMPERATURE_1   = 0x0062U,
		READ_FAN_SPEED_1     = 0x0070U,
		READ_FAN_SPEED_2     = 0x0071U,
		MFR_ID_B0B5          = 0x0080U,
		MFR_ID_B6B11         = 0x0081U,
		MFR_MODEL_B0B5       = 0x0082U,
		MFR_MODEL_B6B11      = 0x0083U,
		MFR_REVISION_B0B5    = 0x0084U,
		MFR_LOCATION_B0B2    = 0x0085U,
		MFR_DATE_B0B5        = 0x0086U,
		MFR_SERIAL_B0B5      = 0x0087U,
		MFR_SERIAL_B6B11     = 0x0088U,
		SCALING_FACTOR       = 0x00C0U,
		SYSTEM_STATUS        = 0x00C1U, 
		SYSTEM_CONFIG        = 0x00C2U,
		DIRECTION_CTRL       = 0x0100U,
		REVERSE_VOUT_SET     = 0x0120U,
		REVERSE_IOUT_SET     = 0x0130U,
		BIDIRECTIONAL_CONFIG = 0x0140U
	};

	enum class SCALE_FACTOR: uint8_t
	{
		NOT_SUPPORTED = 0x00,
		MILLI         = 0x04,
		CENTI         = 0x05,
		DECI          = 0x06,
		ONE           = 0x07,
		TEN           = 0x08,
		HUNDRED       = 0x09
	};

	enum class EEP_CONFIG: uint8_t
	{
		IMMEDIATE    = 0x00,
		DELAY_1_MIN  = 0x01,
		DELAY_10_MIN = 0x02
	};

	enum class OP_INIT: uint8_t
	{
		POWER_OFF  = 0x00,
		POWER_ON   = 0x01,
		POWER_LAST = 0x02
	};

	struct SCALING_FACTORS
	{
		SCALE_FACTOR iin;
		SCALE_FACTOR temp1;
		SCALE_FACTOR fan_speed;
		SCALE_FACTOR vin;
		SCALE_FACTOR iout;
		SCALE_FACTOR vout;
	};

	constexpr static uint32_t GET_HOST_TO_BIC_ADDR(const uint8_t bic_addr)
	{
		return 0x000C0200U | bic_addr;
	};
	constexpr static uint32_t GET_BIC_TO_HOST_ADDR(const uint8_t bic_addr)
	{
		return 0x000C0300U | bic_addr;
	};
	constexpr static uint32_t HOST_BCAST_ADDR  = 0x00C03FF;

	BIC_2200();
	~BIC_2200();

	bool open(const std::string& iface);
	bool close();
protected:

	bool wait_tx_can_packet(const std::chrono::microseconds& max_wait_time, const can_frame& frame);
	bool wait_rx_can_packet(const std::chrono::microseconds& max_wait_time, can_frame* const out_frame);

	int m_fd;
};