/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#pragma once

class BIC_2200
{
public:

	enum class CMD_ID: uint16_t
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

	BIC_2200();
	~BIC_2200();

	bool open(const std::string& iface);
	bool close();
protected:

	bool wait_tx_can_packet(const std::chrono::microseconds& max_wait_time, const can_frame& frame);
	bool wait_rx_can_packet(const std::chrono::microseconds& max_wait_time, can_frame* const out_frame);

	int m_fd;
};