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

#include "emb-lin-dev/I2C_dev_base.hpp"

class TCAL9539 : public I2C_dev_base
{
public:
	TCAL9539(const std::shared_ptr<I2C_bus_base>& bus, const long id);
	~TCAL9539() override;
	enum class CMD_CODE : uint8_t
	{
		IN0          = 0x00U,
		IN1          = 0x01U,
		OUT0         = 0x02U,
		OUT1         = 0x03U,
		CONF0        = 0x06U,
		CONF1        = 0x07U,
		OUT_DRIVE0   = 0x40U,
		OUT_DRIVE1   = 0x41U,
		OUT_DRIVE2   = 0x42U,
		OUT_DRIVE3   = 0x43U,
		INPUT_LATCH0 = 0x44U,
		INPUT_LATCH1 = 0x45U,
		PU_PD_EN0    = 0x46U,
		PU_PD_EN1    = 0x47U,
		PU_PD0       = 0x48U,
		PU_PD1       = 0x49U,
		OUTPUT_CONF  = 0x4FU
	};

	bool read_input(uint16_t* const out_reg);
	bool write_output(const uint16_t reg);

	bool set_pin_input(const uint16_t reg);

	bool set_pu_pd_en(const uint16_t reg);
	bool set_pu_pd(const uint16_t reg);

protected:
	bool set_reg_16(const uint8_t a_low, const uint16_t reg);
	bool get_reg_16(const uint8_t a_low, uint16_t * const out_reg);
};