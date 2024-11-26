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
#include "emb-lin-dev/gpio_base.hpp"

class TCA9539 : public I2C_dev_base, public gpio_base
{
public:
	TCA9539(const std::shared_ptr<I2C_bus_base>& bus, const long id);
	~TCA9539() override;
	enum class CMD_CODE : uint8_t
	{
		IN0          = 0x00U,
		IN1          = 0x01U,
		OUT0         = 0x02U,
		OUT1         = 0x03U,
		CONF0        = 0x06U,
		CONF1        = 0x07U
	};

	bool read_input(uint16_t* const out_reg);
	bool write_output(const uint16_t reg);

	bool set_pin_input(const uint16_t reg);

	// gpio_base
	bool set_line(const unsigned int idx, const int value) override;
	bool get_line(const unsigned int idx, int* const out_value) override;

	bool set_all_lines(const uint64_t value) override;
	bool get_all_lines(uint64_t* const out_value) override;

	size_t get_num_lines() const
	{
		return 16;
	}

protected:
	bool set_reg_16(const uint8_t a_low, const uint16_t reg);
	bool get_reg_16(const uint8_t a_low, uint16_t * const out_reg);
};