/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * This software is distrubuted in the hope it will be useful, but without any warranty, including the implied warrranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE.txt for details.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2023 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#pragma once

#include "emb-lin-dev/SPIdev.hpp"
#include "RTD_util.hpp"

#include <string_view>
#include <memory>

#include <cstdint>

class max38165_spidev
{
public:

	// use existing spidev without configuring it
	bool init(const std::shared_ptr<SPIdev>& spidev, const std::shared_ptr<RTD_util>& rtd_lut, const float r_ref);

	static bool configure_spidev(const std::shared_ptr<SPIdev>& spidev);

	bool enable_vbias();
	bool disable_vbias();
	bool start_oneshot();

	bool read_resistance_reg(uint16_t* const r_reg);
	
	//blocks during conversion
	bool read_resistance_oneshot(float* out_res_ratio);

	//blocks during conversion
	bool get_temp_oneshot(float* out_degC);

	bool set_high_fault(const uint16_t val);
	bool set_low_fault(const uint16_t val);

protected:

	// max38165 reference resistor
	float m_r_ref;

	std::shared_ptr<SPIdev> m_spidev;

	std::shared_ptr<const RTD_util> m_rtd_lut;
};
