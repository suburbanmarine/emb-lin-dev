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
#include <map>

#include <array>
#include <optional>
#include <vector>

#include <cstddef>
#include <cstdint>

class MB85RC : public I2C_dev_base
{
public:
	// NXP says 12bit mf, 9bit part id, 3bit die rev
	// ramXeed says 12bit mf, 4bit density, 8bit proprietary use
	enum class DEVICE_ID_CODE : uint32_t
	{
		MB85RC64TA  = 0x00A358U,
		MB85RC256TY = 0x00A498U,
		MB85RC512TY = 0x00A598U,
	};

	MB85RC(const std::shared_ptr<I2C_bus_base>& bus, const long id);
	~MB85RC() override;

	bool read_device_id(uint32_t* const out_id);

	bool sleep();
	bool wake();

};