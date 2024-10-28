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
#include "gpiod_base.hpp"

#include <gpiod.h>

class SPIdev_gpiod_cs : public SPIdev_cs
{
public:
	SPIdev_gpiod_cs(const std::shared_ptr<gpiod_base>& gpio, gpiod_line* const line);
	~SPIdev_gpiod_cs() override;

	bool init() override;
	bool assert_cs() override;
	bool release_cs() override;

protected:
	std::shared_ptr<gpiod_base> m_gpio;

	gpiod_line* m_line;
};
