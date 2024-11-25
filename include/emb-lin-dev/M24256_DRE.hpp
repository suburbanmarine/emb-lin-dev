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

#include "emb-lin-dev/M24XXX_DRE_base.hpp"

#include <array>

class M24256_DRE : public M24XXX_DRE_base
{
public:
	typedef std::array<uint8_t, 128> Pagebuffer;

	M24256_DRE(const std::shared_ptr<I2C_bus_base>& bus, const long id);

protected:
	typedef std::array<uint8_t, 128+2> Writebuffer;
};