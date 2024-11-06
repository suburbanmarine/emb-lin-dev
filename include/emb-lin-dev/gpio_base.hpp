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

#include <cstddef>
#include <cstdint>

class gpio_base
{
public:
	gpio_base()
	{

	}
	virtual ~gpio_base()
	{

	}

	virtual bool set_line(const unsigned int idx, const int value) = 0;
	virtual bool get_line(const unsigned int idx, int* const out_value) = 0;

	virtual bool set_all_lines(const uint64_t value) = 0;
	virtual bool get_all_lines(uint64_t* const out_value) = 0;

	virtual size_t get_num_lines() const = 0;

protected:

};
