/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2023 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
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
