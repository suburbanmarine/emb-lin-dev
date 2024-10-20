/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2023 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
*/

#pragma once

#include "emb-lin-dev/gpio_base.hpp"

#include <gpiod.h>

class gpiod_base : public gpio_base
{
public:
	gpiod_base();
	~gpiod_base() override;

	virtual bool open(const char* name);
	virtual void close();

	gpiod_line* get_line_handle(const size_t idx);
	
	bool set_line(const unsigned int idx, const int value) override;
	bool get_line(const unsigned int idx, int* const out_value) override;

	bool set_all_lines(const uint64_t value) override
	{
		return false;
	}
	bool get_all_lines(uint64_t* const out_value) override
	{
		return false;
	}

	size_t get_num_lines() const override
	{
		return m_io_lines.num_lines;
	}


protected:

	gpiod_chip* m_io;
	gpiod_line_bulk m_io_lines;
};


class imx_gpio1 : public gpiod_base
{
public:

protected:
};

class imx_gpio2 : public gpiod_base
{
public:

protected:
};

class imx_gpio5 : public gpiod_base
{
public:

	// bool get_temp1_drdy(bool* const out_drdy);
	// bool get_temp2_drdy(bool* const out_drdy);

	// bool set_periph_en(const bool en);

protected:
};
