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

#include <string>
#include <thread>
#include <exception>
#include <mutex>

class I2C_bus_base
{
public:
	I2C_bus_base(const std::string& bus_path);
	virtual ~I2C_bus_base();

	bool open();
	bool close();

	bool set_device_id(long id);
	bool get_funcs(unsigned long* const out_funcs);

	int get_fd() const
	{
		return m_fd;
	}

	std::recursive_mutex& get_mutex()
	{
		return m_mutex;
	}

	// read a word, but MSB first instead of smbus standard of LSB first
	// emulates the i2c_smbus_read_word_data_swapped kernel function that seems to be missing from libi2c
	int32_t i2c_smbus_read_word_data_swapped(const uint8_t addr, const uint8_t cmd);

	int32_t i2c_smbus_write_word_data_swapped(const uint8_t addr, const uint8_t cmd, const uint16_t reg);

	std::string get_path() const
	{
		return m_bus_path;
	}

protected:

	std::recursive_mutex m_mutex;

	int m_fd;
	std::string m_bus_path;
};

class I2C_bus_open_close
{
public:
	I2C_bus_open_close(I2C_bus_base& bus);
	virtual ~I2C_bus_open_close();
protected:
	I2C_bus_base& m_bus;
	std::unique_lock<std::recursive_mutex> m_lock;
};
