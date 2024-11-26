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

#include "emb-lin-dev/TCA9539.hpp"

#include <spdlog/spdlog.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
extern "C"
{
	#include <i2c/smbus.h>
}

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>

TCA9539::TCA9539(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
{

}
TCA9539::~TCA9539()
{

}

bool TCA9539::read_input(uint16_t* const out_reg)
{
	return get_reg_16(uint8_t(CMD_CODE::IN0), out_reg);
}
bool TCA9539::write_output(const uint16_t reg)
{
	return set_reg_16(uint8_t(CMD_CODE::OUT0), reg);
}
bool TCA9539::set_pin_input(const uint16_t reg)
{
	return set_reg_16(uint8_t(CMD_CODE::CONF0), reg);
}

bool TCA9539::set_line(const unsigned int idx, const int value)
{
	if(idx > get_num_lines())
	{
		return false;
	}

	uint16_t reg;
	if( ! read_input(&reg) )
	{
		return false;
	}

	if(value)
	{
		reg |= (1U << idx);
	}
	else
	{
		reg &= ~(1U << idx);
	}

	return write_output(reg);	
}
bool TCA9539::get_line(const unsigned int idx, int* const out_value)
{
	if(idx > get_num_lines())
	{
		return false;
	}

	uint16_t reg;
	if( ! read_input(&reg) )
	{
		return false;
	}

	*out_value = (reg & (1U << idx)) ? (1) : (0);

	return true;
}

bool TCA9539::set_all_lines(const uint64_t value)
{
	return write_output(value & 0xFFFFU);
}
bool TCA9539::get_all_lines(uint64_t* const out_value)
{
	uint16_t reg;
	if( ! read_input(&reg) )
	{
		return false;
	}

	if(out_value)
	{
		*out_value = reg;
	}

	return true;
}

bool TCA9539::set_reg_16(const uint8_t a_low, const uint16_t reg)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = i2c_smbus_write_word_data(m_bus->get_fd(), a_low, reg);
	if(ret < 0)
	{
		return false;
	}

	return true;	
}

bool TCA9539::get_reg_16(const uint8_t a_low, uint16_t * const out_reg)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = i2c_smbus_read_word_data(m_bus->get_fd(), a_low);
	if(ret < 0)
	{
		return false;
	}

	if(out_reg)
	{
		*out_reg = ret;
	}

	return true;	
}