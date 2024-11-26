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

#include "emb-lin-dev/TCAL9539.hpp"

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

TCAL9539::TCAL9539(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
{

}
TCAL9539::~TCAL9539()
{

}

bool TCAL9539::read_input(uint16_t* const out_reg)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = i2c_smbus_read_byte_data(m_bus->get_fd(), uint8_t(CMD_CODE::IN0));
	if(ret < 0)
	{
		return false;
	}

	ret = i2c_smbus_read_byte_data(m_bus->get_fd(), uint8_t(CMD_CODE::IN1));
	if(ret < 0)
	{
		return false;
	}

	return true;
}
bool TCAL9539::write_output(const uint16_t reg)
{
	return set_reg_16(uint8_t(CMD_CODE::OUT0), uint8_t(CMD_CODE::OUT1), reg);
}
bool TCAL9539::set_pin_input(const uint16_t reg)
{
	return set_reg_16(uint8_t(CMD_CODE::CONF0), uint8_t(CMD_CODE::CONF1), reg);
}
bool TCAL9539::set_pu_pd_en(const uint16_t reg)
{
	return set_reg_16(uint8_t(CMD_CODE::PU_PD_EN0), uint8_t(CMD_CODE::PU_PD_EN1), reg);
}
bool TCAL9539::set_pu_pd(const uint16_t reg)
{
	return set_reg_16(uint8_t(CMD_CODE::PU_PD0), uint8_t(CMD_CODE::PU_PD1), reg);
}

bool TCAL9539::set_reg_16(const uint8_t a_low, const uint8_t a_high, const uint16_t reg)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = i2c_smbus_write_byte_data(m_bus->get_fd(), a_low, (reg & 0x00FFU) >> 0);
	if(ret < 0)
	{
		return false;
	}

	ret = i2c_smbus_write_byte_data(m_bus->get_fd(), a_high, (reg & 0xFF00U) >> 8);
	if(ret < 0)
	{
		return false;
	}

	return true;	
}