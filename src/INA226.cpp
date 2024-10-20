/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#include "emb-lin-dev/INA226.hpp"

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

#include <array>
#include <algorithm>

INA226::INA226(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
{

}

bool INA226::configure()
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	uint16_t reg = 0x0000U; // disable power and current calc
	int32_t ret = m_bus->i2c_smbus_write_word_data_swapped(m_dev_addr, uint8_t(REG_ADDR::CAL), reg);
	if(ret < 0)
	{
		SPDLOG_ERROR("I2C failed: {:d}", errno);
		return false;
	}

	reg = 0x0000U; // disable alerts
	ret = m_bus->i2c_smbus_write_word_data_swapped(m_dev_addr, uint8_t(REG_ADDR::MASKEN), reg);
	if(ret < 0)
	{
		SPDLOG_ERROR("I2C failed: {:d}", errno);
		return false;
	}

	reg = 0; // freerunning ADC, page read/write, stuck bus detect
	reg |= uint16_t(0x02U << 9); // AVG     - 16
	reg |= uint16_t(0x05U << 6); // VBUS CT - 2.116ms
	reg |= uint16_t(0x05U << 3); // VSH CT  - 2.116ms
	reg |= uint16_t(0x07U << 0); // MODE    - continous
	ret = m_bus->i2c_smbus_write_word_data_swapped(m_dev_addr, uint8_t(REG_ADDR::CONFIG), reg);
	if(ret < 0)
	{
		SPDLOG_ERROR("I2C failed: {:d}", errno);
		return false;
	}

	return true;
}

bool INA226::reset()
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	uint16_t reg = 0x8000U; // reset
	int32_t ret = m_bus->i2c_smbus_write_word_data_swapped(m_dev_addr, uint8_t(REG_ADDR::CONFIG), reg);
	if(ret < 0)
	{
		SPDLOG_ERROR("I2C failed: {:d}", errno);
		return false;
	}

	return true;
}

bool INA226::get_mfg_id(uint16_t* const out_id)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = m_bus->i2c_smbus_read_word_data_swapped(m_dev_addr, uint8_t(REG_ADDR::MFG_ID));
	if(ret < 0)
	{
		return false;
	}

	if(out_id)
	{
		*out_id = ret & 0xFFFFU;
	}

	return true;
}
bool INA226::get_die_id(uint16_t* const out_id)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = m_bus->i2c_smbus_read_word_data_swapped(m_dev_addr, uint8_t(REG_ADDR::DIE_ID));
	if(ret < 0)
	{
		return false;
	}

	if(out_id)
	{
		*out_id = ret & 0xFFFFU;
	}

	return true;
}

bool INA226::get_shunt_reg(uint16_t* const out_reg)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = m_bus->i2c_smbus_read_word_data_swapped(m_dev_addr, uint8_t(REG_ADDR::SHUNT_V));
	if(ret < 0)
	{
		return false;
	}

	if(out_reg)
	{
		*out_reg = ret & 0xFFFFU;
	}

	return true;	
}

bool INA226::get_bus_reg(uint16_t* const out_reg)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = m_bus->i2c_smbus_read_word_data_swapped(m_dev_addr, uint8_t(REG_ADDR::BUS_V));
	if(ret < 0)
	{
		return false;
	}

	if(out_reg)
	{
		*out_reg = ret & 0xFFFFU;
	}

	return true;	
}