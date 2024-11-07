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

#include "emb-lin-dev/MCP23017.hpp"

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

#include <endian.h>

MCP23017::MCP23017(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
{

}

bool MCP23017::read_reg8(const uint8_t addr, uint8_t* const out_val)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = i2c_smbus_read_byte_data(m_bus->get_fd(), addr);
	if(ret < 0)
	{
		SPDLOG_ERROR("I2C failed: {:d}", errno);
		return false;
	}

	if(out_val)
	{
		*out_val = be16toh(ret & 0xFFFFU);
	}

	return true;
}
bool MCP23017::write_reg8(const uint8_t addr, const uint8_t val)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = i2c_smbus_write_byte_data(m_bus->get_fd(), addr, val);
	if(ret < 0)
	{
		SPDLOG_ERROR("I2C failed: {:d}", errno);
	}

	return ret == 0;
}
bool MCP23017::read_reg16(const uint8_t addr, uint16_t* const out_val)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = i2c_smbus_read_word_data(m_bus->get_fd(), addr);
	if(ret < 0)
	{
		SPDLOG_ERROR("I2C failed: {:d}", errno);
		return false;
	}

	if(out_val)
	{
		*out_val = ret & 0xFFFF;
	}

	return true;
}
bool MCP23017::write_reg16(const uint8_t addr, const uint16_t val)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = i2c_smbus_write_word_data(m_bus->get_fd(), addr, val);
	if(ret < 0)
	{
		SPDLOG_ERROR("I2C failed: {:d}", errno);
	}

	return ret == 0;
}
bool MCP23017::write_verify_reg8(const uint8_t addr, const uint8_t val)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	bool fail = false;

	int32_t ret = i2c_smbus_write_byte_data(m_bus->get_fd(), addr, val);
	if(ret < 0)
	{
		SPDLOG_ERROR("I2C failed: {:d}", errno);
		fail = true;
	}

	ret = i2c_smbus_read_byte_data(m_bus->get_fd(), addr);
	if(ret < 0)
	{
		SPDLOG_ERROR("I2C failed: {:d}", errno);
		fail = true;
	}

	return (!fail) && (ret == val);
}
bool MCP23017::write_verify_reg16(const uint8_t addr, const uint16_t val)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	bool fail = false;

	int32_t ret = i2c_smbus_write_word_data(m_bus->get_fd(), addr, val);
	if(ret < 0)
	{
		SPDLOG_ERROR("I2C failed: {:d}", errno);
		fail = true;
	}

	ret = i2c_smbus_read_word_data(m_bus->get_fd(), addr);
	if(ret < 0)
	{
		SPDLOG_ERROR("I2C failed: {:d}", errno);
		fail = true;
	}

	return (!fail) && (ret == val);
}