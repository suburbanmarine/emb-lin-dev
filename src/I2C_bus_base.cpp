/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2023 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#include "emb-lin-dev/I2C_bus_base.hpp"

#include <spdlog/spdlog.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

I2C_bus_base::I2C_bus_base(const std::string& bus_path)
{
	m_fd = -1;
	m_bus_path = bus_path;
}
I2C_bus_base::~I2C_bus_base()
{
	close();
}

bool I2C_bus_base::open()
{
	if(m_fd >= 0)
	{
		return true;
	}
	
	int ret = ::open(m_bus_path.c_str(), O_RDWR);
	if(ret < 0)
	{
		SPDLOG_ERROR("Could not open bus {:s}, errno: {:d}", m_bus_path, errno);
		return false;
	}

	m_fd = ret;

	return true;
}
bool I2C_bus_base::close()
{
	if(m_fd < 0)
	{
		return true;
	}

	int ret = ::close(m_fd);
	if(ret != 0)
	{
		SPDLOG_WARN("Error on close bus {:s}, errno: {:d}", m_bus_path, errno);
	}

	m_fd = -1;

	return ret == 0;
}

bool I2C_bus_base::set_device_id(long id)
{
	if(ioctl(m_fd, I2C_SLAVE, id) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	return true;
}

bool I2C_bus_base::get_funcs(unsigned long* const out_funcs)
{
	if(ioctl(m_fd, I2C_FUNCS, out_funcs) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	return true;
}

int32_t I2C_bus_base::i2c_smbus_read_word_data_swapped(const uint8_t addr, const uint8_t cmd)
{
	std::array<uint8_t, 2> data_buf;

	std::array<uint8_t, 1> addr_buf;
	addr_buf[0] = cmd;

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = addr;
	trx[0].flags = 0;
	trx[0].len   = addr_buf.size();
	trx[0].buf   = addr_buf.data();

	trx[1].addr  = addr;
	trx[1].flags = I2C_M_RD;
	trx[1].len   = data_buf.size();
	trx[1].buf   = data_buf.data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();
	if(ioctl(m_fd, I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return -errno;
	}

	return (uint16_t(data_buf[0]) << 8) | (uint16_t(data_buf[1]) << 0);
}

int32_t I2C_bus_base::i2c_smbus_write_word_data_swapped(const uint8_t addr, const uint8_t cmd, const uint16_t reg)
{
	std::array<uint8_t, 2> data_buf;
	data_buf[0] = (reg & 0xFF00) >> 8;
	data_buf[1] = (reg & 0x00FF) >> 0;

	std::array<uint8_t, 1> addr_buf;
	addr_buf[0] = cmd;

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = addr;
	trx[0].flags = 0;
	trx[0].len   = addr_buf.size();
	trx[0].buf   = addr_buf.data();

	trx[1].addr  = addr;
	trx[1].flags = 0;
	trx[1].len   = data_buf.size();
	trx[1].buf   = data_buf.data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();
	if(ioctl(m_fd, I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return -errno;
	}

	return 0;
}

I2C_bus_open_close::I2C_bus_open_close(I2C_bus_base& bus) : m_bus(bus)
{
	// TODO: how to handle lock timeout, could throw if block is required, possibly after a timeout
	// m_lock = std::unique_lock<std::recursive_mutex>(m_bus.get_mutex(), std::try_to_lock);
	// m_lock = std::unique_lock<std::recursive_mutex>(m_bus.get_mutex(), std::chrono::milliseconds(20));
	m_lock = std::unique_lock<std::recursive_mutex>(m_bus.get_mutex());

	if( ! m_bus.open() )
	{
		throw std::runtime_error("Could not open bus");
	}
}
I2C_bus_open_close::~I2C_bus_open_close()
{
	if( ! m_bus.close() )
	{
		SPDLOG_ERROR("Error when closing bus");
	}
}