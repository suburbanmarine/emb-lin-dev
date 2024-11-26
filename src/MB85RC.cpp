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

#include "emb-lin-dev/MB85RC.hpp"

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

#include <algorithm>

const std::map<uint32_t, size_t> MB85RC::DEVICE_PROPERTIES = {
	{uint32_t(DEVICE_ID_CODE::MB85RC64TA) ,   64*1024 / 8},
	{uint32_t(DEVICE_ID_CODE::MB85RC256TY),  256*1024 / 8},
	{uint32_t(DEVICE_ID_CODE::MB85RC512TY),  512*1024 / 8},
	{uint32_t(DEVICE_ID_CODE::MB85RC1MT),   1024*1024 / 8}
};

MB85RC::MB85RC(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
{

}

MB85RC::~MB85RC()
{

}

bool MB85RC::probe()
{
	uint32_t id;
	if( ! read_device_id(&id) )
	{
		return false;
	}

	const auto it = DEVICE_PROPERTIES.find(id);
	if(it == DEVICE_PROPERTIES.end())
	{
		return false;
	}

	m_device_id = it->first;
	m_size      = it->second;

	return true;
}

bool MB85RC::read_device_id(uint32_t* const out_id)
{
	// i2ctransfer -a -y 0 w1@0x7C 0xA0 r3@0x7C

	std::array<uint8_t, 1> cmd;
	cmd[0] = m_dev_addr << 1U;

	std::array<uint8_t, 3> resp;

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = 0x7C; // ie 0xF8
	trx[0].flags = 0;
	trx[0].len   = cmd.size();
	trx[0].buf   = cmd.data();

	trx[1].addr  = 0x7C; // ie 0xF9
	trx[1].flags = I2C_M_RD;
	trx[1].len   = resp.size();
	trx[1].buf   = resp.data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();

	{
		std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

		if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
		{
			SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
			return false;
		}
	}

	if(out_id)
	{
		*out_id = 
			(uint32_t(resp[0]) << 16) | 
			(uint32_t(resp[1]) <<  8) | 
			(uint32_t(resp[2]) <<  0);
	}

	return true;
}

bool MB85RC::sleep()
{
	// i2ctransfer -a -y 0 w1@0x7C 0xA0 w0@0x43
	std::array<uint8_t, 1> cmd;
	cmd[0] = m_dev_addr << 1;

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = 0x7C; // ie 0xF8
	trx[0].flags = 0;
	trx[0].len   = cmd.size();
	trx[0].buf   = cmd.data();

	trx[1].addr  = 0x86 >> 1;
	trx[1].flags = 0;
	trx[1].len   = 0;
	trx[1].buf   = nullptr;

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();

	{
		std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

		if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
		{
			SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
			return false;
		}
	}

	return true;
}

bool MB85RC::wake()
{
	// i2ctransfer -a -y 0 w0@0x50
	std::array<i2c_msg, 1> trx {};
	trx[0].addr  = m_dev_addr;
	trx[0].flags = I2C_M_RD;
	trx[0].len   = 0;
	trx[0].buf   = nullptr;

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();

	{
		std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

		if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
		{
			SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
			return false;
		}
	}

	// wait for trec, 400-450us
	std::this_thread::sleep_for(std::chrono::milliseconds(1));

	return true;
}

bool MB85RC::read(const size_t addr, void* buf, const size_t size)
{
	SPDLOG_DEBUG("MB85RC::read 0x{:02X} {:d}@0x{:04X}", m_dev_addr, size, addr);

	std::array<uint8_t, 2> addr_data;
	addr_data[0] = (addr & 0xFF00U) >> 8;
	addr_data[1] = (addr & 0x00FFU) >> 0;

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = m_dev_addr;
	trx[0].flags = 0;
	trx[0].len   = addr_data.size();
	trx[0].buf   = addr_data.data();

	trx[1].addr  = m_dev_addr;
	trx[1].flags = I2C_M_RD;
	trx[1].len   = size;
	trx[1].buf   = (unsigned char*)buf;

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();

	{
		std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

		if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
		{
			SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
			return false;
		}
	}

	return true;
}

bool MB85RC::write(const size_t addr, const void* buf, const size_t size)
{		
	SPDLOG_DEBUG("MB85RC::write 0x{:02X} {:d}@0x{:04X}", m_dev_addr, size, addr);

	std::array<uint8_t, 2> addr_data;
	addr_data[0] = (addr & 0xFF00U) >> 8;
	addr_data[1] = (addr & 0x00FFU) >> 0;

	// gather IO not supported by several drivers and/or FT260S, so we need a temp buffer
	// std::vector<uint8_t> write_buf;
	// write_buf.reserve(prop.addr_size + prop.page_size);
	// write_buf.insert(write_buf.end(), addr_data.data(), addr_data.data() + addr_data.size());
	// write_buf.insert(write_buf.end(), buf, buf + addr_data.size());

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = m_dev_addr;
	trx[0].flags = 0;
	trx[0].len   = addr_data.size();
	trx[0].buf   = addr_data.data();

	trx[1].addr  = m_dev_addr;
	trx[1].flags = I2C_M_NOSTART;
	trx[1].len   = size;
	trx[1].buf   = (unsigned char*)buf;

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();

	{
		std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

		if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
		{
			SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
			return false;
		}
	}

	return true;
}

bool MB85RC::fill(const uint8_t val)
{
	std::array<uint8_t, 32> buf;
	buf.fill(val);

	const size_t mem_size = m_size.value();
	for(size_t i = 0; i < mem_size; i += buf.size())
	{
		const size_t num_to_write = std::min(buf.size(), mem_size - i);
		if( ! write(i, buf.data(), num_to_write) )
		{
			return false;
		}
	}

	return true;
}