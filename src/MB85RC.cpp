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

MB85RC::MB85RC(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
{

}

MB85RC::~MB85RC()
{

}

bool MB85RC::read_device_id(uint32_t* const out_id)
{
	// i2ctransfer -a -y 0 w1@0x7C 0xA0 r3@0x7C

	std::array<uint8_t, 1> cmd;
	cmd[0] = m_dev_addr << 1U;

	std::array<uint8_t, 3> resp;

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = 0x7C;
	trx[0].flags = 0;
	trx[0].len   = cmd.size();
	trx[0].buf   = cmd.data();

	trx[1].addr  = 0x7C;
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
	trx[0].addr  = 0x7C;
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
	std::this_thread::sleep_for(std::chrono::millisecond(1));

	return true;
}