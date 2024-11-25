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

#include "emb-lin-dev/M24C64_DRE.hpp"

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

M24C64_DRE::M24C64_DRE(const std::shared_ptr<I2C_bus_base>& bus, const long id) : M24XXX_DRE_base(bus, id)
{

}

bool M24C64_DRE::write_id_page(const Pagebuffer& data)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	Writebuffer buf;
	buf[0] = 0x00;
	buf[1] = 0x00;
	std::copy_n(data.data(), data.size(), buf.data()+get_addrsize());

	std::array<i2c_msg, 1> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = buf.size();
	trx[0].buf   = buf.data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	if( ! wait_write_complete() )
	{
		SPDLOG_ERROR("Device write failed");
		return false;
	}

	return true;
}
bool M24C64_DRE::read_id_page(Pagebuffer* const out_buf)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	std::array<uint8_t, 2> addr_data = {{0x00, 0x00}};

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = addr_data.size();
	trx[0].buf   = addr_data.data();

	trx[1].addr  = get_idpage_addr();
	trx[1].flags = I2C_M_RD;
	trx[1].len   = out_buf->size();
	trx[1].buf   = out_buf->data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	return true;
}
bool M24C64_DRE::read_id_code(Device_id_code* const out_buf)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	std::array<uint8_t, 2> addr_data = {{0x00, 0x00}};

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = addr_data.size();
	trx[0].buf   = addr_data.data();

	trx[1].addr  = get_idpage_addr();
	trx[1].flags = I2C_M_RD;
	trx[1].len   = out_buf->size();
	trx[1].buf   = out_buf->data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	return true;
}
bool M24C64_DRE::lock_id_page()
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	std::array<uint8_t, 3> cmd;
	cmd[0] = 0x04; // A10 set
	cmd[1] = 0x00;
	cmd[2] = 0x02; // lock command

	std::array<i2c_msg, 1> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = cmd.size();
	trx[0].buf   = cmd.data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	if( ! wait_write_complete() )
	{
		SPDLOG_ERROR("Device write failed");
		return false;
	}

	return true;
}
bool M24C64_DRE::get_id_lock_status(bool* const is_locked)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	std::array<uint8_t, 3> cmd;
	cmd[0] = 0x04; // A10 set
	cmd[1] = 0x00;
	cmd[2] = 0x00;

	// probe lock bit, then reset
	// it would be better to do this as a combined operation right after the NACK, and just be a S/P toggle pair with no data
	// but linux does not let us express that well
	// 0 len buffers are rejected by the driver
	// maybe should bit bang it...
	std::array<i2c_msg, 1> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = cmd.size();
	trx[0].buf   = cmd.data();
	// trx[1].addr  = get_idpage_addr();
	// trx[1].flags = I2C_M_RD;
	// trx[1].len   = 0;
	// trx[1].buf   = nullptr;

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();
	int ret = ioctl(m_bus->get_fd(), I2C_RDWR, &idat);
	if(ret < 0)
	{
		// TODO: tell missing eeprom from locked eeprom, check errno?
		if(is_locked)
		{
			*is_locked = true;
		}
	}
	else
	{
		if(is_locked)
		{
			*is_locked = false;
		}
	}

	// reset
	// it would be better to do this as a combined operation right after the NACK, and just be a S/P toggle pair with no data
	// but linux does not let us express that well
	// 0 len buffers are rejected by the driver
	// maybe should bit bang it...
	// reading seems safe enough. The S bit resets the state machine.
	cmd.fill(0x00);
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = I2C_M_RD;
	trx[0].len   = 1;
	trx[0].buf   = cmd.data();
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_WARN("Error when resetting M24C64_DRE");
	}

	return true;
}
