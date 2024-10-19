/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
*/

#include "emb-lin-dev/PCA9544A.hpp"
#include "emb-lin-dev/PCA9544A_bus.hpp"

#include <spdlog/spdlog.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <array>
#include <mutex>

PCA9544A::PCA9544A(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
{

}

std::shared_ptr<PCA9544A_bus> PCA9544A::open_bus(int8_t bus)
{
	return std::make_shared<PCA9544A_bus>(*this, bus);
}

bool PCA9544A::select_none()
{
	std::unique_lock<std::recursive_mutex> bus_lock = std::unique_lock<std::recursive_mutex>(get_bus()->get_mutex(), std::defer_lock);
	std::unique_lock<std::recursive_mutex> mux_lock = std::unique_lock<std::recursive_mutex>(get_mutex(), std::defer_lock);

	// lock parent bus and mux
	std::lock(bus_lock, mux_lock);

	return select_bus(-1);
}

bool PCA9544A::select_bus(int8_t bus)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	std::array<uint8_t, 1> cmd;
	switch(bus)
	{
		case -1:
		{
			cmd[0] = 0x00;
			break;
		}
		case 0:
		case 1:
		case 2:
		case 3:
		{
			cmd[0] = 0x04 | uint8_t(bus);
			break;
		}
		default:
		{
			return false;
		}
	}

	std::array<i2c_msg, 1> trx {};
	trx[0].addr  = m_dev_addr;
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

	return true;
}
