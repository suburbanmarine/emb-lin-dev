/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
*/

#include "emb-lin-dev/I2C_dev_base.hpp"

#include <spdlog/spdlog.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>

I2C_dev_base::I2C_dev_base(const std::shared_ptr<I2C_bus_base>& bus, const long id)
{
	m_bus      = bus;
	m_dev_addr = id;
}
I2C_dev_base::~I2C_dev_base()
{
	
}
