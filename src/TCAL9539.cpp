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

TCAL9539::TCAL9539(const std::shared_ptr<I2C_bus_base>& bus, const long id) : TCA9539(bus, id)
{

}
TCAL9539::~TCAL9539()
{

}

bool TCAL9539::set_pu_pd_en(const uint16_t reg)
{
	return set_reg_16(uint8_t(CMD_CODE::PU_PD_EN0), reg);
}
bool TCAL9539::set_pu_pd(const uint16_t reg)
{
	return set_reg_16(uint8_t(CMD_CODE::PU_PD0), reg);
}
