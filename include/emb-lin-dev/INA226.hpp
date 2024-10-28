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

#pragma once

#include "emb-lin-dev/I2C_dev_base.hpp"

class INA226 : public I2C_dev_base
{
public:
	INA226(const std::shared_ptr<I2C_bus_base>& bus, const long id);

	bool configure();
	bool reset();

	bool get_mfg_id(uint16_t* const out_id);
	bool get_die_id(uint16_t* const out_id);

	// 0=0mV, 1=2.5uV
	bool get_shunt_reg(uint16_t* const out_reg);
	bool get_shunt_uv(int32_t* const out_uv)
	{
		uint16_t reg;
		if( ! get_shunt_reg(&reg) )
		{
			return false;
		}

		if(out_uv)
		{
			*out_uv = reg * 25 / 10;
		}

		return true;
	}
	bool get_shunt_ma(const int32_t shunt_uohm, int32_t* const out_ma)
	{
		uint16_t reg;
		if( ! get_shunt_reg(&reg) )
		{
			return false;
		}

		if(out_ma)
		{
			*out_ma = reg * 25 * 100 / shunt_uohm;
		}

		return true;
	}

	// 0=0mV, 1=1.25mV
	bool get_bus_reg(uint16_t* const out_reg);
	bool get_bus_mv(int32_t* const out_mv)
	{
		uint16_t reg;
		if( ! get_bus_reg(&reg) )
		{
			return false;
		}

		if(out_mv)
		{
			*out_mv = reg * 1250 / 1000;
		}

		return true;
	}

protected:

	enum class AVG : uint8_t
	{
		AVG_1    = 0x00,
		AVG_4    = 0x01,
		AVG_16   = 0x02,
		AVG_64   = 0x03,
		AVG_128  = 0x04,
		AVG_256  = 0x05,
		AVG_512  = 0x06,
		AVG_1024 = 0x07
	};

	enum class CONV_TIME : uint8_t
	{
		us140 = 0x00,
		us204 = 0x01,
		us332 = 0x02,
		us588 = 0x03,
		us1100 = 0x04,
		us2116 = 0x05,
		us4156 = 0x06,
		us8244 = 0x07
	};

	enum class MODE : uint8_t
	{
		SHUTDOWN   = 0x00,
		TRIG_SHUNT = 0x01,
		TRIG_BUS   = 0x02,
		TRIG_ALL   = 0x03,
		CONT_SHUNT = 0x05,
		CONT_BUS   = 0x06,
		CONT_ALL   = 0x07
	};

	enum class REG_ADDR : uint8_t
	{
		CONFIG   = 0x00,
		SHUNT_V  = 0x01,
		BUS_V    = 0x02,
		POWER    = 0x03,
		CURRENT  = 0x04,
		CAL      = 0x05,
		MASKEN   = 0x06,
		ALERT    = 0x07,
		MFG_ID   = 0xFE,
		DIE_ID   = 0xFF
	};
	
};