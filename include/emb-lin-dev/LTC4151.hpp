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

#include <array>

class LTC4151 : public I2C_dev_base
{
public:
	LTC4151(const std::shared_ptr<I2C_bus_base>& bus, const long id);

	bool configure();

	// 0=0mV, 1=20uV
	bool get_shunt_reg(uint16_t* const out_reg);
	bool get_shunt_v(float* const out_v)
	{
		uint16_t reg;
		if( ! get_shunt_reg(&reg) )
		{
			return false;
		}

		if(out_v)
		{
			*out_v = float(reg) * 20e-6f;
		}

		return true;
	}
	bool get_shunt_uv(int32_t* const out_uv)
	{
		uint16_t reg;
		if( ! get_shunt_reg(&reg) )
		{
			return false;
		}

		if(out_uv)
		{
			*out_uv = reg * 20;
		}

		return true;
	}

	bool get_shunt_ma(const int32_t shunt_uohm, int32_t* const out_ma)
	{
		int32_t shunt_uv;
		if( ! get_shunt_uv(&shunt_uv) )
		{
			return false;
		}

		if(out_ma)
		{
			*out_ma = shunt_uv * 1000 / shunt_uohm;
		}

		return true;
	}

	// 0=0mV, 1=25mV
	bool get_bus_reg(uint16_t* const out_reg);
	bool get_bus_v(float* const out_v)
	{
		uint16_t reg;
		if( ! get_bus_reg(&reg) )
		{
			return false;
		}

		if(out_v)
		{
			*out_v = float(reg) * 25e-3f;
		}

		return true;
	}
	bool get_bus_mv(int32_t* const out_mv)
	{
		uint16_t reg;
		if( ! get_bus_reg(&reg) )
		{
			return false;
		}

		if(out_mv)
		{
			*out_mv = reg * 25;
		}

		return true;
	}

	// 0=0mV, 1=500uV
	bool get_aux_reg(uint16_t* const out_reg);
	bool get_aux_v(float* const out_v)
	{
		uint16_t reg;
		if( ! get_aux_reg(&reg) )
		{
			return false;
		}

		if(out_v)
		{
			*out_v = float(reg) * 500e-6f;
		}

		return true;
	}

	bool get_aux_mv(int32_t* const out_mv)
	{
		uint16_t reg;
		if( ! get_aux_reg(&reg) )
		{
			return false;
		}

		if(out_mv)
		{
			*out_mv = reg * 500 / 1000;
		}

		return true;
	}

	bool read_all_adc_reg(uint16_t* const out_shunt, uint16_t* const out_bus, uint16_t* const out_aux);
	bool read_all_adc_v(uint16_t* const out_shunt, uint16_t* const out_bus, uint16_t* const out_aux)
	{
		uint16_t reg_s;
		uint16_t reg_b;
		uint16_t reg_a;
		if( ! read_all_adc_reg(&reg_s, &reg_b, &reg_a) )
		{
			return false;
		}
		if(out_shunt)
		{
			*out_shunt = float(reg_s) * 20e-6f;
		}
		if(out_bus)
		{
			*out_bus = float(reg_b) * 25e-3f;
		}
		if(out_aux)
		{
			*out_aux = float(reg_a) * 500e-6f;
		}
		return true;
	}

	enum class REG_ADDR : uint8_t
	{
		SENSE_H = 0x00,
		SENSE_L = 0x01,
		VIN_H   = 0x02,
		VIN_L   = 0x03,
		ADIN_H  = 0x04,
		ADIN_L  = 0x05,
		CONTROL = 0x06,
		RESV    = 0x07
	};
protected:

};
