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

#include "emb-lin-dev/HEP_1000.hpp"

#include <spdlog/spdlog.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
extern "C"
{
	#include <i2c/smbus.h>
}

#include <thread>

HEP_1000::HEP_1000(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
{

}
HEP_1000::~HEP_1000()
{

}

bool HEP_1000::read_mfg_info(MFG_INFO* out_val)
{
	if( ! out_val )
	{
		return false;
	}

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		SPDLOG_ERROR("set_device_id failed");
		return false;
	}

	std::array<char, 32> temp;
	std::this_thread::sleep_for(CMD_DELAY);
	int32_t ret = i2c_smbus_read_block_data(m_bus->get_fd(), uint8_t(PMBUS_CMD::MFR_ID), (uint8_t*)temp.data());
	if(ret < 0)
	{
		SPDLOG_ERROR("PMBUS_CMD::MFR_ID failed");
		return false;
	}
	out_val->id.assign(temp.data(), temp.data()+ret);

	std::this_thread::sleep_for(CMD_DELAY);
	ret = i2c_smbus_read_block_data(m_bus->get_fd(), uint8_t(PMBUS_CMD::MFR_MODEL), (uint8_t*)temp.data());
	if(ret < 0)
	{
		SPDLOG_ERROR("PMBUS_CMD::MFR_MODEL failed");
		return false;
	}
	out_val->model.assign(temp.data(), temp.data()+ret);

	std::this_thread::sleep_for(CMD_DELAY);
	ret = i2c_smbus_read_block_data(m_bus->get_fd(), uint8_t(PMBUS_CMD::MFR_REVISION), (uint8_t*)temp.data());
	if(ret < 0)
	{
		SPDLOG_ERROR("PMBUS_CMD::MFR_REVISION failed");
		return false;
	}
	out_val->revision.assign(temp.data(), temp.data()+ret);

	std::this_thread::sleep_for(CMD_DELAY);
	ret = i2c_smbus_read_block_data(m_bus->get_fd(), uint8_t(PMBUS_CMD::MFR_LOCATION), (uint8_t*)temp.data());
	if(ret < 0)
	{
		SPDLOG_ERROR("PMBUS_CMD::MFR_LOCATION failed");
		return false;
	}
	out_val->location.assign(temp.data(), temp.data()+ret);
	
	std::this_thread::sleep_for(CMD_DELAY);
	ret = i2c_smbus_read_block_data(m_bus->get_fd(), uint8_t(PMBUS_CMD::MFR_DATE), (uint8_t*)temp.data());
	if(ret < 0)
	{
		SPDLOG_ERROR("PMBUS_CMD::MFR_DATE failed");
		return false;
	}
	out_val->date.assign(temp.data(), temp.data()+ret);
	
	std::this_thread::sleep_for(CMD_DELAY);
	ret = i2c_smbus_read_block_data(m_bus->get_fd(), uint8_t(PMBUS_CMD::MFR_SERIAL), (uint8_t*)temp.data());
	if(ret < 0)
	{
		SPDLOG_ERROR("PMBUS_CMD::MFR_SERIAL failed");
		return false;
	}
	out_val->serial.assign(temp.data(), temp.data()+ret);

	return true;
}

bool HEP_1000::read_reg_u8(const uint8_t cmd, uint8_t* const reg)
{
	if( ! reg )
	{
		return false;
	}	

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	std::this_thread::sleep_for(CMD_DELAY); // TODO: use an internal timer class for this
	int32_t ret = i2c_smbus_read_byte_data(m_bus->get_fd(), cmd);
	if(ret < 0)
	{
		return false;
	}

	*reg = ret;

	return true;
}

bool HEP_1000::read_reg_u16(const uint8_t cmd, uint16_t* const reg)
{
	if( ! reg )
	{
		return false;
	}	

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	std::this_thread::sleep_for(CMD_DELAY); // TODO: use an internal timer class for this
	int32_t ret = i2c_smbus_read_word_data(m_bus->get_fd(), cmd);
	if(ret < 0)
	{
		return false;
	}

	*reg = ret;

	return true;
}

bool HEP_1000::write_reg_u8(const uint8_t cmd, const uint8_t reg)
{
	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	std::this_thread::sleep_for(CMD_DELAY); // TODO: use an internal timer class for this
	int32_t ret = i2c_smbus_write_byte_data(m_bus->get_fd(), cmd, reg);
	if(ret < 0)
	{
		return false;
	}

	return true;
}

bool HEP_1000::write_reg_u16(const uint8_t cmd, const uint16_t reg)
{
	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}
	
	std::this_thread::sleep_for(CMD_DELAY); // TODO: use an internal timer class for this
	int32_t ret = i2c_smbus_write_word_data(m_bus->get_fd(), cmd, reg);
	if(ret < 0)
	{
		return false;
	}

	return true;
}
bool HEP_1000::read_vout_cmd(float* const out_val)
{
	uint16_t temp;
	if( ! read_vout_cmd_reg(&temp) )
	{
		return false;
	}

	if( ! m_mode_cache.has_value() )
	{
		uint8_t temp_mode;
		if( ! read_vout_mode_reg(&temp_mode) )
		{
			return false;
		}

		m_mode_cache = temp_mode;
	}

	*out_val = linear16_to_float(temp, m_mode_cache.value());

	return true;
}
bool HEP_1000::read_vout_trim(float* const out_val)
{
	uint16_t temp;
	if( ! read_vout_trim_reg(&temp) )
	{
		return false;
	}

	if( ! m_mode_cache.has_value() )
	{
		uint8_t temp_mode;
		if( ! read_vout_mode_reg(&temp_mode) )
		{
			return false;
		}

		m_mode_cache = temp_mode;
	}

	*out_val = linear16_to_float(temp, m_mode_cache.value());

	return true;
}
bool HEP_1000::write_vout_trim(const float val)
{
	if( ! m_mode_cache.has_value() )
	{
		uint8_t temp_mode;
		if( ! read_vout_mode_reg(&temp_mode) )
		{
			return false;
		}

		m_mode_cache = temp_mode;
	}

	const uint16_t temp = float_to_linear16(val, m_mode_cache.value());

	return write_reg_u16(uint8_t(PMBUS_CMD::VOUT_TRIM), temp);
}

bool HEP_1000::read_vin(float* const out_val)
{
	uint16_t temp;
	if( ! read_vin_reg(&temp) )
	{
		return false;
	}

	if( ! m_mode_cache.has_value() )
	{
		uint8_t temp_mode;
		if( ! read_vout_mode_reg(&temp_mode) )
		{
			return false;
		}

		m_mode_cache = temp_mode;
	}

	*out_val = linear16_to_float(temp, m_mode_cache.value());

	return true;
}
bool HEP_1000::read_vout(float* const out_val)
{
	uint16_t temp;
	if( ! read_vout_reg(&temp) )
	{
		return false;
	}

	if( ! m_mode_cache.has_value() )
	{
		uint8_t temp_mode;
		if( ! read_vout_mode_reg(&temp_mode) )
		{
			return false;
		}

		m_mode_cache = temp_mode;
	}

	*out_val = linear16_to_float(temp, m_mode_cache.value());

	return true;
}
bool HEP_1000::read_iout(float* const out_val)
{
	uint16_t temp;
	if( ! read_iout_reg(&temp) )
	{
		return false;
	}

	*out_val = linear11_to_float(temp);

	return true;
}
bool HEP_1000::read_temp1(float* const out_val)
{
	uint16_t temp;
	if( ! read_temp1_reg(&temp) )
	{
		return false;
	}

	*out_val = linear11_to_float(temp);

	return true;
}