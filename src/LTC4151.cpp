/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
*/

#include "emb-lin-dev/LTC4151.hpp"

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

LTC4151::LTC4151(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
{

}

bool LTC4151::configure()
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	const uint8_t reg = 0xC0; // freerunning ADC, page read/write, stuck bus detect
	int32_t ret = i2c_smbus_write_byte_data(m_bus->get_fd(), uint8_t(REG_ADDR::CONTROL), reg);
	if(ret < 0)
	{
		SPDLOG_ERROR("I2C failed: {:d}", errno);
		return false;
	}

	return true;
}

bool LTC4151::get_shunt_reg(uint16_t* const out_reg)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = m_bus->i2c_smbus_read_word_data_swapped(m_dev_addr, uint8_t(REG_ADDR::SENSE_H));
	if(ret < 0)
	{
		return false;
	}

	if(out_reg)
	{
		*out_reg = (ret & 0xFFFFU) >> 4;
	}

	return true;
}

bool LTC4151::get_bus_reg(uint16_t* const out_reg)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = m_bus->i2c_smbus_read_word_data_swapped(m_dev_addr, uint8_t(REG_ADDR::VIN_H));
	if(ret < 0)
	{
		return false;
	}

	if(out_reg)
	{
		*out_reg = (ret & 0xFFFFU) >> 4;
	}

	return true;
}

bool LTC4151::get_aux_reg(uint16_t* const out_reg)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	int32_t ret = m_bus->i2c_smbus_read_word_data_swapped(m_dev_addr, uint8_t(REG_ADDR::ADIN_H));
	if(ret < 0)
	{
		return false;
	}

	if(out_reg)
	{
		*out_reg = (ret & 0xFFFFU) >> 4;
	}

	return true;
}

bool LTC4151::read_all_adc_reg(uint16_t* const out_shunt, uint16_t* const out_bus, uint16_t* const out_aux)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	std::array<uint8_t, 6> adc_buf;

	std::array<uint8_t, 1> addr_buf;
	addr_buf[0] = uint8_t(REG_ADDR::SENSE_H);

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = m_dev_addr;
	trx[0].flags = 0;
	trx[0].len   = addr_buf.size();
	trx[0].buf   = addr_buf.data();

	trx[1].addr  = m_dev_addr;
	trx[1].flags = I2C_M_RD;
	trx[1].len   = adc_buf.size();
	trx[1].buf   = adc_buf.data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	if(out_shunt)
	{
		*out_shunt = uint16_t(adc_buf[0]) << 4 | uint16_t(adc_buf[1]) >> 4;
	}
	if(out_bus)
	{
		*out_bus = uint16_t(adc_buf[2]) << 4 | uint16_t(adc_buf[3]) >> 4;
	}
	if(out_aux)
	{
		*out_aux = uint16_t(adc_buf[4]) << 4 | uint16_t(adc_buf[5]) >> 4;
	}

	return true;
}