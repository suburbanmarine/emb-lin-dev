/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * This software is distrubuted in the hope it will be useful, but without any warranty, including the implied warrranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE.txt for details.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2023 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/


#include "emb-lin-dev/max38615_spidev.hpp"

#include <spdlog/spdlog.h>

#include <linux/spi/spidev.h>

#include <thread>

bool max38165_spidev::init(const std::shared_ptr<SPIdev>& spidev, const std::shared_ptr<RTD_util>& rtd_lut, const float r_ref)
{
	m_spidev = spidev;

	m_rtd_lut = rtd_lut;
	m_r_ref   = r_ref;

	//Set upper fault to ~300C
	if ( ! set_high_fault(0x87B6U) )
	{
		SPDLOG_ERROR("Could not high set fault reg");
		return false;
	}
	//Set lower fault to ~-125C
	if ( ! set_low_fault(0x200AU) )
	{
		SPDLOG_ERROR("Could not low set fault reg");
		return false;
	}

	return true;
}

bool max38165_spidev::configure_spidev(const std::shared_ptr<SPIdev>& spidev)
{
	if( ! spidev->set_mode(SPI_MODE_1) )
	{
		SPDLOG_ERROR("Could not set mode on spidev");
		return false;
	}
	if( ! spidev->set_word_size(8) )
	{
		SPDLOG_ERROR("Could not set word size on spidev");
		return false;
	}
	if( ! spidev->set_max_speed(1000000) )
	{
		SPDLOG_ERROR("Could not set clk speed on spidev");
		return false;
	}

	return true;
}

bool max38165_spidev::enable_vbias()
{
	std::array<uint8_t, 2> out_buf = {0x80, 0x82};

	if( ! m_spidev->write(out_buf) )
	{
		SPDLOG_WARN("Could enable vbias, spi write failed");
		return false;
	}

	return true;
}

bool max38165_spidev::disable_vbias()
{
	std::array<uint8_t, 2> out_buf = {0x80, 0x00};

	if( ! m_spidev->write(out_buf) )
	{
		SPDLOG_WARN("Could disable vbias, spi write failed");
		return false;
	}

	return true;
}

bool max38165_spidev::start_oneshot()
{
	std::array<uint8_t, 2> out_buf = {0x80, 0xA2};

	if( ! m_spidev->write(out_buf) )
	{
		SPDLOG_WARN("Could start oneshot reading, spi write failed");
		return false;
	}

	return true;
}

bool max38165_spidev::set_high_fault(const uint16_t val)
{
	std::array<uint8_t, 3> out_buf = {0x83, uint8_t((val & 0xFF00U) >> 8), uint8_t(val & 0x00FFU)};

	if( ! m_spidev->write(out_buf) )
	{
		return false;
	}

	return true;
}
bool max38165_spidev::set_low_fault(const uint16_t val)
{
	std::array<uint8_t, 3> out_buf = {0x85, uint8_t((val & 0xFF00U) >> 8), uint8_t(val & 0x00FFU)};

	if( ! m_spidev->write(out_buf) )
	{
		return false;
	}

	return true;
}

bool max38165_spidev::read_resistance_reg(uint16_t* const r_reg)
{
	std::array<uint8_t, 1> out_buf = {0x01};
	std::array<uint8_t, 2> in_buf;

	if( ! m_spidev->write_then_read(out_buf, in_buf) )
	{
		SPDLOG_WARN("Could read resistance, spi xfer failed");
		return false;
	}

	if(r_reg)
	{
		*r_reg = (uint16_t(in_buf[0]) << 8) | uint16_t(in_buf[1]);
	}

	return true;
}

bool max38165_spidev::read_resistance_oneshot(float* out_res_ratio)
{
	bool ret = true;

	if( ! enable_vbias() )
	{
		return false;
	}

	//1ms + 10.5RC constants, @ 100ohm * 100nF = 1150us
	std::this_thread::sleep_for(std::chrono::microseconds(1105));

	if( ! start_oneshot() )
	{
		return false;
	}

	//typ 52 - 62.5ms to complete
	//TODO: poll DRDY line
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	uint16_t r_reg = 0;
	if( ! read_resistance_reg(&r_reg) )
	{
		ret = false;
	}

	if( ! disable_vbias() )
	{
		ret = false;
	}

	if(ret)
	{
		if(r_reg & 0x0001U)
		{
			SPDLOG_WARN("Could read resistance, sensor reports fault");
			ret = false;
		}
	}


	if(out_res_ratio)
	{
		*out_res_ratio = float(r_reg >> 1) / 32768.0f;
	}

	return ret;
}

bool max38165_spidev::get_temp_oneshot(float* out_degC)
{
	if( ! m_rtd_lut )
	{
		return false;
	}

	float r_ratio;
	if( ! read_resistance_oneshot(&r_ratio) )
	{
		return false;
	}

	const float r_measured = r_ratio * m_r_ref;

	const float degC = m_rtd_lut->calc_temp_from_ohm(r_measured);

	if(out_degC)
	{
		*out_degC = degC;
	}

	return true;
}