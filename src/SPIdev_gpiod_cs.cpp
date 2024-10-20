/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2023 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
*/

#include "emb-lin-dev/SPIdev_gpiod_cs.hpp"

#include <spdlog/spdlog.h>

#include <thread>

SPIdev_gpiod_cs::SPIdev_gpiod_cs(const std::shared_ptr<gpiod_base>& gpio, gpiod_line* const line)
{
	m_gpio = gpio;
	m_line = line;
}
SPIdev_gpiod_cs::~SPIdev_gpiod_cs()
{
	if(m_line)
	{
		gpiod_line_release(m_line);
		m_line = nullptr;
	}

	m_gpio.reset();
}
bool SPIdev_gpiod_cs::init()
{
	if(  ! m_gpio )
	{
		SPDLOG_ERROR("m_gpio is null");
		return false;
	}

	if(  ! m_line )
	{
		SPDLOG_ERROR("m_line is null");
		return false;
	}

	gpiod_line_request_config cfg;
	memset(&cfg, 0, sizeof(cfg));

	cfg.consumer     = "SpiDev nCS";
	cfg.request_type = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;
	cfg.flags        = 0;

	int ret = gpiod_line_request(m_line, &cfg, 1);

	if(ret != 0)
	{
		SPDLOG_ERROR("gpio request failed");
		return false;
	}

	return true;
}
bool SPIdev_gpiod_cs::assert_cs()
{
	if( ! m_line )
	{
		return false;
	}

	int ret = gpiod_line_set_value(m_line, 0);
	if(ret != 0)
	{
		return false;
	}

	std::this_thread::sleep_for(std::chrono::microseconds(10));

	return true;
}
bool SPIdev_gpiod_cs::release_cs()
{
	if( ! m_line )
	{
		return false;
	}

	int ret = gpiod_line_set_value(m_line, 1);
	if(ret != 0)
	{
		return false;
	}

	std::this_thread::sleep_for(std::chrono::microseconds(10));

	return true;
}
