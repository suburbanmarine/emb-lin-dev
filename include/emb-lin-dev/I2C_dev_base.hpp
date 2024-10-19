/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
*/

#pragma once

#include "I2C_bus_base.hpp"

#include <string>
#include <memory>

class I2C_dev_base
{
public:
	I2C_dev_base(const std::shared_ptr<I2C_bus_base>& bus, const long id);
	virtual ~I2C_dev_base();

	std::shared_ptr<I2C_bus_base> get_bus()
	{
		return m_bus;
	}

protected:

	std::shared_ptr<I2C_bus_base> m_bus;
	long m_dev_addr;
};
