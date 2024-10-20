/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2023 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#pragma once

#include "emb-lin-dev/I2C_bus_base.hpp"

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
