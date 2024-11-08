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

#include "emb-lin-dev/I2C_bus_base.hpp"

#include <memory>
#include <mutex>

class PCA9544A;

class PCA9544A_bus
{
public:
	PCA9544A_bus(PCA9544A& dev, int8_t bus_id);
	virtual ~PCA9544A_bus();
	
protected:
	std::unique_lock<std::recursive_mutex> m_bus_lock;
	std::unique_lock<std::recursive_mutex> m_mux_lock;

	PCA9544A& m_dev;

	std::shared_ptr<I2C_bus_open_close> m_bus_closer;
};