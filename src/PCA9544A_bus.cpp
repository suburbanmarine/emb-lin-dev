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

#include "emb-lin-dev/PCA9544A_bus.hpp"
#include "emb-lin-dev/PCA9544A.hpp"

#include <stdexcept>

PCA9544A_bus::PCA9544A_bus(PCA9544A& dev, int8_t bus_id) : m_dev(dev)
{
	m_bus_lock = std::unique_lock<std::recursive_mutex>(m_dev.get_bus()->get_mutex(), std::defer_lock);
	m_mux_lock = std::unique_lock<std::recursive_mutex>(m_dev.get_mutex(), std::defer_lock);

	// lock parent bus and mux
	std::lock(m_bus_lock, m_mux_lock);

	// open bus
	m_bus_closer = std::make_shared<I2C_bus_open_close>(*m_dev.get_bus());

	// select child bus
	if( ! m_dev.select_bus(bus_id) )
	{
		throw std::runtime_error("Could not select bus");
	}
}
PCA9544A_bus::~PCA9544A_bus()
{
	// release child bus
	if( ! m_dev.select_bus(-1) )
	{
		// TODO: propagate fault info
		// throw std::runtime_error("Could not release bus");
	}

	// close parent bus
	m_bus_closer.reset();

	// destruction will release locks
}