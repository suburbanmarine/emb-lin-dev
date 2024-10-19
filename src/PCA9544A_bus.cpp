/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
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
		throw std::runtime_error("Could not release bus");
	}

	// close parent bus
	m_bus_closer.reset();

	// destruction will release locks
}