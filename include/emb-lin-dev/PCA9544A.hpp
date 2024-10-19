/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
*/

#pragma once

#include "emb-lin-dev/I2C_dev_base.hpp"

class PCA9544A_bus;

class PCA9544A : public I2C_dev_base
{
	friend class PCA9544A_bus;

public:
	PCA9544A(const std::shared_ptr<I2C_bus_base>& bus, const long id);

	// lock parent bus and this mux, open parent, select a child bus
	// release on destruction
	std::shared_ptr<PCA9544A_bus> open_bus(int8_t bus);

	bool select_none();

	std::recursive_mutex& get_mutex()
	{
		return m_mutex;
	}

protected:
	// -1 - no bus
	// 0 = bus 0
	// 1 = bus 1
	// 2 = bus 2
	// 3 = bus 3
	bool select_bus(int8_t bus);

	std::recursive_mutex m_mutex;
};

