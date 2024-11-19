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

#include "emb-lin-dev/I2C_dev_base.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

template<size_t SIZE, size_t PAGE_SIZE, size_t ADDR_SIZE>
class M24XXX_DRE_base : public I2C_dev_base
{
public:
	typedef std::array<uint8_t, PAGE_SIZE> Pagebuffer;
	typedef std::array<uint8_t, 3> Device_id_code;

	M24XXX_DRE_base(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
	{

	}

	long get_idpage_addr() const
	{
		return (unsigned(m_dev_addr) & 0x07U) | 0x58U;
	}

	static constexpr size_t get_size()
	{
		return SIZE;
	}

	static constexpr size_t get_pagesize()
	{
		return PAGE_SIZE;
	}

	static constexpr size_t get_addrsize()
	{
		return ADDR_SIZE;
	}
    
protected:
	typedef std::array<uint8_t, PAGE_SIZE + ADDR_SIZE> Writebuffer;
};
