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

struct M24XXX_DRE_Properties
{
	size_t density_code; // so far, 2^density_code == size, but it could change
	size_t size;
	size_t page_size;
	size_t addr_size;
};

class M24XXX_DRE_base : public I2C_dev_base
{
public:

	// For Device ID code
	static constexpr size_t MF_CODE     = 0x20U; // ST
	static constexpr size_t FAMILY_CODE = 0xE0U; // M24 series

	// ID -> size
	static constexpr std::map<size_t, size_t> DEVICE_SIZE = {
		{0x08,   256},
		{0x09,   512},
		{0x0A,  1024},
		{0x0B,  2048},
		{0x0C,  4096},
		{0x0D,  8192},
		{0x0E, 16384},
		{0x0F, 32768},
		{0x10, 65536}
	};

	// ID -> page_size
	static constexpr std::map<size_t, size_t> PAGE_SIZE = {
		{0x08,  16},
		{0x09,  16},
		{0x0A,  16},
		{0x0B,  16},
		{0x0C,  32},
		{0x0D,  32},
		{0x0E,  64},
		{0x0F,  64},
		{0x10, 128}
	};

	// ID -> addr_size
	static constexpr std::map<size_t, size_t> ADDR_SIZE = {
		{0x08, 1},
		{0x09, 1}, // A8 sent as i2c addr b1 (b0 is R/nW bit)
		{0x0A, 1}, // A9..A8 sent as i2c addr b2..b1
		{0x0B, 1}, // A10..A8 sent as i2c addr b3..b1
		{0x0C, 2},
		{0x0D, 2},
		{0x0E, 2},
		{0x0F, 2},
		{0x10, 2}
	};

	static constexpr std::chrono::milliseconds BYTE_WRITE_TIME(4);
	static constexpr std::chrono::milliseconds PAGE_WRITE_TIME(4);

	M24XXX_DRE_base(const M24XXX_DRE_Properties& prop, const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
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
