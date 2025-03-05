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
#include <map>

#include <array>
#include <optional>
#include <map>
#include <chrono>
#include <vector>

#include <cstddef>
#include <cstdint>

class MB85RC : public I2C_dev_base
{
public:
	// NXP says 12bit mf, 9bit part id, 3bit die rev in doc UM10204
	// ramXeed says 12bit mf, 4bit density, 8bit proprietary use
	enum class DEVICE_ID_CODE : uint32_t
	{
		MB85RC64TA  = 0x00A358U,
		MB85RC256TY = 0x00A498U,
		MB85RC512TY = 0x00A598U,
		MB85RC1MT   = 0x00A758U,
	};

	// device id -> size
	static const std::map<uint32_t, size_t> DEVICE_PROPERTIES;

	MB85RC(const std::shared_ptr<I2C_bus_base>& bus, const long id);
	~MB85RC() override;

	bool probe();
	size_t get_size() const
	{
		return m_size.value();
	}

	bool read_device_id(uint32_t* const out_id);

	bool sleep();
	bool wake();

	template<size_t LEN>
	bool write(const size_t addr, const std::array<uint8_t, LEN>& buf)
	{
		return write(addr, buf.data(), buf.size());
	}

	template<size_t LEN>
	bool read(const size_t addr, std::array<uint8_t, LEN>* const out_buf)
	{
		return read(addr, out_buf->data(), out_buf->size());
	}

	bool write(const size_t addr, const std::vector<uint8_t>& buf)
	{
		return write(addr, buf.data(), buf.size());
	}

	bool read(const size_t addr, std::vector<uint8_t>* const out_buf)
	{
		return read(addr, out_buf->data(), out_buf->size());
	}

	virtual bool read(const size_t addr, void* buf, const size_t size);
	virtual bool write(const size_t addr, const void* buf, const size_t size);

    virtual bool fill(const uint8_t val);
    virtual bool erase()
    {
    	return fill(0xFF);
    }

protected:
	std::optional<uint32_t> m_device_id;
	std::optional<size_t>   m_size;
};