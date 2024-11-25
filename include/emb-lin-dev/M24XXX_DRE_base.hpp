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
#include <chrono>
#include <optional>
#include <vector>

#include <cstddef>
#include <cstdint>


class M24XXX_DRE_base : public I2C_dev_base
{
public:
	struct M24XXX_DRE_ID
	{
		uint8_t mf_code;
		uint8_t fam_code;
		uint8_t density_code; // so far, 2^density_code == size, but it could change
	};

	struct M24XXX_DRE_Properties
	{
		size_t  size;
		uint8_t page_size;
		uint8_t addr_size;
		uint8_t addr_bits;
	};

	typedef std::array<uint8_t, 3> Device_id_code;

	// For Device ID code
	static constexpr size_t MF_CODE     = 0x20U; // ST
	static constexpr size_t FAMILY_CODE = 0xE0U; // M24 series

	// density_code -> size/page_size/addr_size
	static const std::map<uint8_t, M24XXX_DRE_Properties> DEVICE_PROPERTIES;

	static constexpr std::chrono::milliseconds BYTE_WRITE_TIME{4};
	static constexpr std::chrono::milliseconds PAGE_WRITE_TIME{4};

	M24XXX_DRE_base(const std::shared_ptr<I2C_bus_base>& bus, const long id);
	~M24XXX_DRE_base() override;

	static constexpr std::chrono::milliseconds get_max_write_time()
	{
		return PAGE_WRITE_TIME;
	}

	virtual bool probe_eeprom(M24XXX_DRE_ID* const out_id);
	virtual bool read_id_code(Device_id_code* const out_buf);
	virtual bool read_id_page(std::vector<uint8_t>* const out_id_page);
	virtual bool write_id_page(const std::vector<uint8_t>& id_page);
	virtual bool lock_id_page();
	virtual bool get_id_lock_status(bool* const is_locked);

	long get_idpage_addr() const
	{
		return (unsigned(m_dev_addr) & 0x07U) | 0x58U;
	}

	size_t get_size() const
	{
		return m_probed_properties.value().size;
	}

	size_t get_pagesize() const
	{
		return m_probed_properties.value().page_size;
	}

	size_t get_addrsize() const
	{
		return m_probed_properties.value().addr_size;
	}
    
	virtual bool read(const size_t addr, void* buf, const size_t size) = 0;
    virtual bool write(const size_t addr, const void* buf, const size_t size) = 0;

    virtual bool fill(const uint8_t val);
    virtual bool erase()
    {
    	return fill(0xFF);
    }

protected:
	std::optional<M24XXX_DRE_ID> m_probed_id;
	std::optional<M24XXX_DRE_Properties> m_probed_properties;

	virtual bool write_page(const size_t addr, const void* buf, const size_t size);
	virtual bool wait_write_complete();
};
