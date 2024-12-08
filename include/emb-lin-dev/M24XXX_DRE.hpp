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
#include <chrono>
#include <map>
#include <optional>
#include <vector>

#include <cstddef>
#include <cstdint>

class M24XXX_DRE : public I2C_dev_base
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

	M24XXX_DRE(const std::shared_ptr<I2C_bus_base>& bus, const long id);
	~M24XXX_DRE() override;

	static constexpr std::chrono::milliseconds get_max_write_time()
	{
		return PAGE_WRITE_TIME;
	}

	// hint addr size when probing
	// probing an unlocked device with the wrong addr_size is not safe.
	virtual bool probe(const size_t addr_size)
	{
		return probe(addr_size, nullptr);
	}

	// hint addr size when probing
	// probing an unlocked device with the wrong addr_size is not safe.
	virtual bool probe(const size_t addr_size, M24XXX_DRE_ID* const out_id);

	// hint addr size when reading id code
	// probing an unlocked device with the wrong addr_size is not safe.
	virtual bool read_id_code(const size_t addr_size, Device_id_code* const out_buf);
	
	// requires valid probe
	virtual bool read_id_code(Device_id_code* const out_buf);
	
	virtual bool read_id_page(uint8_t* const out_buf, const size_t size);
	virtual bool write_id_page(uint8_t const * const buf, const size_t size);

	bool read_id_page(std::vector<uint8_t>* const out_id_page)
	{
		const M24XXX_DRE_Properties& prop = m_probed_properties.value();

		if((out_id_page->empty()) || (out_id_page->size() > prop.page_size))
		{
			out_id_page->resize(prop.page_size);
		}

		return read_id_page(out_id_page->data(), out_id_page->size());
	}
	bool write_id_page(const std::vector<uint8_t>& id_page)
	{
		return write_id_page(id_page.data(), id_page.size());
	}
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

	size_t get_addrbits() const
	{
		return m_probed_properties.value().addr_bits;
	}

	virtual bool read(const size_t addr, uint8_t* buf, const size_t size);
    virtual bool write(const size_t addr, const uint8_t* buf, const size_t size);

	bool read(const size_t addr, void* buf, const size_t size)
	{
		return read(addr, (uint8_t*)buf, size);
	}
    bool write(const size_t addr, const void* buf, const size_t size)
    {
    	return write(addr, (uint8_t*)buf, size);
    }

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

    virtual bool fill(const uint8_t val);
    virtual bool erase()
    {
    	return fill(0xFF);
    }

protected:
	std::optional<M24XXX_DRE_ID> m_probed_id;
	std::optional<M24XXX_DRE_Properties> m_probed_properties;

	bool get_io_addr(const size_t addr, uint8_t* const dev_addr_with_data_addr, std::array<uint8_t, 2>* const addr_data);

	virtual bool write_page(const size_t addr, const void* buf, const size_t size);
	virtual bool wait_write_complete();
};
