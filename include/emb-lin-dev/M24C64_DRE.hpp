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

#include "emb-lin-dev/M24XXX_DRE_base.hpp"

#include <array>

class M24C64_DRE : public M24XXX_DRE_base<8192, 32, 2>
{
public:

	M24C64_DRE(const std::shared_ptr<I2C_bus_base>& bus, const long id);

	bool write_id_page(const Pagebuffer& data);
	bool read_id_page(Pagebuffer* const out_buf);
	bool read_id_code(Device_id_code* const out_buf);
	bool lock_id_page();

	bool get_id_lock_status(bool* const is_locked);

	static constexpr std::chrono::milliseconds get_max_write_time()
	{
		return std::chrono::milliseconds(4);
	}

	bool read(const size_t addr, void* buf, const size_t size);
    bool write(const size_t addr, const void* buf, const size_t size);

    bool fill(const uint8_t val);
    bool erase()
    {
    	return fill(0xFF);
    }

protected:
  
	// write up to 32b that does not cross a page boundary
	bool write_page(const size_t addr, const void* buf, const size_t size);

    bool wait_write_complete();
};