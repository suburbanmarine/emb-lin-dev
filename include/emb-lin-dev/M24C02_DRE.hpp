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

#include "emb-lin-dev/M24XXX_DRE.hpp"

#include <array>

class M24C02_DRE : public M24XXX_DRE
{
public:
	typedef std::array<uint8_t, 16> Pagebuffer;

	M24C02_DRE(const std::shared_ptr<I2C_bus_base>& bus, const long id);

	[[deprecated]] bool write_id_page(const Pagebuffer& data);
	[[deprecated]] bool read_id_page(Pagebuffer* const out_buf);
	
	bool lock_id_page() override;
	bool get_id_lock_status(bool* const is_locked) override;

protected:

	typedef std::array<uint8_t, 16+1> Writebuffer;
};