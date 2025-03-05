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

#include "emb-lin-dev/MCP23X17.hpp"

class MCP23017 : public I2C_dev_base, public MCP23X17
{
public:
	MCP23017(const std::shared_ptr<I2C_bus_base>& bus, const long id);

protected:
	bool read_reg8(const uint8_t addr, uint8_t* const out_val) override;
	bool write_reg8(const uint8_t addr, const uint8_t val) override;

	// {regB | regA}
	bool read_reg16(const uint8_t addr, uint16_t* const out_val) override;
	bool write_reg16(const uint8_t addr, const uint16_t val) override;

	bool write_verify_reg8(const uint8_t addr, const uint8_t val) override;
	bool write_verify_reg16(const uint8_t addr, const uint16_t val) override;
};