/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2023 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#pragma once

#include "emb-lin-dev/SPIdev.hpp"
#include "emb-lin-dev/gpio_base.hpp"
#include "emb-lin-dev/MCP23X17.hpp"

#include <memory>
#include <mutex>

#include <cstdint>

class MCP23S17 : public gpio_base, public MCP23X17
{
public:
	MCP23S17();
	~MCP23S17() override;

	// Not MT-safe
	// use existing spidev without configuring it
	virtual bool init(const std::shared_ptr<SPIdev>& spidev);

	// Not MT-safe
	static bool configure_spidev(const std::shared_ptr<SPIdev>& spidev);

	// gpio_base compat
	// MT-safe
	bool set_line(const unsigned int idx, int value) override;
	// MT-safe
	bool get_line(const unsigned int idx, int* const out_value) override;
	// MT-safe
	bool set_all_lines(const uint64_t value) override;
	// MT-safe
	bool get_all_lines(uint64_t* const out_value) override;

	size_t get_num_lines() const override
	{
		return 16;
	}

protected:

	bool read_reg8(const uint8_t addr, uint8_t* const out_val) override;
	bool write_reg8(const uint8_t addr, const uint8_t val) override;

	// {regB | regA}
	bool read_reg16(const uint8_t addr, uint16_t* const out_val) override;
	bool write_reg16(const uint8_t addr, const uint16_t val) override;

	bool write_verify_reg8(const uint8_t addr, const uint8_t val) override;
	bool write_verify_reg16(const uint8_t addr, const uint16_t val) override;

	bool probe_bank_mode(bool* const bank);

	bool set_bank_0();

	std::shared_ptr<SPIdev> m_dev;
};
