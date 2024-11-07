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

#include <memory>
#include <mutex>

#include <cstdint>

class MCP23X17
{
public:

	virtual bool set_dir(const uint16_t pin_map);
	virtual bool set_pu(const uint16_t pin_map);
	virtual bool set_latch(const uint16_t pin_map);

	virtual bool get_dir(uint16_t* const pin_map);
	virtual bool get_pu(uint16_t* const pin_map);
	virtual bool get_latch(uint16_t* const pin_map);
	virtual bool get_gpio(uint16_t* const pin_map);

	enum class REG_ADDR_BANK0 : uint8_t
	{
		IODIRA   = 0x00,
		IODIRB   = 0x01,
		IPOLA    = 0x02,
		IPOLB    = 0x03,
		GPINTENA = 0x04,
		GPINTENB = 0x05,
		DEFVALA  = 0x06,
		DEFVALB  = 0x07,
		INTCONA  = 0x08,
		INTCONB  = 0x09,
		IOCONA   = 0x0A,
		IOCONB   = 0x0B,
		GPPUA    = 0x0C,
		GPPUB    = 0x0D,
		INTFA    = 0x0E,
		INTFB    = 0x0F,
		INTCAPA  = 0x10,
		INTCAPB  = 0x11,
		GPIOA    = 0x12,
		GPIOB    = 0x13,
		OLATA    = 0x14,
		OLATB    = 0x15
	};

	enum class REG_ADDR_BANK1 : uint8_t
	{
		IODIRA   = 0x00,
		IPOLA    = 0x01,
		GPINTENA = 0x02,
		DEFVALA  = 0x03,
		INTCONA  = 0x04,
		IOCONA   = 0x05,
		GPPUA    = 0x06,
		INTFA    = 0x07,
		INTCAPA  = 0x08,
		GPIOA    = 0x09,
		OLATA    = 0x0A,
		IODIRB   = 0x10,
		IPOLB    = 0x11,
		GPINTENB = 0x12,
		DEFVALB  = 0x13,
		INTCONB  = 0x14,
		IOCONB   = 0x15,
		GPPUB    = 0x16,
		INTFB    = 0x17,
		INTCAPB  = 0x18,
		GPIOB    = 0x19,
		OLATB    = 0x1A
	};

protected:
	class MCP32S17_regs
	{
	public:
		MCP32S17_regs()
		{
			set_default();
		}

		void set_default()
		{
			IODIR   = 0x0000;
			IPOL    = 0x0000;
			GPINTEN = 0x0000;
			DEFVAL  = 0x0000;
			INTCON  = 0x0000;
			IOCON   = 0x0000;
			GPPU    = 0x0000;
			INTF    = 0x0000;
			INTCAP  = 0x0000;
			GPIO    = 0x0000;
			OLAT    = 0x0000;
		}
		uint16_t IODIR;
		uint16_t IPOL;
		uint16_t GPINTEN;
		uint16_t DEFVAL;
		uint16_t INTCON;
		uint8_t  IOCON;
		uint16_t GPPU;
		uint16_t INTF;
		uint16_t INTCAP;
		uint16_t GPIO;
		uint16_t OLAT;
	};

	virtual bool read_reg8(const uint8_t addr, uint8_t* const out_val) = 0;
	virtual bool write_reg8(const uint8_t addr, const uint8_t val)     = 0;

	// {regB | regA}
	virtual bool read_reg16(const uint8_t addr, uint16_t* const out_val) = 0;
	virtual bool write_reg16(const uint8_t addr, const uint16_t val)     = 0;

	virtual bool write_verify_reg8(const uint8_t addr, const uint8_t val)   = 0;
	virtual bool write_verify_reg16(const uint8_t addr, const uint16_t val) = 0;

	std::recursive_mutex m_mutex;
};
