/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * This software is distrubuted in the hope it will be useful, but without any warranty, including the implied warrranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE.txt for details.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2023 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#include "emb-lin-dev/MCP23S17.hpp"

#include <spdlog/spdlog.h>

#include <linux/spi/spidev.h>

MCP23S17::MCP23S17()
{

}

MCP23S17::~MCP23S17()
{
	
}

bool MCP23S17::init(const std::shared_ptr<SPIdev>& spidev)
{
	m_dev = spidev;

	bool bank = false;
	if( ! probe_bank_mode(&bank) )
	{
		SPDLOG_ERROR("Failed to probe bank");
		return false;
	}

	if(bank)
	{
		SPDLOG_DEBUG("Probed bank=1, set bank=0");

		//set bank to 0
		uint8_t reg_0A;
		if( ! read_reg8((uint8_t)REG_ADDR_BANK1::IOCONA, &reg_0A) )
		{
			SPDLOG_ERROR("Failed to read reg REG_ADDR_BANK1::IOCONA");
			return false;
		}

		reg_0A &= 0x7F;
		if( ! write_verify_reg8((uint8_t)REG_ADDR_BANK1::IOCONA, reg_0A) )
		{
			SPDLOG_ERROR("Failed to write reg REG_ADDR_BANK1::IOCONA");
			return false;
		}

		bool bank2 = false;
		if( ! probe_bank_mode(&bank2) )
		{
			SPDLOG_ERROR("Failed to probe bank");
			return false;
		}

		if(bank2)
		{
			SPDLOG_ERROR("bank still true after clearing");
			return false;	
		}
	}

	// BANK   = 0
	// MIRROR = 1
	// SEQOP  = 0
	// DISSLW = 0
	// HAEN   = 0
	// ODR    = 1
	// INTPOL = 0
	// RES    = 0
	const uint8_t IOCON = 0x68;
	if( ! write_verify_reg8((uint8_t)REG_ADDR_BANK0::IOCONA, IOCON) )
	{
		SPDLOG_ERROR("Failed to write reg REG_ADDR_BANK0::IOCONA");
		return false;
	}

//	if( ! write_verify_reg16((uint8_t)REG_ADDR_BANK0::IODIRA, 0U) )
//	{
//		SPDLOG_ERROR("Failed to write reg REG_ADDR_BANK0::IODIR");
//		return false;
//	}

	if( ! write_verify_reg16((uint8_t)REG_ADDR_BANK0::IPOLA, 0U) )
	{
		SPDLOG_ERROR("Failed to write reg REG_ADDR_BANK0::IPOL");
		return false;
	}

	if( ! write_verify_reg16((uint8_t)REG_ADDR_BANK0::GPINTENA, 0U) )
	{
		SPDLOG_ERROR("Failed to write reg REG_ADDR_BANK0::GPINTEN");
		return false;
	}

	if( ! write_verify_reg16((uint8_t)REG_ADDR_BANK0::DEFVALA, 0U) )
	{
		SPDLOG_ERROR("Failed to write reg REG_ADDR_BANK0::DEFVAL");
		return false;
	}

	if( ! write_verify_reg16((uint8_t)REG_ADDR_BANK0::INTCONA, 0U) )
	{
		SPDLOG_ERROR("Failed to write reg REG_ADDR_BANK0::INTCON");
		return false;
	}

//	if( ! write_verify_reg16((uint8_t)REG_ADDR_BANK0::GPPUA, 0U) )
//	{
//		SPDLOG_ERROR("Failed to write reg REG_ADDR_BANK0::GPPU");
//		return false;
//	}

	if( ! write_verify_reg16((uint8_t)REG_ADDR_BANK0::INTCAPA, 0U) )
	{
		SPDLOG_ERROR("Failed to write reg REG_ADDR_BANK0::INTCAP");
		return false;
	}

//	if( ! write_verify_reg16((uint8_t)REG_ADDR_BANK0::GPIOA, 0U) )
//	{
//		SPDLOG_ERROR("Failed to write reg REG_ADDR_BANK0::GPIO");
//		return false;
//	}

//	if( ! write_verify_reg16((uint8_t)REG_ADDR_BANK0::OLATA, 0U) )
//	{
//		SPDLOG_ERROR("Failed to write reg REG_ADDR_BANK0::OLAT");
//		return false;
//	}

	return true;
}

bool MCP23S17::configure_spidev(const std::shared_ptr<SPIdev>& spidev)
{
	if( ! spidev->set_mode(SPI_MODE_0) )
	{
		SPDLOG_ERROR("Could not set mode on spidev");
		return false;
	}
	if( ! spidev->set_word_size(8) )
	{
		SPDLOG_ERROR("Could not set word size on spidev");
		return false;
	}
	if( ! spidev->set_max_speed(1000000) )
	{
		SPDLOG_ERROR("Could not set clk speed on spidev");
		return false;
	}

	return true;
}

bool MCP23S17::read_reg8(const uint8_t addr, uint8_t* const out_val)
{
	std::array<uint8_t, 2> out_buf = {0x41, addr};
	std::array<uint8_t, 1> in_buf;

	if( ! m_dev->write_then_read(out_buf, in_buf) )
	{
		return false;
	}

	*out_val = in_buf[0];

	return true;
}

bool MCP23S17::write_reg8(const uint8_t addr, const uint8_t val)
{
	std::array<uint8_t, 3> buf = {0x40, addr, val};
	
	if( ! m_dev->write(buf) )
	{
		return false;
	}

	return true;
}

bool MCP23S17::read_reg16(const uint8_t addr, uint16_t* const out_val)
{
	std::array<uint8_t, 2> out_buf = {0x41, addr};
	std::array<uint8_t, 2> in_buf;

	if( ! m_dev->write_then_read(out_buf, in_buf) )
	{
		return false;
	}

	*out_val = (uint16_t(in_buf[1]) << 8) | (uint16_t(in_buf[0]) << 0);

	return true;
}

bool MCP23S17::write_reg16(const uint8_t addr, const uint16_t val)
{
	std::array<uint8_t, 4> buf = {0x40, addr, uint8_t(val & 0x00FF), uint8_t((val & 0xFF00) >> 8)};
	
	if( ! m_dev->write(buf) )
	{
		return false;
	}

	return true;
}

bool MCP23S17::write_verify_reg8(const uint8_t addr, const uint8_t val)
{
	if( ! write_reg8(addr, val) )
	{
		SPDLOG_ERROR("write_reg16 failed");
		return false;
	}

	uint8_t in_val;
	if( ! read_reg8(addr, &in_val) )
	{
		SPDLOG_ERROR("read_reg16 failed");
		return false;
	}

	if(val != in_val)
	{
		SPDLOG_ERROR("Readback did not match write");
		return false;
	}

	return true;
}

bool MCP23S17::write_verify_reg16(const uint8_t addr, const uint16_t val)
{
	if( ! write_reg16(addr, val) )
	{
		SPDLOG_ERROR("write_reg16 failed");
		return false;
	}

	uint16_t in_val;
	if( ! read_reg16(addr, &in_val) )
	{
		SPDLOG_ERROR("read_reg16 failed");
		return false;
	}

	if(val != in_val)
	{
		SPDLOG_ERROR("Readback did not match write");
		return false;
	}

	return true;
}

bool MCP23S17::set_bank_0()
{
	return false;
}

bool MCP23S17::probe_bank_mode(bool* const bank)
{
	// figure out BANK setting without priors

	// if IOCON.BANK == 0
	// 0x05=GPINTENB
	// 0x15=OLATB
	// 0x0A=IOCON
	// 0x0B=IOCON

	// if IOCON.BANK == 1
	// 0x05=IOCON
	// 0x15=IOCON
	// 0x0A=OLATA
	// 0x0B=<DNE>

	//backup regs
	uint8_t reg_05;
	if( ! read_reg8(0x05, &reg_05) )
	{
		return false;
	}

	uint8_t reg_15;
	if( ! read_reg8(0x15, &reg_15) )
	{
		return false;
	}

	uint8_t reg_0A;
	if( ! read_reg8(0x0A, &reg_0A) )
	{
		return false;
	}

	uint8_t val = 0;
	if(reg_05 & 0x80U) val |= 0x01;
	if(reg_15 & 0x80U) val |= 0x02;
	if(reg_0A & 0x80U) val |= 0x04;

	//TODO evaluate the combinations, and look for contridictions to figure out what bank setting
	switch(val)
	{
		case 0x00: // 000
		{
			// bank=0, unambigous
			*bank = false;
			break;			
		}
		case 0x01: // 001
		{
			// bank=0, unambigous
			*bank = false;
			break;			
		}
		case 0x02: // 010
		{
			// bank=0, unambigous
			*bank = false;
			break;
		}
		case 0x03: // 011
		{
			// bank=1, unambigous
			*bank = true;
			break;			
		}
		case 0x04: // 100
		{
			// not expected, not legal?
			// if 0x05/0x15 are not both high, we expect 0x0A to be low
			return false;
			break;
		}
		case 0x05: // 101
		{
			// not expected, not legal?
			// if 0x05/0x15 are not both high, we expect 0x0A to be low
			return false;
			break;			
		}
		case 0x06: // 110
		{
			// not expected, not legal?
			// if 0x05/0x15 are not both high, we expect 0x0A to be low
			return false;
			break;
		}
		case 0x07: // 111
		{
			// bank=1, unambigous
			*bank = true;
			break;			
		}
		default:
		{
			// not expected, not legal?
			// states are finite [0,7]
			break;	
		}
	}

	if(bank)
	{

	}
	else
	{

	}

	return true;
}
bool MCP23S17::set_line(const unsigned int idx, const int value)
{
	if(idx > 15)
	{
		return false;
	}

	{
		std::unique_lock<std::recursive_mutex> lock(m_mutex);

		uint16_t pin_map = 0;
		if( ! get_latch(&pin_map) )
		{
			return false;
		}

		if(value)
		{
			pin_map |= (1U << idx);
		}
		else
		{
			pin_map &= ~(1U << idx);	
		}

		if( ! set_latch(pin_map) )
		{
			return false;
		}
	}

	return true;
}
bool MCP23S17::get_line(const unsigned int idx, int* const out_value)
{
	if(idx > 15)
	{
		return false;
	}

	uint16_t pin_map = 0;
	{
		std::unique_lock<std::recursive_mutex> lock(m_mutex);
		if( ! get_gpio(&pin_map) )
		{
			return false;
		}
	}

	if(out_value)
	{
		if(pin_map & (1U << idx))
		{
			*out_value = 1;
		}
		else
		{
			*out_value = 0;
		}
	}

	return true;
}
bool MCP23S17::set_all_lines(const uint64_t value)
{
	std::unique_lock<std::recursive_mutex> lock(m_mutex);

	const uint16_t pin_map = value & 0x000000000000FFFFULL;
	if( ! set_latch(pin_map) )
	{
		return false;
	}

	return true;
}
bool MCP23S17::get_all_lines(uint64_t* const out_value)
{
	uint16_t pin_map = 0;
	
	{
		std::unique_lock<std::recursive_mutex> lock(m_mutex);
		if( ! get_gpio(&pin_map) )
		{
			return false;
		}
	}

	if(out_value)
	{
		*out_value = pin_map;
	}

	return true;
}
