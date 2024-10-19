/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
*/

#include "emb-lin-dev/MCP23X17.hpp"

bool MCP23X17::set_dir(const uint16_t pin_map)
{
	std::unique_lock<std::recursive_mutex> lock(m_mutex);
	return write_verify_reg16((uint8_t)REG_ADDR_BANK0::IODIRA, pin_map);
}
bool MCP23X17::set_pu(const uint16_t pin_map)
{
	std::unique_lock<std::recursive_mutex> lock(m_mutex);
	return write_verify_reg16((uint8_t)REG_ADDR_BANK0::GPPUA, pin_map);
}
bool MCP23X17::set_latch(const uint16_t pin_map)
{
	std::unique_lock<std::recursive_mutex> lock(m_mutex);
	return write_verify_reg16((uint8_t)REG_ADDR_BANK0::OLATA, pin_map);
}

bool MCP23X17::get_dir(uint16_t* const pin_map)
{
	std::unique_lock<std::recursive_mutex> lock(m_mutex);
	return read_reg16((uint8_t)REG_ADDR_BANK0::IODIRA, pin_map);	
}
bool MCP23X17::get_pu(uint16_t* const pin_map)
{
	std::unique_lock<std::recursive_mutex> lock(m_mutex);
	return read_reg16((uint8_t)REG_ADDR_BANK0::GPPUA, pin_map);
}
bool MCP23X17::get_latch(uint16_t* const pin_map)
{
	std::unique_lock<std::recursive_mutex> lock(m_mutex);
	return read_reg16((uint8_t)REG_ADDR_BANK0::OLATA, pin_map);
}
bool MCP23X17::get_gpio(uint16_t* const pin_map)
{
	std::unique_lock<std::recursive_mutex> lock(m_mutex);
	return read_reg16((uint8_t)REG_ADDR_BANK0::GPIOA, pin_map);
}
