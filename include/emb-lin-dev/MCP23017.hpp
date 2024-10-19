#pragma once

#include "I2C_dev_base.hpp"

#include "MCP23X17.hpp"

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