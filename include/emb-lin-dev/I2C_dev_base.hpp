#pragma once

#include "I2C_bus_base.hpp"

#include <string>
#include <memory>

class I2C_dev_base
{
public:
	I2C_dev_base(const std::shared_ptr<I2C_bus_base>& bus, const long id);
	virtual ~I2C_dev_base();

	std::shared_ptr<I2C_bus_base> get_bus()
	{
		return m_bus;
	}

protected:

	std::shared_ptr<I2C_bus_base> m_bus;
	long m_dev_addr;
};
