#pragma once

#include "emb-lin-dev/SPIdev.hpp"
#include "gpiod_base.hpp"

#include <gpiod.h>

class SPIdev_gpiod_cs : public SPIdev_cs
{
public:
	SPIdev_gpiod_cs(const std::shared_ptr<gpiod_base>& gpio, gpiod_line* const line);
	~SPIdev_gpiod_cs() override;

	bool init() override;
	bool assert_cs() override;
	bool release_cs() override;

protected:
	std::shared_ptr<gpiod_base> m_gpio;

	gpiod_line* m_line;
};
