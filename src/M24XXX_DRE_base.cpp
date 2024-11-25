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

#include "emb-lin-dev/M24XXX_DRE_base.hpp"

#include <spdlog/spdlog.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
extern "C"
{
	#include <i2c/smbus.h>
}

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <algorithm>

// density_code -> size
const std::map<uint8_t, M24XXX_DRE_base::M24XXX_DRE_Properties> M24XXX_DRE_base::DEVICE_PROPERTIES = {
	{0x08U,   {256,  16, 1}},
	{0x09U,   {512,  16, 1}},
	{0x0AU,  {1024,  16, 1}},
	{0x0BU,  {2048,  16, 1}},
	{0x0CU,  {4096,  32, 2}},
	{0x0DU,  {8192,  32, 2}},
	{0x0EU, {16384,  64, 2}},
	{0x0FU, {32768,  64, 2}},
	{0x10U, {65536, 128, 2}}
};

M24XXX_DRE_base::M24XXX_DRE_base(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
{

}

M24XXX_DRE_base::~M24XXX_DRE_base()
{

}

bool M24XXX_DRE_base::probe_eeprom(M24XXX_DRE_ID* const out_id)
{
	Device_id_code buf;
	if( ! read_id_code(&buf) )
	{
		return false;
	}

	m_probed_id = {buf[0], buf[1], buf[2]};

	if( (buf[0] == MF_CODE) && (buf[1] == FAMILY_CODE) )
	{
		const auto it = DEVICE_PROPERTIES.find(buf[2]);
		if(it != DEVICE_PROPERTIES.end())
		{
			m_probed_properties = it->second;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}

	if(out_id)
	{
		*out_id = m_probed_id.value();
	}

	return true;
}
bool M24XXX_DRE_base::read_id_code(Device_id_code* const out_buf)
{
	const M24XXX_DRE_Properties& prop = m_probed_properties.value();

	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	std::array<uint8_t, 2> addr_data;
	addr_data.fill(0);

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = prop.addr_size;
	trx[0].buf   = addr_data.data();

	trx[1].addr  = get_idpage_addr();
	trx[1].flags = I2C_M_RD;
	trx[1].len   = out_buf->size();
	trx[1].buf   = out_buf->data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	return true;
}
bool M24XXX_DRE_base::read_id_page(std::vector<uint8_t>* const out_id_page)
{
	const M24XXX_DRE_Properties& prop = m_probed_properties.value();

	out_id_page->resize(prop.page_size);

	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	std::array<uint8_t, 2> addr_data;
	addr_data.fill(0);

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = prop.addr_size;
	trx[0].buf   = addr_data.data();

	trx[1].addr  = get_idpage_addr();
	trx[1].flags = I2C_M_RD;
	trx[1].len   = out_id_page->size();
	trx[1].buf   = out_id_page->data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	return true;
}
bool M24XXX_DRE_base::write_id_page(const std::vector<uint8_t>& id_page)
{
	const M24XXX_DRE_Properties& prop = m_probed_properties.value();

	if(id_page.size() > prop.page_size)
	{
		return false;
	}

	std::vector<uint8_t> buf;
	buf.reserve(prop.addr_size + prop.page_size);

	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	for(size_t i = 0; i < prop.addr_size; i++)
	{
		buf.push_back(0);
	}
	buf.insert(buf.end(), id_page.begin(), id_page.end());

	std::array<i2c_msg, 1> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = buf.size();
	trx[0].buf   = buf.data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	if( ! wait_write_complete() )
	{
		SPDLOG_ERROR("Device write failed");
		return false;
	}

	return true;
}

bool M24XXX_DRE_base::lock_id_page()
{
	return false;
}

bool M24XXX_DRE_base::get_id_lock_status(bool* const is_locked)
{
	return false;
}

bool M24XXX_DRE_base::wait_write_complete()
{
#if 1
	std::this_thread::sleep_for(get_max_write_time());
	return true;
#else

	bool ret = false;

	if( ! m_bus->set_device_id(m_dev_addr) )
	{
		return false;
	}

	const size_t max_write_ms = get_max_write_time().count() + 1;
	std::array<uint8_t, 1> data;
	data.fill(0);

	std::array<i2c_msg, 1> trx {};

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();

	trx[0].addr  = m_dev_addr;
	trx[0].flags = 0;
	trx[0].len   = data.size();
	trx[0].buf   = data.data();

	for(size_t i = 0; i < max_write_ms; i++)
	{
		if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		else
		{
			ret = true;
			break;
		}
	}

	return ret;
#endif
}
