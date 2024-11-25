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
	{0x08U,   {256,  16, 1, 8}},
	{0x09U,   {512,  16, 1, 9}},
	{0x0AU,  {1024,  16, 1, 10}},
	{0x0BU,  {2048,  16, 1, 11}},
	{0x0CU,  {4096,  32, 2, 12}},
	{0x0DU,  {8192,  32, 2, 13}},
	{0x0EU, {16384,  64, 2, 14}},
	{0x0FU, {32768,  64, 2, 15}},
	{0x10U, {65536, 128, 2, 16}}
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

bool M24XXX_DRE_base::read(const size_t addr, void* buf, const size_t size) 
{
	const M24XXX_DRE_Properties& prop = m_probed_properties.value();

	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	SPDLOG_DEBUG("read 0x{:02X} {:d}@0x{:04X}", m_dev_addr, size, addr);

	uint8_t dev_addr_with_data_addr;
	std::array<uint8_t, 2> addr_data;
	if( ! get_io_addr(addr, &dev_addr_with_data_addr, &addr_data) )
	{
		return false;
	}

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = dev_addr_with_data_addr;
	trx[0].flags = 0;
	trx[0].len   = prop.addr_size;
	trx[0].buf   = addr_data.data();

	trx[1].addr  = dev_addr_with_data_addr;
	trx[1].flags = I2C_M_RD;
	trx[1].len   = size;
	trx[1].buf   = (unsigned char*)buf;

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
bool M24XXX_DRE_base::write(const size_t addr, const void* buf, const size_t size)
{		
	if( (addr + size) > get_size() )
	{
		return false;
	}

	SPDLOG_DEBUG("M24XXX_DRE_base::write 0x{:02X} {:d}@0x{:04X}", m_dev_addr, size, addr);

	size_t num_written = 0;
	const size_t pagesize = get_pagesize();

	// if we are not on a page boundary, unroll the first write
	if( (addr % pagesize) != 0)
	{
		const size_t base_addr    = (addr / pagesize) * pagesize;
		const size_t num_to_write = std::min(base_addr + pagesize - addr, size);

		if( ! write_page(addr, buf, num_to_write) )
		{
			return false;
		}

		num_written += num_to_write;
	}

	// write up to page at a time, on page boundaries
	while(num_written < size)
	{
		const size_t num_to_write = std::min<size_t>(size - num_written, pagesize);
		
		if( ! write_page(addr + num_written, static_cast<unsigned char const *>(buf) + num_written, num_to_write) )
		{
			return false;
		}

		num_written += num_to_write;
	}

	return true;
}

bool M24XXX_DRE_base::fill(const uint8_t val)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	const size_t size     = get_size();
	const size_t pagesize = get_pagesize();

	std::vector<uint8_t> buf(pagesize, val);

	for(size_t i = 0; i < size; i += pagesize)
	{
		if( ! write_page(i, buf.data(), pagesize) )
		{
			return false;
		}
	}

	return true;
}

bool M24XXX_DRE_base::get_io_addr(const size_t addr, uint8_t* const dev_addr_with_data_addr, std::array<uint8_t, 2>* const addr_data)
{
	const M24XXX_DRE_Properties& prop = m_probed_properties.value();

	uint8_t temp = m_dev_addr;
	switch(prop.addr_size)
	{
		case 1:
		{
			//TODO: encode A8-A10 for 4-16kbit mem in the low bits of m_dev_addr
			//use prop.addr_bits and stuff the high bits into low bits of m_dev_addr
			const uint8_t bits_to_pack = prop.addr_bits - 8;
			if(bits_to_pack != 0)
			{
				SPDLOG_ERROR("M24C04-DRE / M24C08-DRE / M24C16-DRE not supported yet");
				return false;
			}
			else
			{
				temp = m_dev_addr;
			}

			(*addr_data)[0] = (addr & 0x00FFU) >> 0;
			break;
		}
		case 2:
		{
			// we never pack addr bits in for a two byte addr
			temp = m_dev_addr;

			(*addr_data)[0] = (addr & 0xFF00U) >> 8;
			(*addr_data)[1] = (addr & 0x00FFU) >> 0;
			break;
		}
		default:
		{
			SPDLOG_ERROR("Invalid addr_size");
			return false;
		}
	}

	*dev_addr_with_data_addr = temp;

	return true;
}

bool M24XXX_DRE_base::write_page(const size_t addr, const void* buf, const size_t size)
{
	const M24XXX_DRE_Properties& prop = m_probed_properties.value();

	SPDLOG_DEBUG("M24C64_DRE::write_page 0x{:02X} {:d}@0x{:04X}", m_dev_addr, size, addr);

	const size_t page_addr = (addr / get_pagesize()) * get_pagesize();
	if( (addr + size) > (page_addr + get_pagesize()) )
	{
		SPDLOG_ERROR("M24XXX_DRE_base::write_page not alligned: {:d}@0x{:04X}", size, addr);
		return false;
	}

	if( size > get_pagesize() )
	{
		SPDLOG_ERROR("M24XXX_DRE_base::write_page size larger than get_pagesize()");
		return false;
	}

	// gather IO not supported by driver and/or FT260S
	std::vector<uint8_t> write_buf;
	write_buf.reserve(prop.addr_size + prop.page_size);

	uint8_t dev_addr_with_data_addr;
	std::array<uint8_t, 2> addr_data;
	if( ! get_io_addr(addr, &dev_addr_with_data_addr, &addr_data) )
	{
		return false;
	}

	write_buf.insert(write_buf.end(), addr_data.data(), addr_data.data() + prop.addr_size);
	write_buf.insert(write_buf.end(), static_cast<uint8_t const *>(buf), static_cast<uint8_t const *>(buf)+size);
	
	std::array<i2c_msg, 1> trx {};
	trx[0].addr  = dev_addr_with_data_addr;
	trx[0].flags = 0;
	trx[0].len   = write_buf.size();
	trx[0].buf   = write_buf.data();

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
