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

#include "emb-lin-dev/M24XXX_DRE.hpp"

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

// density_code -> [size, pagesize, addrsize, addrbits]
const std::map<uint8_t, M24XXX_DRE::M24XXX_DRE_Properties> M24XXX_DRE::DEVICE_PROPERTIES = {
	{0x08U,   {256,  16, 1, 8}},  // M24C02-DRE
	{0x09U,   {512,  16, 1, 9}},  // M24C04-DRE
	{0x0AU,  {1024,  16, 1, 10}}, // M24C08-DRE
	{0x0BU,  {2048,  16, 1, 11}}, // M24C16-DRE
	{0x0CU,  {4096,  32, 2, 12}}, // M24C32-DRE
	{0x0DU,  {8192,  32, 2, 13}}, // M24C64-DRE
	{0x0EU, {16384,  64, 2, 14}}, // M24128-DRE
	{0x0FU, {32768,  64, 2, 15}}, // M24256-DRE
	{0x10U, {65536, 128, 2, 16}}  // M24512-DRE
};

M24XXX_DRE::M24XXX_DRE(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
{

}

M24XXX_DRE::~M24XXX_DRE()
{

}

bool M24XXX_DRE::probe(const size_t addr_size, M24XXX_DRE_ID* const out_id)
{
	m_probed_id.reset();
	m_probed_properties.reset();

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
			// Unknown density code
			return false;
		}
	}
	else
	{
		// MF and i2c fam code does not match
		// Not a M24 DRE series eeprom
		return false;
	}

	if(out_id)
	{
		*out_id = m_probed_id.value();
	}

	return true;
}

bool M24XXX_DRE::force_probe(const M24XXX_DRE_ID& id)
{
	m_probed_id = id;
	m_probed_properties.reset();

	if( (id.mf_code == MF_CODE) && (id.fam_code == FAMILY_CODE) )
	{
		const auto it = DEVICE_PROPERTIES.find(id.density_code);
		if(it != DEVICE_PROPERTIES.end())
		{
			m_probed_properties = it->second;
		}
		else
		{
			// Unknown density code
			return false;
		}
	}
	else
	{
		// MF and i2c fam code does not match
		// Not a M24 DRE series eeprom
		return false;
	}

	return true;
}

bool M24XXX_DRE::read_id_code(const size_t addr_size, Device_id_code* const out_buf)
{
	const M24XXX_DRE_Properties& prop = m_probed_properties.value();

	std::array<uint8_t, 2> addr_data;
	addr_data.fill(0);

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = addr_size;
	trx[0].buf   = addr_data.data();

	trx[1].addr  = get_idpage_addr();
	trx[1].flags = I2C_M_RD;
	trx[1].len   = out_buf->size();
	trx[1].buf   = out_buf->data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();

	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);
	
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	return true;
}

bool M24XXX_DRE::read_id_code(Device_id_code* const out_buf)
{
	const M24XXX_DRE_Properties& prop = m_probed_properties.value();

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

	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);
	
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	return true;
}
bool M24XXX_DRE::read_id_page(uint8_t* const out_buf, const size_t size)
{
	const M24XXX_DRE_Properties& prop = m_probed_properties.value();

	if(size > prop.page_size)
	{
		return false;
	}

	std::array<uint8_t, 2> addr_data = {0, 0};

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = prop.addr_size;
	trx[0].buf   = addr_data.data();

	trx[1].addr  = get_idpage_addr();
	trx[1].flags = I2C_M_RD;
	trx[1].len   = size;
	trx[1].buf   = out_buf;

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();

	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);
	
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	return true;
}
bool M24XXX_DRE::write_id_page(uint8_t const * const buf, const size_t size)
{
	const M24XXX_DRE_Properties& prop = m_probed_properties.value();

	if(size > prop.page_size)
	{
		return false;
	}

	uint8_t dev_addr_with_data_addr;
	std::array<uint8_t, 2> addr_data;
	if( ! get_io_addr(0, &dev_addr_with_data_addr, &addr_data) )
	{
		return false;
	}

	std::vector<uint8_t> i2cbuf;
	i2cbuf.reserve(prop.addr_size + prop.page_size);
	i2cbuf.insert(i2cbuf.end(), addr_data.data(), addr_data.data() + prop.addr_size);
	i2cbuf.insert(i2cbuf.end(), buf, buf + size);

	std::array<i2c_msg, 1> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = i2cbuf.size();
	trx[0].buf   = i2cbuf.data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();

	{
		std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);
		
		if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
		{
			SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
			return false;
		}
	}

	if( ! wait_write_complete() )
	{
		SPDLOG_ERROR("Device write failed");
		return false;
	}

	return true;
}

bool M24XXX_DRE::lock_id_page()
{
	return false;
}

bool M24XXX_DRE::get_id_lock_status(bool* const is_locked)
{
	return false;
	
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	std::array<uint8_t, 1> cmd;
	cmd[0] = 0x00;

	// probe lock bit, then reset
	// it would be better to do this as a combined operation right after the NACK, and just be a S/P toggle pair with no data
	// but linux does not let us express that well
	// 0 len buffers are rejected by the driver
	// maybe should bit bang it...
	std::array<i2c_msg, 1> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = cmd.size();
	trx[0].buf   = cmd.data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();
	int ret = ioctl(m_bus->get_fd(), I2C_RDWR, &idat);
	if(ret < 0)
	{
		// TODO: tell missing eeprom from locked eeprom, check errno?
		if(is_locked)
		{
			*is_locked = true;
		}
	}
	else
	{
		if(is_locked)
		{
			*is_locked = false;
		}
	}

	// reset
	// it would be better to do this as a combined operation right after the NACK, and just be a S/P toggle pair with no data
	// but linux does not let us express that well
	// 0 len buffers are rejected by the driver
	// maybe should bit bang it...
	// reading seems safe enough. The S bit resets the state machine.
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = I2C_M_RD;
	trx[0].len   = cmd.size();
	trx[0].buf   = cmd.data();
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_WARN("Error when resetting M24C64_DRE");
	}

	return true;
}

bool M24XXX_DRE::read(const size_t addr, uint8_t* buf, const size_t size) 
{
	if( (addr + size) > get_size() )
	{
		SPDLOG_ERROR("size is too large");
		return false;
	}

	const M24XXX_DRE_Properties& prop = m_probed_properties.value();

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
	trx[1].buf   = buf;

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();

	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);
	
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
		return false;
	}

	return true;
}
bool M24XXX_DRE::write(const size_t addr, const uint8_t* buf, const size_t size)
{		
	if( (addr + size) > get_size() )
	{
		SPDLOG_ERROR("size is too large");
		return false;
	}

	SPDLOG_DEBUG("M24XXX_DRE::write 0x{:02X} {:d}@0x{:04X}", m_dev_addr, size, addr);

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

bool M24XXX_DRE::fill(const uint8_t val)
{
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

bool M24XXX_DRE::get_io_addr(const size_t addr, uint8_t* const dev_addr_with_data_addr, std::array<uint8_t, 2>* const addr_data)
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

bool M24XXX_DRE::write_page(const size_t addr, const void* buf, const size_t size)
{
	const M24XXX_DRE_Properties& prop = m_probed_properties.value();

	SPDLOG_DEBUG("M24C64_DRE::write_page 0x{:02X} {:d}@0x{:04X}", m_dev_addr, size, addr);

	const size_t page_addr = (addr / get_pagesize()) * get_pagesize();
	if( (addr + size) > (page_addr + get_pagesize()) )
	{
		SPDLOG_ERROR("M24XXX_DRE::write_page not alligned: {:d}@0x{:04X}", size, addr);
		return false;
	}

	if( size > get_pagesize() )
	{
		SPDLOG_ERROR("M24XXX_DRE::write_page size larger than get_pagesize()");
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

	{
		std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

		if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
		{
			SPDLOG_ERROR("ioctl failed, errno: {:d}", errno);
			return false;
		}
	}
	
	if( ! wait_write_complete() )
	{
		SPDLOG_ERROR("Device write failed");
		return false;
	}

	return true;
}

bool M24XXX_DRE::wait_write_complete()
{
#if 1
	std::this_thread::sleep_for(get_max_write_time());
	return true;
#else

	bool ret = false;

	const size_t max_write_ms = get_max_write_time().count() + 1;
	std::array<uint8_t, 1> data;
	data.fill(0);

	std::array<i2c_msg, 1> trx {};
	trx[0].addr  = m_dev_addr;
	trx[0].flags = 0;
	trx[0].len   = data.size();
	trx[0].buf   = data.data();

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();

	for(size_t i = 0; i < max_write_ms; i++)
	{
		int ioctl_ret = 0;
		{
			std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);
			ioctl_ret = ioctl(m_bus->get_fd(), I2C_RDWR, &idat);
		}

		if(ioctl_ret < 0)
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
