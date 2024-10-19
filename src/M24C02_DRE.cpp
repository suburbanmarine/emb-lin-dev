/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
*/

#include "emb-lin-dev/M24C02_DRE.hpp"

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

#include <array>
#include <algorithm>

M24C02_DRE::M24C02_DRE(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
{

}

bool M24C02_DRE::write_id_page(const Pagebuffer& data)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	Writebuffer buf;
	buf[0] = 0x00;
	std::copy_n(data.data(), data.size(), buf.data()+get_addrsize());

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
bool M24C02_DRE::read_id_page(Pagebuffer* const out_buf)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	std::array<uint8_t, 1> addr_data = {{0x00}};

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = addr_data.size();
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
bool M24C02_DRE::read_id_code(Device_id_code* const out_buf)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	std::array<uint8_t, 1> addr_data = {{0x00}};

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = addr_data.size();
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
bool M24C02_DRE::lock_id_page()
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	std::array<uint8_t, 2> cmd;
	cmd[0] = 0x80; // A7 set
	cmd[1] = 0x02; // lock command

	std::array<i2c_msg, 1> trx {};
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = 0;
	trx[0].len   = cmd.size();
	trx[0].buf   = cmd.data();

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
bool M24C02_DRE::get_id_lock_status(bool* const is_locked)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	std::array<uint8_t, 2> cmd;
	cmd[0] = 0x80; // A7 set
	cmd[1] = 0x00;

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
	// trx[1].addr  = get_idpage_addr();
	// trx[1].flags = I2C_M_RD;
	// trx[1].len   = 0;
	// trx[1].buf   = nullptr;

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
	cmd.fill(0x00);
	trx[0].addr  = get_idpage_addr();
	trx[0].flags = I2C_M_RD;
	trx[0].len   = 1;
	trx[0].buf   = cmd.data();
	if(ioctl(m_bus->get_fd(), I2C_RDWR, &idat) < 0)
	{
		SPDLOG_WARN("Error when resetting M24C02_DRE");
	}

	return true;
}
bool M24C02_DRE::read(const size_t addr, void* buf, const size_t size)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	SPDLOG_TRACE("read 0x{:02X} {:d}@0x{:04X}", m_dev_addr, size, addr);

	std::array<uint8_t, 2> addr_buf;
	addr_buf[0] = (addr >> 0) & 0xFFU;

	std::array<i2c_msg, 2> trx {};
	trx[0].addr  = m_dev_addr;
	trx[0].flags = 0;
	trx[0].len   = addr_buf.size();
	trx[0].buf   = addr_buf.data();

	trx[1].addr  = m_dev_addr;
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
bool M24C02_DRE::write(const size_t addr, const void* buf, const size_t size)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	SPDLOG_TRACE("M24C02_DRE::write 0x{:02X} {:d}@0x{:04X}", m_dev_addr, size, addr);

	size_t num_written = 0;
	// if we are not on a page boundary, unroll the first write
	if( (addr % get_pagesize()) != 0)
	{
		const size_t base_addr    = (addr / get_pagesize()) * get_pagesize();
		const size_t num_to_write = std::min(base_addr + get_pagesize() - addr, size);

		if( ! write_page(addr, buf, num_to_write) )
		{
			return false;
		}

		num_written += num_to_write;
	}

	// write up to page at a time, on page boundaries
	while(num_written < size)
	{
		const size_t num_to_write = std::min<size_t>(size - num_written, get_pagesize());
		
		if( ! write_page(addr + num_written, static_cast<unsigned char const *>(buf) + num_written, num_to_write) )
		{
			return false;
		}

		num_written += num_to_write;
	}

	return true;
}

bool M24C02_DRE::fill(const uint8_t val)
{
	std::shared_ptr<I2C_bus_open_close> bus_closer = std::make_shared<I2C_bus_open_close>(*m_bus);

	const size_t size = get_size();

	// gather IO not supported by driver and/or FT260S
	Writebuffer write_buf;
	write_buf.fill(val);

	std::array<i2c_msg, 1> trx {};

	i2c_rdwr_ioctl_data idat {};
	idat.msgs  = trx.data();
	idat.nmsgs = trx.size();

	for(size_t i = 0; i < size; i += get_pagesize())
	{
		const size_t curr_addr    = i;
		const size_t num_to_write = std::min<size_t>(size - i, get_pagesize());

		SPDLOG_TRACE("write 0x{:02X} {:d}@0x{:04X}", m_dev_addr, get_pagesize(), curr_addr);

		write_buf[0] = (curr_addr >> 0) & 0xFFU;

		trx[0].addr  = m_dev_addr;
		trx[0].flags = 0;
		trx[0].len   = num_to_write + get_addrsize();
		trx[0].buf   = write_buf.data();

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
	}

	return true;
}

bool M24C02_DRE::write_page(const size_t addr, const void* buf, const size_t size)
{
	SPDLOG_TRACE("M24C02_DRE::write_page 0x{:02X} {:d}@0x{:04X}", m_dev_addr, size, addr);

	const size_t page_addr = (addr / get_pagesize()) * get_pagesize();
	if( (addr + size) > (page_addr + get_pagesize()) )
	{
		SPDLOG_ERROR("M24C02_DRE::write_page not alligned: {:d}@0x{:04X}", size, addr);
		return false;
	}

	if( size > get_pagesize() )
	{
		SPDLOG_ERROR("M24C02_DRE::write_page size larger than get_pagesize()");
		return false;
	}

	// gather IO not supported by driver and/or FT260S
	Writebuffer write_buf;
	write_buf[0] = (addr & 0xFFU) >> 0;
	std::copy_n(static_cast<unsigned char const *>(buf), size, write_buf.data() + get_addrsize());
	
	std::array<i2c_msg, 1> trx {};
	trx[0].addr  = m_dev_addr;
	trx[0].flags = 0;
	trx[0].len   = size + get_addrsize();
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

bool M24C02_DRE::wait_write_complete()
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
