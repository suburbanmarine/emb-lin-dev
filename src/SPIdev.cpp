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

#include "emb-lin-dev/SPIdev.hpp"

#include <spdlog/spdlog.h>

#include <linux/types.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>


bool SPIdev::open(const std::string_view& path)
{
	if(m_fd >= 0)
	{
		return false;
	}

	if( ! m_spi_bus )
	{
		return false;
	}

	m_path = path;

	int ret = ::open(m_path.c_str(), O_RDWR);
	if(ret < 0)
	{
		SPDLOG_ERROR("Could not open spidev {:s}, errno: {:d}", m_path, errno);

		m_path.clear();
		return false;
	}

	m_fd = ret;

	return true;
}
void SPIdev::close()
{
	if(m_fd < 0)
	{
		return;
	}

	int ret = ::close(m_fd);
	if(ret != 0)
	{
		SPDLOG_WARN("Error on close spidev {:s}, errno: {:d}", m_path, errno);		
	}

	m_fd    = -1;
	m_path.clear();
}

bool SPIdev::set_mode(uint8_t mode)
{
	int ret = ioctl(m_fd, SPI_IOC_WR_MODE, &mode);

	return ret == 0;
}
bool SPIdev::set_word_size(uint8_t word_size)
{
	int ret = ioctl(m_fd, SPI_IOC_WR_BITS_PER_WORD, &word_size);

	return ret == 0;
}
bool SPIdev::set_max_speed(uint32_t max_speed)
{
	int ret = ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed);

	return ret == 0;
}
bool SPIdev::read(Buf_type in_buf)
{
	std::unique_lock<std::mutex> lock(m_spi_bus->get_bus_mutex(), std::defer_lock);

	std::array<spi_ioc_transfer, 1>	xfer;
	zero_fill_spi_ioc_transfer(&xfer);

	xfer[0].rx_buf = (__u64) in_buf.data();
	xfer[0].len    = in_buf.size();

	bool fret = true;

	if( ! assert_cs(lock) )
	{
		SPDLOG_ERROR("spidev {:s} read failed to assert CS", m_path);
		return false;
	}

	const int ret = ioctl(m_fd, SPI_IOC_MESSAGE(1), xfer.data());

	if( ! release_cs(lock) )
	{
		SPDLOG_ERROR("spidev {:s} read failed to release CS", m_path);
		fret = false;
	}

	if(ret < 0)
	{
		SPDLOG_ERROR("spidev {:s} read ioctl SPI_IOC_MESSAGE failed, errno: {:d}", m_path, errno);
		fret = false;
	}

	return fret;
}
bool SPIdev::write(const Buf_type out_buf)
{
	std::unique_lock<std::mutex> lock(m_spi_bus->get_bus_mutex(), std::defer_lock);

	std::array<spi_ioc_transfer, 1>	xfer;
	zero_fill_spi_ioc_transfer(&xfer);

	xfer[0].tx_buf = (__u64) out_buf.data();
	xfer[0].len    = out_buf.size();

	bool fret = true;

	if( ! assert_cs(lock) )
	{
		SPDLOG_ERROR("spidev {:s} write failed to assert CS", m_path);
		return false;
	}

	const int ret = ioctl(m_fd, SPI_IOC_MESSAGE(1), xfer.data());

	if( ! release_cs(lock) )
	{
		SPDLOG_ERROR("spidev {:s} write failed to release CS", m_path);
		fret = false;
	}

	if(ret < 0)
	{
		SPDLOG_ERROR("spidev {:s} write ioctl SPI_IOC_MESSAGE failed, errno: {:d}", m_path, errno);
		fret = false;
	}

	return fret;
}

bool SPIdev::write_and_read(const Buf_type out_buf, Buf_type in_buf)
{
	std::unique_lock<std::mutex> lock(m_spi_bus->get_bus_mutex(), std::defer_lock);

	if(out_buf.size() != in_buf.size())
	{
		return false;
	}

	std::array<spi_ioc_transfer, 1>	xfer;
	zero_fill_spi_ioc_transfer(&xfer);

	xfer[0].tx_buf = (__u64) out_buf.data();
	xfer[0].rx_buf = (__u64) in_buf.data();
	xfer[0].len    = out_buf.size();

	bool fret = true;

	if( ! assert_cs(lock) )
	{
		SPDLOG_ERROR("spidev {:s} write_and_read failed to assert CS", m_path);
		return false;
	}

	const int ret = ioctl(m_fd, SPI_IOC_MESSAGE(1), xfer.data());

	if( ! release_cs(lock) )
	{
		SPDLOG_ERROR("spidev {:s} write_and_read failed to release CS", m_path);
		fret = false;
	}

	if(ret < 0)
	{
		SPDLOG_ERROR("spidev {:s} write_and_read ioctl SPI_IOC_MESSAGE failed, errno: {:d}", m_path, errno);
		fret = false;
	}

	return fret;
}

bool SPIdev::write_then_read(const Buf_type out_buf, Buf_type in_buf)
{
	std::unique_lock<std::mutex> lock(m_spi_bus->get_bus_mutex(), std::defer_lock);

	std::array<spi_ioc_transfer, 2>	xfer;
	zero_fill_spi_ioc_transfer(&xfer);

	xfer[0].tx_buf = (__u64) out_buf.data();
	xfer[0].len    = out_buf.size();

	xfer[1].rx_buf = (__u64) in_buf.data();
	xfer[1].len    = in_buf.size();

	bool fret = true;

	if( ! assert_cs(lock) )
	{
		SPDLOG_ERROR("spidev {:s} write_then_read failed to assert CS", m_path);
		return false;
	}

	const int ret = ioctl(m_fd, SPI_IOC_MESSAGE(2), xfer.data());

	if( ! release_cs(lock) )
	{
		SPDLOG_ERROR("spidev {:s} write_then_read failed to release CS", m_path);
		fret = false;
	}

	if(ret < 0)
	{
		SPDLOG_ERROR("spidev {:s} write_then_read ioctl SPI_IOC_MESSAGE failed, errno: {:d}", m_path, errno);
		fret = false;
	}

	return fret;
}
bool SPIdev::write_then_write(const Buf_type buf1, const Buf_type buf2)
{
	std::unique_lock<std::mutex> lock(m_spi_bus->get_bus_mutex(), std::defer_lock);

	std::array<spi_ioc_transfer, 2>	xfer;
	zero_fill_spi_ioc_transfer(&xfer);

	xfer[0].tx_buf = (__u64) buf1.data();
	xfer[0].len    = buf1.size();

	xfer[1].tx_buf = (__u64) buf2.data();
	xfer[1].len    = buf2.size();

	bool fret = true;

	if( ! assert_cs(lock) )
	{
		SPDLOG_ERROR("spidev {:s} read_then_read failed to assert CS", m_path);
		return false;
	}

	const int ret = ioctl(m_fd, SPI_IOC_MESSAGE(2), xfer.data());

	if( ! release_cs(lock) )
	{
		SPDLOG_ERROR("spidev {:s} write_then_write failed to release CS", m_path);
		fret = false;
	}

	if(ret < 0)
	{
		SPDLOG_ERROR("spidev {:s} write_then_write ioctl SPI_IOC_MESSAGE failed, errno: {:d}", m_path, errno);
		fret = false;
	}

	return fret;
}
bool SPIdev::read_then_read(Buf_type buf1, Buf_type buf2)
{
	std::unique_lock<std::mutex> lock(m_spi_bus->get_bus_mutex(), std::defer_lock);

	std::array<spi_ioc_transfer, 2>	xfer;
	zero_fill_spi_ioc_transfer(&xfer);

	xfer[0].rx_buf = (__u64) buf1.data();
	xfer[0].len    = buf1.size();

	xfer[1].rx_buf = (__u64) buf2.data();
	xfer[1].len    = buf2.size();

	bool fret = true;

	if( ! assert_cs(lock) )
	{
		SPDLOG_ERROR("spidev {:s} read_then_read failed to assert CS", m_path);
		return false;
	}

	const int ret = ioctl(m_fd, SPI_IOC_MESSAGE(2), xfer.data());

	if( ! release_cs(lock) )
	{
		SPDLOG_ERROR("spidev {:s} read_then_read failed to release CS", m_path);
		fret = false;
	}

	if(ret < 0)
	{
		SPDLOG_ERROR("spidev {:s} read_then_read ioctl SPI_IOC_MESSAGE failed, errno: {:d}", m_path, errno);
		fret = false;
	}

	return fret;
}

bool SPIdev::assert_cs(std::unique_lock<std::mutex>& lock)
{
	if(m_cs)
	{
		lock.lock();
		return m_cs->assert_cs();
	}
	return true;
}
bool SPIdev::release_cs(std::unique_lock<std::mutex>& lock)
{
	if(m_cs)
	{
		bool ret = m_cs->release_cs();
		lock.unlock();
		return ret;
	}

	return true;
}