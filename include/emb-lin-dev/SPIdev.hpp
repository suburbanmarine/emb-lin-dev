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

#pragma once

#include <linux/spi/spidev.h>

#include <array>
#include <string>
#include <string_view>
#include <memory>
#include <mutex>

// #include <span>
#include <cstdint>
#include <cstddef>

template<typename T>
class Span
{
public:

	Span()
	{
		m_data = nullptr;
		m_size = 0;
	}

	Span(T* data, size_t len)
	{
		m_data = data;
		m_size = len;
	}

	template<size_t LEN>
	Span(std::array<T, LEN>& buf)
	{
		m_data = buf.data();
		m_size = LEN;
	}

	T * data()
	{
		return m_data;
	}

	T const * data() const
	{
		return m_data;
	}

	size_t size() const
	{
		return m_size;
	}

protected:
	T * m_data;
	size_t   m_size;
};

class SPIdev_cs
{
public:
	SPIdev_cs()
	{

	}
	virtual ~SPIdev_cs()
	{

	}

	virtual bool init()
	{
		return true;
	}

	virtual bool assert_cs()
	{
		return true;
	}
	virtual bool release_cs()
	{
		return true;
	}
};

// Class modeling spidev bus
// Needed to lock userland CS control
// the actual spidev ioctl is threadsafe, but since we do userland CS, we need to lock CS assertion bus wide
// This also means we cannot mix spidev and in-kernel drivers
class SPIdev_bus
{
public:
	SPIdev_bus()
	{

	}
	virtual ~SPIdev_bus()
	{

	}
	std::mutex& get_bus_mutex()
	{
		return m_bus_mutex;
	}
protected:
	std::mutex m_bus_mutex;
};

class SPIdev
{
public:

	typedef Span<uint8_t>       Buf_type;
	typedef Span<const uint8_t> Const_buf_type;

	SPIdev(const std::shared_ptr<SPIdev_bus>& spi_bus)
	{
		m_spi_bus = spi_bus;
		m_fd = -1;
	}
	~SPIdev()
	{
		close();
	}

	int get_fd() const
	{
		return m_fd;
	}

	// Not MT safe
	bool open(const std::string_view& path);
	// Not MT safe
	void close();

	// Not MT safe
	// SPI_MODE_0..SPI_MODE_3 or combination of SPI_CPOL (idle high if set) and SPI_CPHA flags (sample on trailing edge if set)
	bool set_mode(uint8_t mode);
	// Not MT safe
	bool set_word_size(uint8_t word_size);
	// Not MT safe
	bool set_max_speed(uint32_t max_speed);

	// Not MT safe
	void set_cs_cb(const std::shared_ptr<SPIdev_cs>& cs)
	{
		m_cs = cs;
	}

	// MT safe
	bool read(Buf_type out_buf);
	// MT safe
	bool write(const Buf_type buf);

	// MT safe
	bool write_and_read(const Buf_type out_buf, Buf_type in_buf);
	// MT safe
	bool write_then_read(const Buf_type out_buf, Buf_type in_buf);
	// MT safe
	bool write_then_write(const Buf_type buf1, const Buf_type buf2);
	// MT safe
	bool read_then_read(Buf_type buf1, Buf_type buf2);

protected:

	//if cb exists, locks lock then asserts_cs
	bool assert_cs(std::unique_lock<std::mutex>& lock);
	//if cb exists, releases cs then unlocks lock. lock is released even if cs release fails
	bool release_cs(std::unique_lock<std::mutex>& lock);

	template<size_t LEN>
	static void zero_fill_spi_ioc_transfer(std::array<spi_ioc_transfer, LEN>* const out_buf)
	{
		memset(out_buf->data(), 0, sizeof(spi_ioc_transfer)*out_buf->size());
	}

	int m_fd;
	std::string m_path;

	std::shared_ptr<SPIdev_bus> m_spi_bus;
	std::shared_ptr<SPIdev_cs>  m_cs;
};
