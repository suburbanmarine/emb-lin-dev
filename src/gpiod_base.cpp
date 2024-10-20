/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2023 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
*/

#include "emb-lin-dev/gpiod_base.hpp"

#include <spdlog/spdlog.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

gpiod_base::gpiod_base()
{
	m_io = nullptr;
	m_io_lines = {};
	gpiod_line_bulk_init(&m_io_lines);
}
gpiod_base::~gpiod_base()
{
	close();
}

bool gpiod_base::open(const char* name)
{
	m_io = gpiod_chip_open_by_name(name);
	if( ! m_io )
	{
		return false;
	}

	if(gpiod_chip_get_all_lines(m_io, &m_io_lines) != 0)
	{
		return false;
	}

	// const char* m_io_name  = gpiod_chip_name(m_io);
	// const char* m_io_label = gpiod_chip_label(m_io);

	return true;
}
void gpiod_base::close()
{
	if(m_io)
	{
		gpiod_chip_close(m_io);
		gpiod_line_bulk_init(&m_io_lines);
		m_io = nullptr;
	}
}

gpiod_line* gpiod_base::get_line_handle(const size_t idx)
{
	if(idx >= m_io_lines.num_lines)
	{
		return nullptr;
	}

	return m_io_lines.lines[idx];
}

bool gpiod_base::set_line(const unsigned int idx, const int value)
{
	if(idx >= m_io_lines.num_lines)
	{
		return false;
	}

	if(gpiod_line_set_value(m_io_lines.lines[idx], value) != 0)
	{
		return false;
	}

	return true;
};

bool gpiod_base::get_line(const unsigned int idx, int* const out_value)
{
	if(idx >= m_io_lines.num_lines)
	{
		return false;
	}

	int ret = gpiod_line_get_value(m_io_lines.lines[idx]);
	if(ret < 0)
	{
		return false;
	}

	if(out_value)
	{
		*out_value = ret;
	}

	return true;
};
