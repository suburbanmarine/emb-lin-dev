/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#include "emb-lin-dev/BIC_2200.hpp"

#include <emb-lin-util/Timespec_util.hpp>

#include <signal.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>

#include <array>

#include <cstring>

BIC_2200::BIC_2200()
{
	m_fd = -1;
}
BIC_2200::~BIC_2200()
{
	close();
}

bool BIC_2200::open(const std::string& iface)
{
	if(m_fd >= 0)
	{
		close();
	}

	sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	ifreq ifr;
	memset(&ifr, 0, sizeof(ifr));

	m_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(m_fd < 0)
	{
		return false;
	}

	strcpy(ifr.ifr_name, iface.c_str() );
	int ret = ioctl(m_fd, SIOCGIFINDEX, &ifr);
	if(ret < 0)
	{
		return false;
	}

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	ret = bind(m_fd, (sockaddr*)&addr, sizeof(addr));
	if(ret < 0)
	{
		return false;
	}

    can_err_mask_t err_mask = CAN_ERR_MASK;
    ret = setsockopt(m_fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
	if(ret < 0)
	{
		return false;
	}

	return true;
}
bool BIC_2200::close()
{
	if(m_fd < 0)
	{
		return true;
	}

	int ret = ::close(m_fd);
	m_fd    = -1;

	return ret == 0;
}


bool BIC_2200::wait_tx_can_packet(const std::chrono::microseconds& max_wait_time, const can_frame& tx_frame)
{
	pollfd write_fds[] = {
		{.fd = m_fd, .events = POLLOUT}
	};

	timespec max_wait_ts = Timespec_util::from_chrono(max_wait_time);
	sigset_t sigmask;
	sigemptyset(&sigmask);

	int ret = ppoll(write_fds, sizeof(write_fds) / sizeof(pollfd), &max_wait_ts, &sigmask);
	if(ret < 0)
	{
		// TODO: log errno
		return false;
	}
	else if(ret == 0)
	{
		// no space
		return false;
	}

	ssize_t len = write(m_fd, &tx_frame, sizeof(can_frame));

	if (len < 0)
	{
		// todo log errno
        return false;
	}

	if (len != sizeof(can_frame) )
	{
		// underrun?
		return false;
	}

	return true;
}

bool BIC_2200::wait_rx_can_packet(const std::chrono::microseconds& max_wait_time, can_frame* const out_rx_frame)
{
	pollfd read_fds[] = {
		{.fd = m_fd, .events = POLLIN}
	};

	timespec max_wait_ts = Timespec_util::from_chrono(max_wait_time);
	sigset_t sigmask;
	sigemptyset(&sigmask);

	int ret = ppoll(read_fds, sizeof(read_fds) / sizeof(pollfd), &max_wait_ts, &sigmask);
	if(ret < 0)
	{
		// TODO: log errno
		return false;
	}
	else if(ret == 0)
	{
		// no data
		return false;
	}

	ssize_t len = read(m_fd, out_rx_frame, sizeof(can_frame));

	if (len < 0)
	{
		// todo log errno
        return false;
	}

	if (len != sizeof(can_frame) )
	{
		// underrun?
		return false;
	}

	return true;
}