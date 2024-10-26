/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#include "emb-lin-dev/BIC_2200.hpp"

BIC_2200::BIC_2200()
{
	m_fd = -1;
}
BIC_2200::~BIC_2200()
{
	close();
}

bool open(const std::string& iface)
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
bool close()
{
	if(m_fd < 0)
	{
		return true;
	}

	int ret = close(m_fd);
	m_fd    = -1;

	return ret == 0;
}


bool wait_tx_can_packet(const std::chrono::microseconds& max_wait_time, const can_frame& tx_frame)
{
	//TODO: use epoll or poll to handle timeout for ability to enqueue frame

	ssize_t len = write(m_fd, tx_frame, sizeof(can_frame));

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

bool wait_rx_can_packet(const std::chrono::microseconds& max_wait_time, can_frame* const out_rx_frame)
{
	//TODO: use epoll or poll to handle timeout for new frame

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