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
#include <thread>

#include <cstring>

bool BIC_2200::BIC2200_Packet::to_can_frame(can_frame* const out_can_frame) const
{
	if( ! out_can_frame )
	{
		return false;
	}

	if(payload.size() > 6)
	{
		return false;
	}

	memset(out_can_frame, 0, sizeof(can_frame));

	out_can_frame->can_id  = (addr & CAN_EFF_MASK) | CAN_EFF_FLAG;
	out_can_frame->len     = payload.size() + 2;
	out_can_frame->data[0] = (uint16_t(cmd) & 0x00FFU) >> 0;
	out_can_frame->data[1] = (uint16_t(cmd) & 0xFF00U) >> 8;

	memcpy(out_can_frame->data+2, payload.data(), payload.size());

	return true;
}

bool BIC_2200::BIC2200_Packet::from_can_frame(const can_frame& frame)
{
	if(frame.len > 8)
	{
		return false;
	}

	if(frame.len < 2)
	{
		return false;
	}

	if( (addr & CAN_EFF_FLAG) == 0)
	{
		return false;
	}

	if( (addr & CAN_RTR_FLAG) == 0)
	{
		return false;
	}

	if( (addr & CAN_ERR_FLAG) == 0)
	{
		return false;
	}

	addr = frame.can_id & CAN_EFF_MASK;
	cmd  = static_cast<CMD_OPCODE>( (uint16_t(frame.data[1]) << 8) | (uint16_t(frame.data[0]) << 0) );

	payload.resize(frame.len-2);
	memcpy(payload.data(), frame.data+2, frame.len-2);

	return true;
}

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

	if( ! m_tx_stopwatch.start() )
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


bool BIC_2200::read_mf_id(std::string* const out_mf_id)
{
	BIC2200_Packet cmd;
	cmd.addr = GET_HOST_TO_BIC_ADDR(0);
	cmd.cmd  = CMD_OPCODE::MFR_ID_B0B5;

	if( ! wait_tx_can_packet(std::chrono::milliseconds(10), cmd) )
	{
		return false;
	}

	BIC2200_Packet r0;
	if( ! wait_rx_can_packet(std::chrono::milliseconds(MAX_RESPONSE_TIME) * 3, &r0) )
	{
		return false;
	}

	cmd.cmd = CMD_OPCODE::MFR_ID_B6B11;
	if( ! wait_tx_can_packet(std::chrono::milliseconds(10), cmd) )
	{
		return false;
	}

	BIC2200_Packet r1;
	if( ! wait_rx_can_packet(std::chrono::milliseconds(MAX_RESPONSE_TIME) * 3, &r1) )
	{
		return false;
	}

	if( (r0.cmd != CMD_OPCODE::MFR_ID_B0B5) || (r1.cmd != CMD_OPCODE::MFR_ID_B6B11) )
	{
		return false;
	}

	if(out_mf_id)
	{
		out_mf_id->clear();
		out_mf_id->insert(out_mf_id->end(), r0.payload.begin(), r0.payload.end());
		out_mf_id->insert(out_mf_id->end(), r1.payload.begin(), r1.payload.end());
	}

	return true;
}
bool BIC_2200::read_model(std::string* const out_model)
{
	return true;
}
bool BIC_2200::read_fw_rev(std::vector<std::string>* const out_fw_rev)
{
	return true;
}
bool BIC_2200::read_serial(std::string* const out_serial)
{
	return true;
}

bool BIC_2200::wait_tx_can_packet(const std::chrono::nanoseconds& max_wait_time, const BIC2200_Packet& packet)
{
	can_frame tx_frame;

	if( ! packet.to_can_frame(&tx_frame) )
	{
		return false;
	}

	return wait_tx_can_packet(max_wait_time, tx_frame);
}

bool BIC_2200::wait_tx_can_packet(const std::chrono::nanoseconds& max_wait_time, const can_frame& tx_frame)
{
	// Enforce max packet rate of MIN_REQUEST_PERIOD and adjust max_wait_ts
	{
		bool is_exp;
		if( ! m_tx_stopwatch.is_expired(MIN_REQUEST_PERIOD, &is_exp) )
		{
			return false;
		}

		if( ! is_exp )
		{
			std::chrono::nanoseconds watch_time;
			if( ! m_tx_stopwatch.get_time(&watch_time) )
			{
				return false;
			}

			std::chrono::nanoseconds sleep_duration = MIN_REQUEST_PERIOD - watch_time;
			if(sleep_duration > std::chrono::nanoseconds::zero())
			{
				std::this_thread::sleep_for(sleep_duration);
			}
		}
	}

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

	if( ! m_tx_stopwatch.reset() )
	{
		return false;
	}

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

bool BIC_2200::wait_rx_can_packet(const std::chrono::nanoseconds& max_wait_time, BIC2200_Packet* const out_packet)
{
	can_frame rx_frame;
	if( ! wait_rx_can_packet(max_wait_time, &rx_frame) )
	{
		return false;
	}

	if( ! out_packet->from_can_frame(rx_frame) )
	{
		return false;
	}

	return true;
}

bool BIC_2200::wait_rx_can_packet(const std::chrono::nanoseconds& max_wait_time, can_frame* const out_rx_frame)
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