/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#include "emb-lin-dev/Victron_modbus_tcp.hpp"

#include <cstring>

const static std::map<std::string, Victron_modbus_tcp::VictronModbusTcpRegister> VICTRON_REG_MAP = 
{
	{ "/Serial", {.path = "/Serial", .address = 800, .type = Victron_modbus_tcp::RegisterType::STRING, .scalefactor = 0, .writable = false} },
	{ "/Hub4/L1/AcPowerSetpoint", {.path = "/Hub4/L1/AcPowerSetpoint", .address = 37, .type = Victron_modbus_tcp::RegisterType::INT16, .scalefactor = 1, .writable = true} }
};

bool Victron_modbus_tcp::Modbus_tcp_frame::serialize(std::vector<uint8_t>* const out_frame)
{
	const uint16_t length = 4+payload.size();
	out_frame->resize(length);

	(*out_frame)[0] = (trx_id      & 0xFF00U) >> 8;
	(*out_frame)[1] = (trx_id      & 0x00FFU) >> 0;
	(*out_frame)[2] = (protocol_id & 0xFF00U) >> 8;
	(*out_frame)[3] = (protocol_id & 0x00FFU) >> 0;
	(*out_frame)[4] = (length      & 0xFF00U) >> 8;
	(*out_frame)[5] = (length      & 0x00FFU) >> 0;
	(*out_frame)[6] = unit_id;
	(*out_frame)[7] = func_code;
	(*out_frame)[8] = (reg_id & 0xFF00U) >> 8;
	(*out_frame)[9] = (reg_id & 0x00FFU) >> 0;

	memcpy(out_frame->data()+10, payload.data(), payload.size());

	return true;
}

bool Victron_modbus_tcp::Modbus_tcp_frame::deserialize(const std::vector<uint8_t>& frame)
{
	if(frame.size() < MBAP_HDR_LEN)
	{
		return false;
	}

	// MODBUS Application Protocol (MBAP) header
	trx_id          = (uint16_t(frame[0]) << 8) | (uint16_t(frame[1]) << 0);
	protocol_id     = (uint16_t(frame[2]) << 8) | (uint16_t(frame[3]) << 0);
	uint16_t length = (uint16_t(frame[4]) << 8) | (uint16_t(frame[5]) << 0);
	unit_id         = frame[6];

	if(protocol_id != 0)
	{
		return false;
	}

	if(frame.size() < MBAP_PACKET_MIN_LEN)
	{
		return false;
	}

	func_code = frame[7];

	reg_id = (uint16_t(frame[8]) << 8) | (uint16_t(frame[9]) << 0);

	payload.resize(length - 4);
	memcpy(payload.data(), frame.data()+8, payload.size());

	return true;
}