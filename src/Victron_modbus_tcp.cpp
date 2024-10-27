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

const std::map<std::string, Victron_modbus_tcp::VictronModbusTcpRegister> Victron_modbus_tcp::VICTRON_REG_MAP = 
{
	{ "/Serial",                      {.path = "/Serial",                      .address =   800, .type = Victron_modbus_tcp::RegisterType::STRING, .scalefactor = std::make_pair(0, 0),   .writable = false} },
	{ "/Hub4/L1/AcPowerSetpoint",     {.path = "/Hub4/L1/AcPowerSetpoint",     .address =    37, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/L2/AcPowerSetpoint",     {.path = "/Hub4/L2/AcPowerSetpoint",     .address =    40, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/L3/AcPowerSetpoint",     {.path = "/Hub4/L3/AcPowerSetpoint",     .address =    41, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/DisableCharge",          {.path = "/Hub4/DisableCharge",          .address =    38, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/DisableFeedIn",          {.path = "/Hub4/DisableFeedIn",          .address =    39, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/DoNotFeedInOvervoltage", {.path = "/Hub4/DoNotFeedInOvervoltage", .address =    65, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/L1/MaxFeedInPower",      {.path = "/Hub4/L1/MaxFeedInPower",      .address =    66, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(1, 100), .writable = true} },
	{ "/Hub4/L2/MaxFeedInPower",      {.path = "/Hub4/L2/MaxFeedInPower",      .address =    67, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(1, 100), .writable = true} },
	{ "/Hub4/L3/MaxFeedInPower",      {.path = "/Hub4/L3/MaxFeedInPower",      .address =    68, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(1, 100), .writable = true} },
	{ "/Hub4/TargetPowerIsMaxFeedIn", {.path = "/Hub4/TargetPowerIsMaxFeedIn", .address =    71, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/FixSolarOffsetTo100mV",  {.path = "/Hub4/FixSolarOffsetTo100mV",  .address =    72, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Settings/Ess/Mode",           {.path = "/Settings/Ess/Mode",           .address =  4921, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Ess/AcPowerSetpoint",         {.path = "/Ess/AcPowerSetpoint",         .address =  4922, .type = Victron_modbus_tcp::RegisterType::INT32,  .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Ess/DisableFeedIn",           {.path = "/Ess/DisableFeedIn",           .address =  4924, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	
	{ "/Dc/Battery/Voltage",          {.path = "/Dc/Battery/Voltage",          .address =   840, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(10, 1),  .writable = false} },
	{ "/Dc/Battery/Current",          {.path = "/Dc/Battery/Current",          .address =   841, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(10, 1),  .writable = false} },
	{ "/Dc/Battery/Power",            {.path = "/Dc/Battery/Power",            .address =   842, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(1, 1),   .writable = false} },
	{ "/Dc/Battery/Soc",              {.path = "/Dc/Battery/Soc",              .address =   843, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(1, 1),   .writable = false} },
	{ "/Dc/Battery/State",            {.path = "/Dc/Battery/State",            .address =   844, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(1, 1),   .writable = false} },
	{ "/Dc/Battery/ConsumedAmphours", {.path = "/Dc/Battery/ConsumedAmphours", .address =   845, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(10, -1), .writable = false} },
	{ "/Dc/Battery/TimeToGo",         {.path = "/Dc/Battery/TimeToGo",         .address =   846, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(1, 100), .writable = false} },

	{ "/Ac/ActiveIn/L1/V",            {.path = "/Ac/ActiveIn/L1/V",            .address =     3, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L2/V",            {.path = "/Ac/ActiveIn/L2/V",            .address =     4, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L3/V",            {.path = "/Ac/ActiveIn/L3/V",            .address =     5, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L1/I",            {.path = "/Ac/ActiveIn/L1/I",            .address =     6, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L2/I",            {.path = "/Ac/ActiveIn/L2/I",            .address =     7, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L3/I",            {.path = "/Ac/ActiveIn/L3/I",            .address =     8, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L1/F",            {.path = "/Ac/ActiveIn/L1/F",            .address =     9, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/L2/F",            {.path = "/Ac/ActiveIn/L2/F",            .address =    10, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/L3/F",            {.path = "/Ac/ActiveIn/L3/F",            .address =    11, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/L1/P",            {.path = "/Ac/ActiveIn/L1/P",            .address =    12, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/ActiveIn/L2/P",            {.path = "/Ac/ActiveIn/L2/P",            .address =    13, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/ActiveIn/L3/P",            {.path = "/Ac/ActiveIn/L3/P",            .address =    14, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(1, 10), .writable  = false} },

	{ "/Ac/Out/L1/V",                 {.path = "/Ac/Out/L1/V",                 .address =    15, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L2/V",                 {.path = "/Ac/Out/L2/V",                 .address =    16, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L3/V",                 {.path = "/Ac/Out/L3/V",                 .address =    17, .type = Victron_modbus_tcp::RegisterType::UINT16, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L1/I",                 {.path = "/Ac/Out/L1/I",                 .address =    18, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L2/I",                 {.path = "/Ac/Out/L2/I",                 .address =    19, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L3/I",                 {.path = "/Ac/Out/L3/I",                 .address =    20, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(10, 1), .writable = false} },
	{ "/Ac/Out/L1/F",                 {.path = "/Ac/Out/L1/F",                 .address =    21, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/CurrentLimit",    {.path = "/Ac/ActiveIn/CurrentLimit",    .address =    22, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(10, 1), .writable = false} },
	{ "/Ac/Out/L1/P",                 {.path = "/Ac/Out/L1/P",                 .address =    23, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/Out/L2/P",                 {.path = "/Ac/Out/L2/P",                 .address =    24, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/Out/L3/P",                 {.path = "/Ac/Out/L3/P",                 .address =    25, .type = Victron_modbus_tcp::RegisterType::INT16,  .scalefactor = std::make_pair(1, 10), .writable  = false} }
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