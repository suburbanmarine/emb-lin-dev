/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * This software is distrubuted in the hope it will be useful, but without any warranty, including the implied warrranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE.txt for details.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024-2025 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#include "emb-lin-dev/Victron_modbus_tcp.hpp"

#include <spdlog/spdlog.h>

// TODO: maybe just directly read the spreadsheet, or have a script dump the spreadsheet here
// TODO: add length field so we can read strings easily
// TODO: add min/max fields for bounds checking
const std::map<std::string, Victron_modbus_tcp::VictronModbusTcpRegister> Victron_modbus_tcp::VICTRON_REG_MAP = 
{
	{ "/Serial",                      {.path = "/Serial",                      .address =   800, .type = RegisterType::STRING, .num_reg = 6, .scalefactor = std::make_pair(0, 0),   .writable = false} },
	{ "/Hub4/L1/AcPowerSetpoint",     {.path = "/Hub4/L1/AcPowerSetpoint",     .address =    37, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/L2/AcPowerSetpoint",     {.path = "/Hub4/L2/AcPowerSetpoint",     .address =    40, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/L3/AcPowerSetpoint",     {.path = "/Hub4/L3/AcPowerSetpoint",     .address =    41, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/DisableCharge",          {.path = "/Hub4/DisableCharge",          .address =    38, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/DisableFeedIn",          {.path = "/Hub4/DisableFeedIn",          .address =    39, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/DoNotFeedInOvervoltage", {.path = "/Hub4/DoNotFeedInOvervoltage", .address =    65, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/L1/MaxFeedInPower",      {.path = "/Hub4/L1/MaxFeedInPower",      .address =    66, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 100), .writable = true} },
	{ "/Hub4/L2/MaxFeedInPower",      {.path = "/Hub4/L2/MaxFeedInPower",      .address =    67, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 100), .writable = true} },
	{ "/Hub4/L3/MaxFeedInPower",      {.path = "/Hub4/L3/MaxFeedInPower",      .address =    68, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 100), .writable = true} },
	{ "/Hub4/TargetPowerIsMaxFeedIn", {.path = "/Hub4/TargetPowerIsMaxFeedIn", .address =    71, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Hub4/FixSolarOffsetTo100mV",  {.path = "/Hub4/FixSolarOffsetTo100mV",  .address =    72, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Settings/Ess/Mode",           {.path = "/Settings/Ess/Mode",           .address =  4921, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Ess/AcPowerSetpoint",         {.path = "/Ess/AcPowerSetpoint",         .address =  4922, .type = RegisterType::INT32,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	{ "/Ess/DisableFeedIn",           {.path = "/Ess/DisableFeedIn",           .address =  4924, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = true} },
	
	{ "/Dc/Battery/Voltage",          {.path = "/Dc/Battery/Voltage",          .address =   840, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1),  .writable = false} },
	{ "/Dc/Battery/Current",          {.path = "/Dc/Battery/Current",          .address =   841, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1),  .writable = false} },
	{ "/Dc/Battery/Power",            {.path = "/Dc/Battery/Power",            .address =   842, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = false} },
	{ "/Dc/Battery/Soc",              {.path = "/Dc/Battery/Soc",              .address =   843, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = false} },
	{ "/Dc/Battery/State",            {.path = "/Dc/Battery/State",            .address =   844, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 1),   .writable = false} },
	{ "/Dc/Battery/ConsumedAmphours", {.path = "/Dc/Battery/ConsumedAmphours", .address =   845, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, -1), .writable = false} },
	{ "/Dc/Battery/TimeToGo",         {.path = "/Dc/Battery/TimeToGo",         .address =   846, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(1, 100), .writable = false} },

	{ "/Ac/ActiveIn/L1/V",            {.path = "/Ac/ActiveIn/L1/V",            .address =     3, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L2/V",            {.path = "/Ac/ActiveIn/L2/V",            .address =     4, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L3/V",            {.path = "/Ac/ActiveIn/L3/V",            .address =     5, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L1/I",            {.path = "/Ac/ActiveIn/L1/I",            .address =     6, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L2/I",            {.path = "/Ac/ActiveIn/L2/I",            .address =     7, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L3/I",            {.path = "/Ac/ActiveIn/L3/I",            .address =     8, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/ActiveIn/L1/F",            {.path = "/Ac/ActiveIn/L1/F",            .address =     9, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/L2/F",            {.path = "/Ac/ActiveIn/L2/F",            .address =    10, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/L3/F",            {.path = "/Ac/ActiveIn/L3/F",            .address =    11, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/L1/P",            {.path = "/Ac/ActiveIn/L1/P",            .address =    12, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/ActiveIn/L2/P",            {.path = "/Ac/ActiveIn/L2/P",            .address =    13, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/ActiveIn/L3/P",            {.path = "/Ac/ActiveIn/L3/P",            .address =    14, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },

	{ "/Ac/Out/L1/V",                 {.path = "/Ac/Out/L1/V",                 .address =    15, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L2/V",                 {.path = "/Ac/Out/L2/V",                 .address =    16, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L3/V",                 {.path = "/Ac/Out/L3/V",                 .address =    17, .type = RegisterType::UINT16, .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L1/I",                 {.path = "/Ac/Out/L1/I",                 .address =    18, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L2/I",                 {.path = "/Ac/Out/L2/I",                 .address =    19, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable  = false} },
	{ "/Ac/Out/L3/I",                 {.path = "/Ac/Out/L3/I",                 .address =    20, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable = false} },
	{ "/Ac/Out/L1/F",                 {.path = "/Ac/Out/L1/F",                 .address =    21, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(100, 1), .writable = false} },
	{ "/Ac/ActiveIn/CurrentLimit",    {.path = "/Ac/ActiveIn/CurrentLimit",    .address =    22, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(10, 1), .writable = false} },
	{ "/Ac/Out/L1/P",                 {.path = "/Ac/Out/L1/P",                 .address =    23, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/Out/L2/P",                 {.path = "/Ac/Out/L2/P",                 .address =    24, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} },
	{ "/Ac/Out/L3/P",                 {.path = "/Ac/Out/L3/P",                 .address =    25, .type = RegisterType::INT16,  .num_reg = 0, .scalefactor = std::make_pair(1, 10), .writable  = false} }
};


Victron_modbus_tcp::Victron_modbus_tcp()
{
	
}
Victron_modbus_tcp::~Victron_modbus_tcp()
{
	
}

bool Victron_modbus_tcp::read_serial(std::string* const out_serial)
{
	Modbus_pdu_response_03 resp_pdu;
	if( ! read_register("/Serial", &resp_pdu) )
	{
		return false;
	}

	if(resp_pdu.is_exception())
	{
		return false;
	}

	if(out_serial)
	{
		out_serial->clear();
		out_serial->insert(out_serial->end(), resp_pdu.payload.begin(), resp_pdu.payload.end());
	}

	return true;
}

bool Victron_modbus_tcp::read_register(const std::string& register_name, Modbus_pdu_response_03* const out_resp)
{
	auto reg_info = VICTRON_REG_MAP.find(register_name);
	if(reg_info == VICTRON_REG_MAP.end())
	{
		// add metadata about register to VICTRON_REG_MAP
		return false;
	}

	size_t num_reg = reg_info->second.num_reg;
	if( num_reg == 0)
	{
		num_reg = get_regtype_payload_length(reg_info->second.type);
	}

	if(num_reg == 0)
	{
		// unsupported by this api, eg a unknown length string
		return false;
	}

	Modbus_pdu_request_03 pdu;
	pdu.func_code = uint8_t(FUNCTION_CODE::READ_HOLDING_REGISTERS);
	pdu.reg_start = reg_info->second.address;
	pdu.num_reg   = num_reg;

	Modbus_tcp_frame cmd_frame;
	cmd_frame.trx_id  = req_id++;
	cmd_frame.unit_id = uint8_t(CERBO_GX_UNIT_ID::VECAN);
	if( ! pdu.serialize(&cmd_frame.pdu) )
	{
		return false;
	}

	Modbus_tcp_frame resp_frame;
	if( ! send_cmd_resp(cmd_frame, &resp_frame) )
	{
		return false;
	}

	if( ! out_resp->deserialize(resp_frame.pdu) )
	{
		return false;
	}

	return true;
}

bool Victron_modbus_tcp::write_register(const std::string& register_name, const double val, Modbus_pdu_response_16* const out_resp)
{
	auto reg_info = VICTRON_REG_MAP.find(register_name);
	if(reg_info == VICTRON_REG_MAP.end())
	{
		// add metadata about register to VICTRON_REG_MAP
		return false;
	}

	double scaled_val = val;
 	scaled_val *= double(reg_info->second.scalefactor.first);
 	scaled_val /= double(reg_info->second.scalefactor.second);

 	const long long int_scaled_val = std::llround(scaled_val);

	Modbus_pdu_request_16 pdu;
	pdu.func_code = uint8_t(FUNCTION_CODE::WRITE_MULTIPLE_REGISTERS);
	pdu.reg_start = reg_info->second.address;
	pdu.num_reg   = Victron_modbus_tcp::get_regtype_payload_length(reg_info->second.type);

	switch(reg_info->second.type)
	{
		case RegisterType::INT16:
		case RegisterType::UINT16:
		{
			pdu.length     = 2;
			pdu.payload.resize(2);
			pdu.payload[0] = (int_scaled_val & 0xFF00U) >> 8;
			pdu.payload[1] = (int_scaled_val & 0x00FFU) >> 0;
			break;
		}
		case RegisterType::INT32:
		case RegisterType::UINT32:
		{
			pdu.length     = 4;
			pdu.payload.resize(4);
			pdu.payload[0] = (int_scaled_val & 0xFF000000U) >> 24;
			pdu.payload[1] = (int_scaled_val & 0x00FF0000U) >> 16;
			pdu.payload[2] = (int_scaled_val & 0x0000FF00U) >> 8;
			pdu.payload[3] = (int_scaled_val & 0x000000FFU) >> 0;
			break;
		}
		default:
		{
			return false;
		}
	}

	return write_register(pdu, out_resp);
}

bool Victron_modbus_tcp::write_register(const Modbus_pdu_request_16& pdu, Modbus_pdu_response_16* const out_resp)
{
	Modbus_tcp_frame cmd_frame;
	cmd_frame.trx_id  = req_id++;
	cmd_frame.unit_id = uint8_t(CERBO_GX_UNIT_ID::VECAN);
	if( ! pdu.serialize(&cmd_frame.pdu) )
	{
		return false;
	}

	Modbus_tcp_frame resp_frame;
	if( ! send_cmd_resp(cmd_frame, &resp_frame) )
	{
		return false;
	}

	if( ! out_resp->deserialize(resp_frame.pdu) )
	{
		return false;
	}

	return true;	
}
