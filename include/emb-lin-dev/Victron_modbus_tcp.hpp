/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#pragma once 

#include <cstdint>

class Victron_modbus_tcp
{
public:
	Victron_modbus_tcp();
	~Victron_modbus_tcp();

	enum class FUNCTION_CODE : uint8_t
	{
		READ_HOLDING_REGISTERS   = 0x01U,
		READ_INPUT_REGISTERS     = 0x02U,
		WRITE_SINGLE_REGISTER    = 0x03U,
		WRITE_MULTIPLE_REGISTERS = 0x0AU
	};

	enum class ERROR : uint8_t
	{
		ILLEGAL_FUNCTION                        = 0x01U,
		ILLEGAL_DATA_ADDRESS                    = 0x02U,
		ILLEGAL_DATA_VALUE                      = 0x03U,
		GATEWAY_PATH_UNAVAILABLE                = 0x0AU,
		GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0BU
	};

	enum class REGISTER_ID : uint8_t
	{
		AcPowerSetpointL1 = 37,
		AcPowerSetpointL2 = 40,
		AcPowerSetpointL3 = 41,
		DisableCharge     = 38,
		DisableFeedIn     = 39
	}
};
