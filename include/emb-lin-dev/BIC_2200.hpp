/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * This software is distrubuted in the hope it will be useful, but without any warranty, including the implied warrranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE.txt for details.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#pragma once

#include <emb-lin-util/Stopwatch.hpp>

#include <linux/can.h>

#include <chrono>
#include <string>
#include <vector>

#include <cstdint>

class BIC_2200
{
public:

	enum class CMD_OPCODE : uint16_t
	{
		OPERATION            = 0x0000U, 
		VOUT_SET             = 0x0020U,
		IOUT_SET             = 0x0030U,
		FAULT_STATUS         = 0x0040U,
		READ_VIN             = 0x0050U,
		READ_VOUT            = 0x0060U,
		READ_IOUT            = 0x0061U,
		READ_TEMPERATURE_1   = 0x0062U,
		READ_FAN_SPEED_1     = 0x0070U,
		READ_FAN_SPEED_2     = 0x0071U,
		MFR_ID_B0B5          = 0x0080U,
		MFR_ID_B6B11         = 0x0081U,
		MFR_MODEL_B0B5       = 0x0082U,
		MFR_MODEL_B6B11      = 0x0083U,
		MFR_REVISION_B0B5    = 0x0084U,
		MFR_LOCATION_B0B2    = 0x0085U,
		MFR_DATE_B0B5        = 0x0086U,
		MFR_SERIAL_B0B5      = 0x0087U,
		MFR_SERIAL_B6B11     = 0x0088U,
		SCALING_FACTOR       = 0x00C0U,
		SYSTEM_STATUS        = 0x00C1U, 
		SYSTEM_CONFIG        = 0x00C2U,
		DIRECTION_CTRL       = 0x0100U,
		REVERSE_VOUT_SET     = 0x0120U,
		REVERSE_IOUT_SET     = 0x0130U,
		BIDIRECTIONAL_CONFIG = 0x0140U
	};

	enum class SCALE_FACTOR : uint8_t
	{
		NOT_SUPPORTED = 0x00,
		MILLI         = 0x04,
		CENTI         = 0x05,
		DECI          = 0x06,
		ONE           = 0x07,
		TEN           = 0x08,
		HUNDRED       = 0x09
	};

	enum class EEP_CONFIG : uint8_t
	{
		IMMEDIATE    = 0x00,
		DELAY_1_MIN  = 0x01,
		DELAY_10_MIN = 0x02
	};

	enum class OP_INIT : uint8_t
	{
		POWER_OFF  = 0x00,
		POWER_ON   = 0x01,
		POWER_LAST = 0x02
	};

	// SCALING_FACTOR (0x00C0)
	struct SCALING_FACTORS
	{
		SCALE_FACTOR iin;
		SCALE_FACTOR temp1;
		SCALE_FACTOR fan_speed;
		SCALE_FACTOR vin;
		SCALE_FACTOR iout;
		SCALE_FACTOR vout;
	};

	// FAULT_STATUS (0x0040) bitmask values
	enum class FAULT_STATUS_BITMASK : uint16_t
	{
		FAULT_STATUS_FAN_FAIL = 1U << 0,
		FAULT_STATUS_OTP      = 1U << 1,
		FAULT_STATUS_OVP      = 1U << 2,
		FAULT_STATUS_OLP      = 1U << 3,
		FAULT_STATUS_SHORT    = 1U << 4,
		FAULT_STATUS_AC_FAIL  = 1U << 5,
		FAULT_STATUS_OP_OFF   = 1U << 6,
		FAULT_STATUS_HI_TEMP  = 1U << 7,
		FAULT_STATUS_HV_OVP   = 1U << 8
	};

	// SYSTEM_STATUS (0x00C1) bitmask values
	enum class SYSTEM_STATUS_BITMASK : uint16_t
	{
		nSLAVE_MASTER = 1U << 0,
		DC_OK         = 1U << 1,
		PFC_OK        = 1U << 2,
		ADL_ON        = 1U << 3,
		INITIALIZING  = 1U << 4,
		EEPROM_ERROR  = 1U << 5
	};

	class BIC2200_Packet
	{
	public:
		BIC2200_Packet()
		{
			payload.reserve(6);
		}
		~BIC2200_Packet()
		{
			
		}

		uint32_t   addr;
		CMD_OPCODE cmd;
		std::vector<uint8_t> payload; // up to 6 bytes of payload

		bool to_can_frame(can_frame* const out_can_frame) const;
		bool from_can_frame(const can_frame& frame);

		bool is_bic_addr(const uint8_t bic_addr) const
		{
			return (addr & 0x000000FF) == uint32_t(bic_addr);
		}

		bool is_bic_command() const
		{
			return (addr & 0xFFFFFF00) == 0x000C0200U;
		}
		bool is_bic_command(const uint8_t bic_addr) const
		{
			return is_bic_command() && is_bic_addr(bic_addr);
		}
		bool is_bic_command(const uint8_t bic_addr, const CMD_OPCODE bic_cmd) const
		{
			return is_bic_command() && is_bic_addr(bic_addr) && (cmd == bic_cmd);
		}

		bool is_bic_response() const
		{
			return (addr & 0xFFFFFF00) == 0x000C0300U;
		}
		bool is_bic_response(const uint8_t bic_addr) const
		{
			return is_bic_response() && is_bic_addr(bic_addr);
		}
		bool is_bic_response(const uint8_t bic_addr, const CMD_OPCODE bic_cmd) const
		{
			return is_bic_response(bic_addr) && (cmd == bic_cmd);
		}
		bool is_bic_response(const uint8_t bic_addr, const CMD_OPCODE bic_cmd, const size_t len) const
		{
			return is_bic_response(bic_addr, bic_cmd) && (payload.size() == len);
		}
	};

	constexpr static uint32_t GET_HOST_TO_BIC_ADDR(const uint8_t bic_addr)
	{
		return 0x000C0200U | bic_addr;
	};
	constexpr static uint32_t GET_BIC_TO_HOST_ADDR(const uint8_t bic_addr)
	{
		return 0x000C0300U | bic_addr;
	};
	constexpr static uint32_t HOST_BCAST_ADDR  = 0x00C03FF;

	BIC_2200();
	~BIC_2200();

	// eg iface == "can0"
	bool open(const std::string& iface);
	bool close();

	// read metadata
	bool read_mf_id(std::string* const out_mf_id);
	bool read_model(std::string* const out_model);
	bool read_fw_rev(std::vector<std::string>* const out_fw_rev);
	bool read_serial(std::string* const out_date, std::string* const out_serial);

	// read state
	bool read_ac_vin(uint32_t* const out_vin_mv);
	bool read_dc_vout(uint32_t* const out_vout_mv);
	bool read_dc_iout(uint32_t* const out_iout_ma);
	bool read_system_status(uint16_t* const out_reg);
	bool read_fault_status(uint16_t* const out_reg);

	// set config
	bool set_dc_charge_vout(const uint32_t vout_mv);
	bool set_dc_charge_iout(const uint32_t iout_ma);

	bool set_dc_discharge_vout(const uint32_t vout_mv);
	bool set_dc_discharge_iout(const uint32_t iout_ma);

	bool set_system_config(const bool eep_disable, const EEP_CONFIG eep_config, const OP_INIT op_init, const bool can_enable);
	bool set_bidr_config(const bool nAUTO_MANUAL);

	// control regen manually, if can_enable == true and nAUTO_MANUAL == true
	bool set_dir_control(const bool nACTODC_DCTOAC);

protected:

	constexpr static std::chrono::milliseconds MIN_REQUEST_PERIOD = std::chrono::milliseconds(20);
	constexpr static std::chrono::milliseconds MAX_RESPONSE_TIME  = std::chrono::milliseconds(5);
	constexpr static std::chrono::milliseconds MIN_MARGIN_TIME    = std::chrono::milliseconds(5);

	// max_wait_time is delay in addition to the delay needed to enforce MIN_REQUEST_PERIOD
	bool wait_tx_can_packet(const std::chrono::nanoseconds& max_wait_time, const BIC2200_Packet& packet);
	bool wait_tx_can_packet(const std::chrono::nanoseconds& max_wait_time, const can_frame& frame);
	
	bool wait_rx_can_packet(const std::chrono::nanoseconds& max_wait_time, BIC2200_Packet* const out_packet);
	bool wait_rx_can_packet(const std::chrono::nanoseconds& max_wait_time, can_frame* const out_frame);

	int m_fd;
	Stopwatch m_tx_stopwatch;
	constexpr static uint8_t m_bic_addr = 0; // TODO allow this to be configured
};