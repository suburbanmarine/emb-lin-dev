/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
*/

#pragma once

#include "I2C_dev_base.hpp"

#include <optional>
#include <chrono>
#include <cstdint>
#include <cmath>

// class is not threadsafe - some calls transparently cache params for other calls to use
// calls to hw need to be slow - meanwell says wait 50ms between calls
// TODO add pacing timer?
class HEP_1000 : public I2C_dev_base
{
public:
	enum class PMBUS_CMD : uint8_t
	{
		OPERATION              = 0x01,
		ON_OFF_CONFIG          = 0x02,
		CAPABILITY             = 0x19,
		VOUT_MODE              = 0x20,
		VOUT_COMMAND           = 0x21,
		VOUT_TRIM              = 0x22,
		IOUT_OC_FAULT_LIMIT    = 0x46,
		IOUT_OC_FAULT_RESPONSE = 0x47,
		STATUS_WORD            = 0x79,
		STATUS_VOUT            = 0x7A,
		STATUS_IOUT            = 0x7B,
		STATUS_INPUT           = 0x7C,
		STATUS_TEMPERATURE     = 0x7D,
		STATUS_CML             = 0x7E,
		STATUS_MFR_SPECIFIC    = 0x80,
		READ_VIN               = 0x88,
		READ_VOUT              = 0x8B,
		READ_IOUT              = 0x8C,
		READ_TEMPERATURE_1     = 0x8D,
		PMBUS_REVISION         = 0x98,
		MFR_ID                 = 0x99,
		MFR_MODEL              = 0x9A,
		MFR_REVISION           = 0x9B,
		MFR_LOCATION           = 0x9C,
		MFR_DATE               = 0x9D,
		MFR_SERIAL             = 0x9E,
		CURVE_CC               = 0xB0,
		CURVE_CV               = 0xB1,
		CURVE_FV               = 0xB2,
		CURVE_TC               = 0xB3,
		CURVE_CONFIG           = 0xB4,
		CURVE_CC_TIMEOUT       = 0xB5,
		CURVE_CV_TIMEOUT       = 0xB6,
		CURVE_FLOAT_TIMEOUT    = 0xB7,
		CHG_STATUS             = 0xB8,
		SYSTEM_CONFIG          = 0xBE,
		SYSTEM_STATUS          = 0xBF
	};

	class MFG_INFO
	{
	public:
		std::string id;
		std::string model;
		std::string revision;
		std::string location;
		std::string date;
		std::string serial;
	};

	// TODO: Not portable, but probably works
	struct CURVE_CONFIG
	{
		union
		{
			struct
			{
				uint16_t CUVS  : 2; // b0-b1
				uint16_t TCS   : 2; // b2-b3
				uint16_t       : 1; // b4
				uint16_t       : 1; // b5
				uint16_t STGS  : 1; // b6
				uint16_t CUVE  : 1; // b7
				uint16_t CCTOE : 1; // b8
				uint16_t CVTOE : 1; // b9
				uint16_t FVTOE : 1; // b10
				uint16_t       : 1; // b11
				uint16_t       : 1; // b12
				uint16_t       : 1; // b13
				uint16_t       : 1; // b14
				uint16_t       : 1; // b15
			} flags;
			uint16_t reg;
		};
	};

	// TODO: Not portable, but probably works
	struct CHG_STATUS
	{
		union
		{
			struct
			{
				uint16_t FULLM : 1; // b0
				uint16_t CCM   : 1; // b1
				uint16_t CVM   : 1; // b2
				uint16_t FVM   : 1; // b3
				uint16_t       : 1; // b4
				uint16_t       : 1; // b5
				uint16_t       : 1; // b6
				uint16_t       : 1; // b7
				uint16_t       : 1; // b8
				uint16_t       : 1; // b9
				uint16_t NTCER : 1; // b10
				uint16_t BTNC  : 1; // b11
				uint16_t       : 1; // b12
				uint16_t CCTOF : 1; // b13
				uint16_t CVTOF : 1; // b14
				uint16_t FVTOF : 1; // b15
			} flags;
			uint16_t reg;
		};
	};

	// TODO: Not portable, but probably works
	struct SYSTEM_CONFIG
	{
		union
		{
			struct
			{
				uint16_t PM_CTRL        : 1; // b0
				uint16_t OPERATION_INIT : 2; // b1-b2
				uint16_t : 1; // b3
				uint16_t : 1; // b4
				uint16_t : 1; // b5
				uint16_t : 1; // b6
				uint16_t : 1; // b7
				uint16_t EEP_CONFIG : 2; // b8-b9 0: immediate write, 1: 1min delay, 2: 10 min delay
				uint16_t EEP_OFF    : 1; // b10   0: write to eeprom, 1: no write to eeprom
				uint16_t : 1; // b11
				uint16_t : 1; // b12
				uint16_t : 1; // b13
				uint16_t : 1; // b14
				uint16_t : 1; // b15
			} flags;
			uint16_t reg;
		};
	};

	// TODO: Not portable, but probably works
	struct SYSTEM_STATUS
	{
		union
		{
			struct
			{
				uint16_t            : 1; // b0
				uint16_t DC_OK      : 1; // b1
				uint16_t            : 1; // b2
				uint16_t            : 1; // b3
				uint16_t ADL_ON     : 1; // b4
				uint16_t INIT_STATE : 1; // b5
				uint16_t EEP_ERROR  : 1; // b6
				uint16_t            : 1; // b7
				uint16_t            : 1; // b8
				uint16_t            : 1; // b9
				uint16_t            : 1; // b10
				uint16_t            : 1; // b11
				uint16_t            : 1; // b12
				uint16_t            : 1; // b13
				uint16_t            : 1; // b14
				uint16_t            : 1; // b15
			} flags;
			uint16_t reg;
		};
	};

	// TODO: Not portable, but probably works
	struct STATUS_WORD
	{
		union
		{
			struct
			{
				uint16_t NONE_OF_THE_ABOVE : 1; // b0
				uint16_t CML               : 1; // b1
				uint16_t TEMPERATURE       : 1; // b2
				uint16_t VIN_UV_FAULT      : 1; // b3
				uint16_t IOUT_OC_FAULT     : 1; // b4
				uint16_t VOUT_OV_FAULT     : 1; // b5
				uint16_t OFF               : 1; // b6
				uint16_t BUSY              : 1; // b7
				uint16_t UNKNOWN           : 1; // b8
				uint16_t OTHER             : 1; // b9
				uint16_t FANS              : 1; // b10
				uint16_t nPG_STATUS        : 1; // b11
				uint16_t MFR_SPECIFIC      : 1; // b12
				uint16_t INPUT             : 1; // b13
				uint16_t IOUT_POUT         : 1; // b14
				uint16_t VOUT              : 1; // b15
			} flags;
			uint16_t reg;
		};
	};
	
	// TODO: Not portable, but probably works
	struct STATUS_VOUT
	{
		union
		{
			struct
			{
				uint8_t VOUT_TRACKING_ERROR : 1; // b0
				uint8_t TOFF_MAX_WARNING    : 1; // b1
				uint8_t TON_MAX_FAULT       : 1; // b2
				uint8_t VOUT_MAX_MIN        : 1; // b3
				uint8_t VOUT_UV_FAULT       : 1; // b4
				uint8_t VOUT_UV_WARNING     : 1; // b5
				uint8_t VOUT_OV_WARNING     : 1; // b6
				uint8_t VOUT_OV_FAULT       : 1; // b7
			} flags;
			uint8_t reg;
		};
	};
	
	// TODO: Not portable, but probably works
	struct STATUS_IOUT
	{
		union
		{
			struct
			{
				uint8_t POUT_OP_WARNING     : 1; // b0
				uint8_t POUT_OP_FAULT       : 1; // b1
				uint8_t POWER_LIMITING_MODE : 1; // b2
				uint8_t CURRENT_SHARE_FAULT : 1; // b3
				uint8_t IOUT_UC_FAULT       : 1; // b4
				uint8_t IOUT_OC_WARNING     : 1; // b5
				uint8_t IOUT_OC_LV_FAULT    : 1; // b6
				uint8_t IOUT_OC_FAULT       : 1; // b7
			} flags;
			uint8_t reg;
		};
	};
	
	// TODO: Not portable, but probably works
	struct STATUS_INPUT
	{
		union
		{
			struct
			{
				uint8_t PIN_OP_WARNING  : 1; // b0
				uint8_t IIN_OC_WARNING  : 1; // b1
				uint8_t IIN_OC_FAULT    : 1; // b2
				uint8_t OFF_INS_INPUT_V : 1; // b3
				uint8_t VIN_UV_FAULT    : 1; // b4
				uint8_t VIN_UV_WARNING  : 1; // b5
				uint8_t VIN_OV_WARNING  : 1; // b6
				uint8_t VIN_OV_FAULT    : 1; // b7
			} flags;
			uint8_t reg;
		};
	};
	
	// TODO: Not portable, but probably works
	struct STATUS_TEMPERATURE
	{
		union
		{
			struct
			{
				uint8_t : 1; // b0
				uint8_t : 1; // b1
				uint8_t : 1; // b2
				uint8_t : 1; // b3
				uint8_t UT_FAULT   : 1; // b4
				uint8_t UT_WARNING : 1; // b5
				uint8_t OT_WARNING : 1; // b6
				uint8_t OT_FAULT   : 1; // b7
			} flags;
			uint8_t reg;
		};
	};

	// TODO: Not portable, but probably works
	struct STATUS_CML
	{
		union
		{
			struct
			{
				uint8_t OTHER_MEM_LOGIC_FAULT : 1; // b0
				uint8_t COMM_FAULT            : 1; // b1
				uint8_t : 1; // b2
				uint8_t PROC_FAULT            : 1; // b3
				uint8_t MEM_FAULT             : 1; // b4
				uint8_t PACKET_ERROR          : 1; // b5
				uint8_t BAD_DATA              : 1; // b6
				uint8_t BAD_CMD               : 1; // b7
			} flags;
			uint8_t reg;
		};
	};

	// TODO: Not portable, but probably works
	struct CAPABILITY
	{
		union
		{
			struct
			{
				uint8_t : 1; // b0
				uint8_t : 1; // b1
				uint8_t AVSBUS         : 1; // b2
				uint8_t NUMERIC_FORMAT : 1; // b3
				uint8_t nSMBALERT      : 1; // b4
				uint8_t BUS_SPEED      : 2; // b5-b6
				uint8_t PEC            : 1; // b7
			} flags;
			uint8_t reg;
		};
	};

	struct ON_OFF_CONFIG
	{
		union
		{
			struct
			{
				uint8_t CONTROL_PIN_OFF_RATE : 1; // b0
				uint8_t CONTROL_PIN_POL      : 1; // b1
				uint8_t CONTROL_PIN_ACT      : 1; // b2
				uint8_t CONTROL_CMD_ACT      : 1; // b3
				uint8_t CONTROL_SEL          : 1; // b4
				uint8_t : 3; // b5-b7
			} flags;
			uint8_t reg;
		};
	};

	struct OPERATION
	{
		union
		{
			struct
			{
				uint8_t : 1; // b0
				uint8_t TRANSITION_CTRL   : 1; // b1
				uint8_t MARGIN_FAULT_RESP : 2; // b2-B3
				uint8_t VOLTAGE_CMD_SRC   : 2; // b2-B4
				uint8_t OFF_BEHAVIOR      : 1; // b6
				uint8_t ON_OFF_STATE      : 1; // b7
			} flags;
			uint8_t reg;
		};
	};

	HEP_1000(const std::shared_ptr<I2C_bus_base>& bus, const long id);
	virtual ~HEP_1000();

	bool read_pmbus_rev(uint8_t* const part1_rev, uint8_t* const part2_rev)
	{
		if( ! (part1_rev && part2_rev) )
		{
			return false;
		}

		uint8_t temp;
		if( ! read_reg_u8(uint8_t(PMBUS_CMD::PMBUS_REVISION), &temp) )
		{
			return false;
		}

		*part1_rev = (temp & 0xF0U) >> 4;
		*part2_rev = (temp & 0x0FU) >> 0;

		return true;
	}

	bool read_cap_reg(CAPABILITY* const out_val)
	{
		return read_reg_u8(uint8_t(PMBUS_CMD::CAPABILITY), &(out_val->reg));
	}

	bool read_mfg_info(MFG_INFO* const out_val);
	bool read_on_off_config(ON_OFF_CONFIG* const out_val)
	{
		return read_reg_u8(uint8_t(PMBUS_CMD::ON_OFF_CONFIG), &(out_val->reg));
	}

	bool read_vout_mode_reg(uint8_t* const out_val)
	{
		if( ! read_reg_u8(uint8_t(PMBUS_CMD::VOUT_MODE), out_val) )
		{
			return false;
		}

		m_mode_cache = *out_val;

		return true;
	}
	bool read_vout_cmd_reg(uint16_t* const out_val)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::VOUT_COMMAND), out_val);
	}
	bool read_vout_trim_reg(uint16_t* const out_val)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::VOUT_TRIM), out_val);
	}

	bool read_vout_cmd(float* const out_val);
	bool read_vout_trim(float* const out_val);
	bool write_vout_trim(const float val);

	bool read_vin_reg(uint16_t* const out_val)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::READ_VIN), out_val);
	}
	bool read_vout_reg(uint16_t* const out_val)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::READ_VOUT), out_val);
	}
	bool read_iout_reg(uint16_t* const out_val)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::READ_IOUT), out_val);
	}
	bool read_temp1_reg(uint16_t* const out_val)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::READ_TEMPERATURE_1), out_val);
	}

	// volts
	bool read_vin(float* const out_val);
	// volts
	bool read_vout(float* const out_val);
	// amps
	bool read_iout(float* const out_val);
	// degC
	bool read_temp1(float* const out_val);

	bool read_status_word_reg(STATUS_WORD* const out_val)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::STATUS_WORD), &(out_val->reg));
	}
	bool read_status_vout_reg(STATUS_VOUT* const out_val)
	{
		return read_reg_u8(uint8_t(PMBUS_CMD::STATUS_VOUT), &(out_val->reg));
	}
	bool read_status_iout_reg(STATUS_IOUT* const out_val)
	{
		return read_reg_u8(uint8_t(PMBUS_CMD::STATUS_IOUT), &(out_val->reg));
	}
	bool read_status_input_reg(STATUS_INPUT* const out_val)
	{
		return read_reg_u8(uint8_t(PMBUS_CMD::STATUS_INPUT), &(out_val->reg));
	}
	bool read_status_temp_reg(STATUS_TEMPERATURE* const out_val)
	{
		return read_reg_u8(uint8_t(PMBUS_CMD::STATUS_TEMPERATURE), &(out_val->reg));
	}
	bool read_status_cml_reg(STATUS_CML* const out_val)
	{
		return read_reg_u8(uint8_t(PMBUS_CMD::STATUS_CML), &(out_val->reg));
	}
	bool read_status_mfg_reg(uint8_t* const out_val)
	{
		return read_reg_u8(uint8_t(PMBUS_CMD::STATUS_MFR_SPECIFIC), out_val);
	}

	bool read_curve_config(CURVE_CONFIG* const out_val)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::CURVE_CONFIG), &(out_val->reg));
	}
	bool read_chg_status(CHG_STATUS* const out_val)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::CHG_STATUS), &(out_val->reg));
	}
	bool read_system_config(SYSTEM_CONFIG* const out_val)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::SYSTEM_CONFIG), &(out_val->reg));
	}
	bool read_system_status(SYSTEM_STATUS* const out_val)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::SYSTEM_STATUS), &(out_val->reg));
	}

	bool read_charge_cc(uint16_t* const out_reg)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::CURVE_CC), out_reg);
	}
	bool read_charge_cv(uint16_t* const out_reg)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::CURVE_CV), out_reg);
	}
	bool read_charge_fv(uint16_t* const out_reg)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::CURVE_FV), out_reg);
	}
	bool read_charge_tc(uint16_t* const out_reg)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::CURVE_TC), out_reg);
	}

	bool read_charge_cc(float* const out_val_amps);  // constant current limit
	bool read_charge_cv(float* const out_val_volts); // compliance voltage during const current charge. once this current is reached, V is held here until current drops to tc.
	bool read_charge_fv(float* const out_val_volts); // float voltage, voltage is kept here once current drops to tc
	bool read_charge_tc(float* const out_val_amps);  // current limit to "complete" charging and move to float

	bool write_charge_cc(const float val_amps);
	bool write_charge_cv(const float val_volts);
	bool write_charge_fv(const float val_volts);
	bool write_charge_tc(const float val_amps);

	bool read_timeout_cc_reg(uint16_t* const out_reg)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::CURVE_CC_TIMEOUT), out_reg);
	}
	bool read_timeout_cv_reg(uint16_t* const out_reg)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::CURVE_CV_TIMEOUT), out_reg);
	}
	bool read_timeout_float_reg(uint16_t* const out_reg)
	{
		return read_reg_u16(uint8_t(PMBUS_CMD::CURVE_FLOAT_TIMEOUT), out_reg);
	}

	bool read_timeout_cc(float* const val_s);
	bool read_timeout_cv(float* const val_s);
	bool read_timeout_float(float* const val_s);
	bool write_timeout_cc(const float val_s);
	bool write_timeout_cv(const float val_s);
	bool write_timeout_float(const float val_s);

	// https://graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend
	template<size_t LEN>
	static constexpr int signextend(const int x)
	{
		struct {signed int val : LEN;} s;
		s.val = x;
		return s.val;
	}

	static constexpr uint8_t get_addr(const uint8_t sel)
	{
		return BASE_ADDR | (sel & 0x03U);
	}

	static constexpr float linear16_to_float(const uint16_t val, const uint8_t mode_reg)
	{
		const int exp = signextend<5>(mode_reg & 0x1FU);

		const float scale = std::pow(2.0f, exp); // TODO: not sure if faster easy way to do this, exp is typically negative. Could divide in a for loop...

		return float(val) * scale;
	}
	static constexpr uint16_t float_to_linear16(const float val, const uint8_t mode_reg)
	{
		const int exp = signextend<5>(mode_reg & 0x1FU);

		const float scale = std::pow(2.0f, exp); // TODO: not sure if faster easy way to do this, exp is typically negative. Could divide in a for loop...

		return uint16_t(std::round(float(val) / scale));
	}
	static constexpr float linear11_to_float(const uint16_t val)
	{
		const uint16_t data = (val & 0x07FFU) >> 0;
		const uint8_t  mode = (val & 0xF800U) >> 11;

		return linear16_to_float(data, mode);
	}
	static constexpr uint16_t float_to_linear11(const float val, const uint8_t exp)
	{
		const uint16_t temp = float_to_linear16(val, exp);

		return ((exp & 0x001FU) << 11) | (temp & 0x07FFU);
	}

protected:

	std::optional<uint8_t> m_mode_cache; // maybe we should cook the float scale instead of storing the mode

	bool read_reg_u8(const uint8_t cmd, uint8_t* const reg);
	bool read_reg_u16(const uint8_t cmd, uint16_t* const reg);

	bool write_reg_u8(const uint8_t cmd, const uint8_t reg);
	bool write_reg_u16(const uint8_t cmd, const uint16_t reg);

	static constexpr uint8_t BASE_ADDR = 0x40U;
	static constexpr std::chrono::milliseconds CMD_DELAY = std::chrono::milliseconds(50);
};
