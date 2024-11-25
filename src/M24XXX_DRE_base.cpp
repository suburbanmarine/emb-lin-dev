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

#include "emb-lin-dev/M24XXX_DRE_base.hpp"

// density_code -> size
const std::map<uint8_t, M24XXX_DRE_base::M24XXX_DRE_Properties> M24XXX_DRE_base::DEVICE_PROPERTIES = {
	{0x08U,   {256,  16, 1}},
	{0x09U,   {512,  16, 1}},
	{0x0AU,  {1024,  16, 1}},
	{0x0BU,  {2048,  16, 1}},
	{0x0CU,  {4096,  32, 2}},
	{0x0DU,  {8192,  32, 2}},
	{0x0EU, {16384,  64, 2}},
	{0x0FU, {32768,  64, 2}},
	{0x10U, {65536, 128, 2}}
};

M24XXX_DRE_base::M24XXX_DRE_base(const std::shared_ptr<I2C_bus_base>& bus, const long id) : I2C_dev_base(bus, id)
{

}

M24XXX_DRE_base::~M24XXX_DRE_base()
{

}

bool M24XXX_DRE_base::probe_eeprom(M24XXX_DRE_ID* const out_id)
{
	return false;
}
bool M24XXX_DRE_base::read_id_code(Device_id_code* const out_buf)
{
	return false;
}
bool M24XXX_DRE_base::read_id_page(std::vector<uint8_t>* const out_id_page)
{
	return false;
}
bool M24XXX_DRE_base::write_id_page(const std::vector<uint8_t>& id_page)
{
	return false;
}
bool M24XXX_DRE_base::lock_id_page()
{
	return false;
}
bool M24XXX_DRE_base::get_id_lock_status(bool* const is_locked)
{
	return false;
}