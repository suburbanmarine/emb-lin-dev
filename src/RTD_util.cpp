/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2023 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#include "emb-lin-dev/RTD_util.hpp"

#include <cmath>

float RTD_util::calc_temp_from_ohm(const float r) const
{
	const int32_t r_lu = roundf(r * 1e3f);

	auto u_it = mohm_degC_lut.upper_bound(r_lu); // return first key after k
	if(u_it == mohm_degC_lut.end())
	{
		return std::numeric_limits<float>::quiet_NaN();
	}

	if(u_it == mohm_degC_lut.begin())
	{
		return std::numeric_limits<float>::quiet_NaN();
	}

	auto l_it = std::prev(u_it, 1);

	return interp(r_lu, *l_it, *u_it);
}

float RTD_util::calc_res_neg(const float degC) const
{
	const float degC2 = degC  * degC;
	const float degC3 = degC2 * degC;
	return R0 * (1.0f + A*degC + B*degC2 + C*(degC - 100.0f)*degC3);
}

float RTD_util::calc_res_pos(const float degC) const
{
	const float degC2 = degC  * degC;
	return R0 * (1.0f + A*degC + B*degC2);
}

void RTD_util::generate_lut(int t0, int t1, const int dt)
{
	for(int t = t0; t <= t1; t += dt)
	{
		const float temp = float(t);
		
		float res = 0.0f;
		if(t < 0)
		{
			res  = calc_res_neg(temp);
		}
		else
		{
			res  = calc_res_pos(temp);
		}

		const int32_t r_lu = roundf(res * 1e3f);
		mohm_degC_lut.insert(std::make_pair(r_lu, temp));
	}
}

float RTD_util::interp(const float x, const std::pair<float, float>& m, const std::pair<float, float>& n)
{
	return m.second + (x - m.first) * (n.second - m.second) / (n.first - m.first);
}