/**
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2023 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the 3-Clause BSD LICENSE. See LICENSE.txt for details.
*/

#pragma once

#include <cstdint>
#include <limits>
#include <map>

class RTD_util
{
public:

	RTD_util(const float R_ref) : R0(R_ref), Z3((4.0f  * B) / R0)
	{

	}
	// generate the lut over the range [t0, t1]
	void generate_lut(const int t0, const int t1, const int dt);

	//MT-safe
	// Resistance to temperature
	// returns NaN on error
	float calc_temp_from_ohm(const float r) const;

	// Temperature to resistance
	float calc_res_neg(const float degC) const;
	float calc_res_pos(const float degC) const;

protected:
	
	static float interp(const float x, const std::pair<float, float>& m, const std::pair<float, float>& n);

	const float R0;

	std::map<int32_t, float> mohm_degC_lut;

	constexpr static float A = 3.850e-3f; // DIN
	constexpr static float B = -5.775e-7f;
	constexpr static float C = -4.183e-12f;


	constexpr static float Z1 = -1.0f * A;
	constexpr static float Z2 = -4.0f * B;
	const            float Z3;
	constexpr static float Z4 = 2.0f  * B;
};

class PT100_func : public RTD_util
{
public:
	PT100_func() : RTD_util(100.0f)
	{

	}
};

class PT1000_func : public RTD_util
{
public:
	PT1000_func() : RTD_util(1000.0f)
	{

	}
};