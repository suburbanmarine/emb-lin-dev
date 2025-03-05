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

#include <botan/x509cert.h>
#include <botan/certstor.h>

#include <boost/filesystem/path.hpp>

#include <span>

class Certificate_store
{
public:
	Certificate_store();
	virtual ~Certificate_store();
	bool init(const boost::filesystem::path& dir);

	// add a certificate to the persistant on-disk allowed list of certs
	bool add_cert_to_allowlist(const Botan::X509_Certificate& device_cert);

	bool is_cert_valid_and_allowed(const Botan::X509_Certificate& device_cert);

protected:

	static std::vector<uint8_t> calc_sha256(const std::span<const char>& buf);
	static std::vector<uint8_t> calc_sha256(const std::span<const uint8_t>& buf);
	static std::string calc_sha256_str(const std::span<const uint8_t> buf);

	static std::string base64_encode(const std::span<const uint8_t>& buf);
	static std::vector<uint8_t> base64_decode(const std::string& str);

	boost::filesystem::path m_cert_dir;

	Botan::Certificate_Store_In_Memory m_cert_db;
	// Botan::Flatfile_Certificate_Store m_cert_db
	// Botan::Certificate_Store_In_SQLite m_cert_db

	// Botan::X509_Certificate m_tng_root_cert;
};