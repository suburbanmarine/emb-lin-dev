/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#include <emb-lin-dev/crypto/Certificate_store.hpp>

#include <emb-lin-dev/crypto/ATECC608_TNGTLS_iface.hpp>

#include "emb-lin-util/File_util.hpp"

#include <botan/base64.h>
#include <botan/hex.h>
#include <botan/x509_key.h>
#include <botan/hash.h>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range_core.hpp>

#include <spdlog/spdlog.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <memory>

std::string Certificate_store::base64_encode(const std::span<const uint8_t>& buf)
{
	return Botan::base64_encode(buf.data(), buf.size());
}
std::vector<uint8_t> Certificate_store::base64_decode(const std::string& str)
{
	std::vector<uint8_t> buf;
	
	{
		const size_t len = Botan::base64_decode_max_output(str.size());
		buf.resize(len);
	}

	size_t outlen = Botan::base64_decode(buf.data(), str.data(), str.size());
	buf.resize(outlen);

	return buf;
}

std::vector<uint8_t> Certificate_store::calc_sha256(const std::span<const char>& buf)
{
	std::unique_ptr<Botan::HashFunction> hash = Botan::HashFunction::create_or_throw("SHA-256");
	hash->update(reinterpret_cast<const uint8_t*>(buf.data()), buf.size());
	return hash->final_stdvec();
}

std::vector<uint8_t> Certificate_store::calc_sha256(const std::span<const uint8_t>& buf)
{
	std::unique_ptr<Botan::HashFunction> hash = Botan::HashFunction::create_or_throw("SHA-256");
	hash->update(buf.data(), buf.size());
	return hash->final_stdvec();
}

std::string Certificate_store::calc_sha256_str(const std::span<const uint8_t> buf)
{
	const std::vector<uint8_t> hash = calc_sha256(buf);

	return Botan::hex_encode(hash, false);
}

Certificate_store::Certificate_store()
{

}

Certificate_store::~Certificate_store()
{
	
}

bool Certificate_store::init(const boost::filesystem::path& dir)
{
	if( ! boost::filesystem::is_directory(dir) )
	{
		SPDLOG_ERROR("Error loading certs, path {:s} is not a directory", dir.string());
		return false;
	}

	Botan::X509_Certificate root_cert_tng;
	if( ! ATECC608_TNGTLS_iface::read_root_certificate(&root_cert_tng) )
	{
		SPDLOG_ERROR("Certificate_store::init could not get root cert pubkey");
		return false;					
	}

	std::vector<uint8_t> file_buf;
	for(const auto& dir_ent : boost::make_iterator_range(boost::filesystem::directory_iterator(dir), {}))
	{
		try
		{
			if( ! boost::filesystem::is_regular_file(dir_ent.status()) )
			{
				SPDLOG_ERROR("Certificate_store::init failed to load {:s}", dir_ent.path().string());
				continue;
			}

			Envelope_signature_with_nonce env;
			if( ! env.read_json(dir_ent.path().string()) )
			{
				SPDLOG_ERROR("Certificate_store::init failed to load {:s}", dir_ent.path().string());
				continue;
			}

			ATECC608_info info;
			try
			{
				info.from_cbor(env.msg);
			}
			catch(const std::exception& e)
			{
				SPDLOG_ERROR("Certificate_store::init failed to load {:s}", dir_ent.path().string());
				continue;
			}

			Botan::X509_Certificate device_cert(
				Botan::unlock(
					Botan::base64_decode(info.device_cert)
				)
			);

			Botan::X509_Certificate signer_cert(
				Botan::unlock(
					Botan::base64_decode(info.signer_cert)
				)
			);

			Botan::X509_Certificate root_cert(
				Botan::unlock(
					Botan::base64_decode(info.root_cert)
				)
			);

			const std::string device_cert_fingerprint_b64   = base64_encode( calc_sha256( device_cert.BER_encode() ) );
			SPDLOG_DEBUG("Loading cert: {:s}", device_cert_fingerprint_b64);

			// Check cert root
			{
				if(root_cert_tng != root_cert)
				{
					SPDLOG_ERROR("Certificate_store::init unknown root cert");
					continue;
				}

				std::unique_ptr< Botan::Public_Key > root_cert_pubkey = root_cert.load_subject_public_key();
				if( ! root_cert_pubkey )
				{
					SPDLOG_ERROR("Certificate_store::init could not get root cert pubkey");
					continue;
				}

				std::unique_ptr< Botan::Public_Key > signer_cert_pubkey = signer_cert.load_subject_public_key();
				if( ! signer_cert_pubkey )
				{
					SPDLOG_ERROR("Certificate_store::init could not get signer cert pubkey");
					continue;
				}

				if( ! signer_cert.check_signature(*root_cert_pubkey) )
				{
					SPDLOG_ERROR("Certificate_store::init signer_cert not signed by root_cert");
					continue;
				}
				else
				{
					SPDLOG_DEBUG("signer_cert sig ok for {:s}", device_cert_fingerprint_b64);
				}

				if( ! device_cert.check_signature(*signer_cert_pubkey) )
				{
					SPDLOG_ERROR("Certificate_store::init device_cert not signed by signer_cert");
					continue;
				}
				else
				{
					SPDLOG_DEBUG("device_cert sig ok for {:s}", device_cert_fingerprint_b64);
				}
			
				std::unique_ptr< Botan::Public_Key > device_cert_pubkey = device_cert.load_subject_public_key();
				if( ! device_cert_pubkey )
				{
					SPDLOG_ERROR("Certificate_store::init could not get device cert pubkey");
					continue;
				}

				// check signature of overall message against device_cert_pubkey
				// this ties the attestation key to the MCP root cert
				{
					if( ! ATECC608_iface::verify_ext_with_nonce(std::span<uint8_t>(env.msg.data(), env.msg.size()), info.serial_number, info.config, env.sig, *device_cert_pubkey) )
					{
						SPDLOG_ERROR("envelope sig bad for {:s}", device_cert_fingerprint_b64);
						continue;
					}
					else
					{
						SPDLOG_DEBUG("envelope sig ok for {:s}", device_cert_fingerprint_b64);
					}
				}

				// check the common name of the device cert matches the sn in the config blob
				{
					std::vector<std::string> subject_info = device_cert.subject_info("X520.CommonName");
					if(subject_info.size() != 1)
					{
						continue;
					}

					if( subject_info[0] != fmt::format("sn{:s}", Botan::hex_encode(info.serial_number.data(), info.serial_number.size())) )
					{
						SPDLOG_ERROR("device_cert CommonName bad for {:s}", device_cert_fingerprint_b64);
						continue;
					}
					else
					{

						SPDLOG_DEBUG("device_cert CommonName ok for {:s}", device_cert_fingerprint_b64);
					}
				}
			}

			// check key attestation
			{
				const auto it = info.pubkeys.find("ATTESTATION");
				if(it == info.pubkeys.end())
				{
					SPDLOG_ERROR("Could not get attestation key");
					continue;
				}

				std::shared_ptr<const Botan::Public_Key> tmpkey(Botan::X509::load_key(
																				Botan::unlock(Botan::base64_decode(it->second))
																			));

				std::shared_ptr<const Botan::EC_PublicKey> attestation_pubkey = std::dynamic_pointer_cast<const Botan::EC_PublicKey>(tmpkey);
				if( ! attestation_pubkey )
				{
					SPDLOG_ERROR("Could not get attestation key");
					continue;
				}

				std::list<std::string> key_to_check = {"MASTER", "USER0", "USER1", "USER2"};

				bool keys_ok = true;
				for(const auto& keyName : key_to_check)
				{
					const auto keyIt = info.key_attestation.find(keyName);
					if(keyIt == info.key_attestation.end())
					{
						keys_ok = false;
						continue;
					}

					const std::string&       key_name     = keyIt->first;
					const KeyFingerprintSig& key_sig_data = keyIt->second;

					if( ! ATECC608_iface::verify_key_fingerprint_signature(info.serial_number, info.config, key_sig_data, *attestation_pubkey) )
					{
						SPDLOG_ERROR("Certificate_store::init verify_key_fingerprint_signature failed for {:s}", key_name);
						keys_ok = false;
						continue;
					}
					else
					{
						SPDLOG_DEBUG("Certificate_store::init verify_key_fingerprint_signature good for {:s}", key_name);
					}
				}

				if( ! keys_ok )
				{
					continue;
				}
			}

			m_cert_db.add_certificate(device_cert);
		}
		catch(const std::exception& e)
		{
			SPDLOG_ERROR("Error loading cert {:s}: {:s}, continuing", dir_ent.path().string(), e.what());
			continue;
		}
	}

	return true;
}
