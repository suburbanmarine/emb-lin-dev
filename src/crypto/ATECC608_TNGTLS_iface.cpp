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

#include <emb-lin-dev/crypto/ATECC608_TNGTLS_iface.hpp>
#include <emb-lin-dev/crypto/ATECC608_botan.hpp>

#include "tng_atcacert_client.h" // MCP util for TNG preloaded certs

#include "tnglora_cert_def_2_device.h"
#include "tnglora_cert_def_4_device.h"
#include "tngtls_cert_def_2_device.h"
#include "tngtls_cert_def_3_device.h"
#include "tflxtls_cert_def_4_device.h"
#include "tngtls_cert_def_1_signer.h"
#include "tng_root_cert.h"

#include "atcacert/atcacert_def.h"
#include "atcacert/atcacert.h"
#include "atcacert/atcacert_client.h"

#include <botan/base64.h>
#include <botan/der_enc.h>
#include <botan/ec_group.h>
#include <botan/hash.h>

#include <botan/x509_ca.h>
#include <botan/x509_key.h>
#include <botan/x509self.h>
#include <botan/x509path.h>

#include <spdlog/spdlog.h>
#include <fmt/format.h>

#include <list>
#include <span>

void to_json(nlohmann::json& j, const ATECC608_info& val)
{
	j = nlohmann::json {
		{"revision",        Botan::base64_encode(val.revision.data(),      val.revision.size())},
		{"serial_number",   Botan::base64_encode(val.serial_number.data(), val.serial_number.size())},
		{"config",          Botan::base64_encode(val.config.data(),        val.config.size())},
		{"otp0",            Botan::base64_encode(val.otp0.data(),          val.otp0.size())},
		{"otp1",            Botan::base64_encode(val.otp1.data(),          val.otp1.size())},
		{"pubkeys",         val.pubkeys},
		{"key_attestation", val.key_attestation},
		{"device_cert",     val.device_cert},
		{"signer_cert",     val.signer_cert},
		{"root_cert",       val.root_cert}
	};
}
void from_json(const nlohmann::json& j, ATECC608_info& val)
{
	std::string tempstr;
	std::vector<uint8_t> tempvec;

	j.at("revision").get_to(tempstr);
	tempvec = Botan::unlock(Botan::base64_decode(tempstr));
	ATECC608_iface::copy_vec_to_arr(tempvec, val.revision);

	j.at("serial_number").get_to(tempstr);
	tempvec = Botan::unlock(Botan::base64_decode(tempstr));
	ATECC608_iface::copy_vec_to_arr(tempvec, val.serial_number);

	j.at("config").get_to(tempstr);
	tempvec = Botan::unlock(Botan::base64_decode(tempstr));
	ATECC608_iface::copy_vec_to_arr(tempvec, val.config);

	j.at("otp0").get_to(tempstr);
	tempvec = Botan::unlock(Botan::base64_decode(tempstr));
	ATECC608_iface::copy_vec_to_arr(tempvec, val.otp0);

	j.at("otp1").get_to(tempstr);
	tempvec = Botan::unlock(Botan::base64_decode(tempstr));
	ATECC608_iface::copy_vec_to_arr(tempvec, val.otp1);

	j.at("pubkeys").get_to(val.pubkeys);
	j.at("key_attestation").get_to(val.key_attestation);
	j.at("device_cert").get_to(val.device_cert);
	j.at("signer_cert").get_to(val.signer_cert);
	j.at("root_cert").get_to(val.root_cert);
}

// copied from tng_atca.c, since it does not let us look up by string id from otp0
const std::map<std::string, atcacert_def_t const *> ATECC608_TNGTLS_iface::m_cert_def_lookup = 
{
	{"wdNxAjae", &g_tngtls_cert_def_2_device},
	{"Rsuy5YJh", &g_tngtls_cert_def_2_device},
	{"BxZvm6q2", &g_tnglora_cert_def_2_device},
	{"MKMwyhP1", &g_tflxtls_cert_def_4_device},
	{"KQp2ZkD8", &g_tngtls_cert_def_3_device},
	{"x6tjuZMy", &g_tngtls_cert_def_3_device},
	{"jsMu7iYO", &g_tnglora_cert_def_4_device},
	{"09qJNxI3", &g_tnglora_cert_def_4_device}
};

bool ATECC608_TNGTLS_iface::init(const uint8_t bus, const uint8_t address)
{
	if( ! ATECC608_iface::init(bus, address) )
	{
		return false;
	}

	// read pub keys
	{
		std::list<int> key_ids = {
			int(KEY_SLOT_ID::MASTER),
			int(KEY_SLOT_ID::ATTESTATION),
			int(KEY_SLOT_ID::USER0),
			int(KEY_SLOT_ID::USER1),
			int(KEY_SLOT_ID::USER2)
		};
		for(const int& slot_id : key_ids)
		{
			std::array<uint8_t, 64> tmpbuf;
			if( ! read_pubkey(slot_id, &tmpbuf) )
			{
				SPDLOG_ERROR("ATECC608_TNGTLS_iface::init could not read pubkey {:d}", slot_id);
				return false;
			}

			Botan::EC_Group secp256r1_group("secp256r1");
			std::shared_ptr<Botan::ECDSA_PublicKey> tmpkey = std::make_shared<Botan::ECDSA_PublicKey>(
				secp256r1_group,
				secp256r1_group.point(
					Botan::BigInt(tmpbuf.data()+ 0, 32),
					Botan::BigInt(tmpbuf.data()+32, 32)
				)
			);

			if( ! tmpkey )
			{
				SPDLOG_ERROR("ATECC608_TNGTLS_iface::init could not parse pubkey {:d}", slot_id);
				return false;
			}

			if( ! tmpkey->check_key(m_rng, true) )
			{
				SPDLOG_ERROR("ATECC608_TNGTLS_iface::init gave us bad pubkey {:d}", slot_id);
			}

			m_pubkeys.insert_or_assign(slot_id, tmpkey);

			SPDLOG_DEBUG("Pubkey {:d}: {:s} / {:s} ok", 
				slot_id,
				Botan::base64_encode(tmpbuf.data()+ 0, 32),
				Botan::base64_encode(tmpbuf.data()+32, 32)
			);
		}
	}

	if( ! read_device_certificate(&m_device_cert) )
	{
		SPDLOG_ERROR("ATECC608_TNGTLS_iface::init get_device_certificate failed");
		return false;
	}
	if( ! read_signer_certificate(&m_signer_cert) )
	{
		SPDLOG_ERROR("ATECC608_TNGTLS_iface::init get_signer_certificate failed");
		return false;
	}
	if( ! read_root_certificate(&m_root_cert) )
	{
		SPDLOG_ERROR("ATECC608_TNGTLS_iface::init get_root_certificate failed");
		return false;
	}

	std::unique_ptr< Botan::Public_Key > m_root_cert_pubkey = m_root_cert.load_subject_public_key();
	if( ! m_root_cert_pubkey )
	{
		SPDLOG_ERROR("ATECC608_TNGTLS_iface::init could not get root cert pubkey");
		return false;
	}

	std::unique_ptr< Botan::Public_Key > m_signer_cert_pubkey = m_signer_cert.load_subject_public_key();
	if( ! m_signer_cert_pubkey )
	{
		SPDLOG_ERROR("ATECC608_TNGTLS_iface::init could not get root cert pubkey");
		return false;
	}

	if( ! m_signer_cert.check_signature(*m_root_cert_pubkey) )
	{
		SPDLOG_ERROR("ATECC608_TNGTLS_iface::init m_signer_cert not signed by m_root_cert");
		return false;
	}
	else
	{
		SPDLOG_DEBUG("ATECC608_TNGTLS_iface::init m_signer_cert signature ok by m_root_cert");
	}

	if( ! m_device_cert.check_signature(*m_signer_cert_pubkey) )
	{
		SPDLOG_ERROR("ATECC608_TNGTLS_iface::init m_device_cert not signed by m_signer_cert");
		return false;
	}
	else
	{
		SPDLOG_DEBUG("ATECC608_TNGTLS_iface::init m_device_cert signature ok by m_signer_cert");
	}

	{
		m_factory_cert_store = Botan::Certificate_Store_In_Memory();
		m_factory_cert_store.add_certificate(m_root_cert);
		std::vector<Botan::X509_Certificate> cert_chain = {m_device_cert, m_signer_cert};
		Botan::Path_Validation_Result path_ret = Botan::x509_path_validate(cert_chain, Botan::Path_Validation_Restrictions(false, 128), m_factory_cert_store);
		if(path_ret.successful_validation())
		{
			SPDLOG_INFO("device cert path validation OK");
		}
		else
		{
			SPDLOG_WARN("device cert path validation failed: {:s}", path_ret.result_string());
		}
	}

	try
	{
		std::shared_ptr<const Botan::ECDSA_PublicKey> master_pubkey = get_master_pubkey();
		if( ! master_pubkey )
		{
			SPDLOG_ERROR("ATECC608_TNGTLS_iface::init m_device_cert_pubkey error");
			return false;
		}

		std::unique_ptr< Botan::Public_Key > m_device_cert_pubkey = m_device_cert.load_subject_public_key();
		if( ! m_device_cert_pubkey )
		{
			SPDLOG_ERROR("ATECC608_TNGTLS_iface::init m_device_cert_pubkey error");
			return false;
		}

		Botan::ECDSA_PublicKey m_device_cert_ec_pubkey(m_device_cert_pubkey->algorithm_identifier(), m_device_cert_pubkey->public_key_bits());

		if(m_device_cert_ec_pubkey.domain() != master_pubkey->domain())
		{
			SPDLOG_ERROR("ATECC608_TNGTLS_iface::init m_device_cert_pubkey domain does not match");
			return false;
		}
		if(m_device_cert_ec_pubkey.public_point() != master_pubkey->public_point())
		{
			SPDLOG_ERROR("ATECC608_TNGTLS_iface::init m_device_cert_pubkey public_point does not match");
			return false;
		}
	}
	catch(const std::exception& e)
	{
		SPDLOG_ERROR("m_device_cert_pubkey error: {:s}", e.what());
		return false;
	}

	SPDLOG_DEBUG("ATECC608_TNGTLS_iface::init key m_root_cert:\n{:s}",   m_root_cert.to_string());
	SPDLOG_DEBUG("ATECC608_TNGTLS_iface::init key m_signer_cert:\n{:s}", m_signer_cert.to_string());
	SPDLOG_DEBUG("ATECC608_TNGTLS_iface::init key m_device_cert:\n{:s}", m_device_cert.to_string());

	// record attestation signatures
	{
		std::array<uint16_t, 4> key_ids = {
			uint16_t(KEY_SLOT_ID::MASTER),
			uint16_t(KEY_SLOT_ID::USER0),
			uint16_t(KEY_SLOT_ID::USER1),
			uint16_t(KEY_SLOT_ID::USER2)
		};

		for(const uint16_t& slot_id : key_ids)
		{
			KeyFingerprintSig sig_data;
			if( ! sign_key_fingerprint(slot_id, uint16_t(KEY_SLOT_ID::ATTESTATION), &sig_data) )
			{
				return false;
			}

			m_key_attestation.insert_or_assign(slot_id, sig_data);
		}
	}

	return true;
}

bool ATECC608_TNGTLS_iface::write_otp_i2c_address(const uint8_t addr)
{
	return write_updateextra(UPDATE_MODE_SELECTOR, addr);
}

std::string ATECC608_TNGTLS_iface::get_device_cert_id() const
{
	// Use OTP0 string, eg x6tjuZMy, to id cert defaults

	const size_t cert_id_len = strnlen(reinterpret_cast<char const *>(m_otp0_cache.data()), m_otp0_cache.size());
	
	std::string cert_id;
	cert_id.assign(reinterpret_cast<char const *>(m_otp0_cache.data()), cert_id_len);

	return cert_id;
}

void const * ATECC608_TNGTLS_iface::get_device_cert_def()
{
	// Use OTP0 string, eg x6tjuZMy, to id cert defaults

	const std::string cert_id = get_device_cert_id();

	atcacert_def_t const * cert_def = nullptr;

	auto it = m_cert_def_lookup.find(cert_id);
	if(it == m_cert_def_lookup.end())
	{
		return nullptr;
	}
	cert_def = it->second;

	return cert_def;
}

bool ATECC608_TNGTLS_iface::read_device_certificate(Botan::X509_Certificate* const out_cert)
{
	if( ! out_cert )
	{
		return false;
	}

	// Slot 10 has compressed cert, 72B long
	// also see atcacert_read_cert_ext
	// this should be the cert for the slot0 key
	// see tng_atcacert_read_device_cert

	atcacert_def_t const * const cert_def = static_cast<atcacert_def_t const *>(get_device_cert_def());
	if( ! cert_def )
	{
		return false;
	}

	// Read signer key
	std::array<uint8_t, 72> ca_public_key;
	{
		ATCA_STATUS ret = atcacert_read_device_loc_ext(m_dev, &cert_def->ca_cert_def->public_key_dev_loc, ca_public_key.data());
		if(ret != ATCACERT_E_SUCCESS)
		{
			return false;
		}

		if (cert_def->ca_cert_def->public_key_dev_loc.count == 72U)
		{
			atcacert_public_key_remove_padding(ca_public_key.data(), ca_public_key.data());
		}
	}

	size_t max_cert_size = 0;
	ATCA_STATUS ret = atcacert_max_cert_size(cert_def, &max_cert_size);
	if(ret != ATCACERT_E_SUCCESS)
	{
		return false;
	}

	std::vector<uint8_t> device_cert_der(max_cert_size);

	ret = atcacert_read_cert_ext(m_dev, cert_def, ca_public_key.data(), device_cert_der.data(), &max_cert_size);
	if(ret != ATCACERT_E_SUCCESS)
	{
		return false;
	}

	device_cert_der.resize(max_cert_size);

	*out_cert = Botan::X509_Certificate(device_cert_der);

	return true;
}
bool ATECC608_TNGTLS_iface::read_signer_certificate(Botan::X509_Certificate* const out_cert)
{
	if( ! out_cert )
	{
		return false;
	}

	// Slot 11 has pubkey
	// Slot 12 has compressed cert
	// see tng_atcacert_read_signer_cert

	size_t max_cert_size = 0;
	int ret = tng_atcacert_max_signer_cert_size(&max_cert_size);
	if(ret != ATCACERT_E_SUCCESS)
	{
		SPDLOG_ERROR("tng_atcacert_max_signer_cert_size: {:d}", ret);
		return false;
	}

	atcacert_def_t const * const cert_def = static_cast<atcacert_def_t const *>(get_device_cert_def());
	if( ! cert_def )
	{
		return false;
	}

	atcacert_def_t const * const ca_cert_def = cert_def->ca_cert_def;
	uint8_t const * ca_public_key = &g_cryptoauth_root_ca_002_cert[CRYPTOAUTH_ROOT_CA_002_PUBLIC_KEY_OFFSET];

	std::vector<uint8_t> sig_cert_der(max_cert_size);

	ret = atcacert_read_cert_ext(m_dev, ca_cert_def, ca_public_key, sig_cert_der.data(), &max_cert_size);
	if(ret != ATCACERT_E_SUCCESS)
	{
		SPDLOG_ERROR("atcacert_read_cert_ext: {:d}", ret);
		return false;
	}
	sig_cert_der.resize(max_cert_size);

	*out_cert = Botan::X509_Certificate(sig_cert_der);

	return true;
}
bool ATECC608_TNGTLS_iface::read_root_certificate(Botan::X509_Certificate* const out_cert)
{
	if( ! out_cert )
	{
		return false;
	}

	// root cert is compiled into the cryptoauthlib library

	size_t max_cert_size = 0;
	int ret = tng_atcacert_root_cert_size(&max_cert_size);
	if(ret != ATCACERT_E_SUCCESS)
	{
		SPDLOG_ERROR("tng_atcacert_root_cert_size: {:d}", ret);
		return false;
	}

	std::vector<uint8_t> root_cert_der(max_cert_size);
	ret = tng_atcacert_root_cert(root_cert_der.data(), &max_cert_size);
	if(ret != ATCACERT_E_SUCCESS)
	{
		SPDLOG_ERROR("tng_atcacert_root_cert: {:d}", ret);
		return false;
	}
	root_cert_der.resize(max_cert_size);

	*out_cert = Botan::X509_Certificate(root_cert_der);

	return true;
}

std::vector<uint8_t> ATECC608_TNGTLS_iface::get_cert_chain_der() const
{
	std::vector<uint8_t> tmpvec;
	tmpvec.reserve(2048);

	Botan::DER_Encoder der(tmpvec);

	m_device_cert.encode_into(der);
	m_signer_cert.encode_into(der);
	m_root_cert.encode_into(der);

	return tmpvec;
}

std::string ATECC608_TNGTLS_iface::get_cert_chain_pem() const
{
	std::string chain;
	chain.reserve(4096);

	chain.append(m_device_cert.PEM_encode());
	chain.append(m_signer_cert.PEM_encode());
	chain.append(m_root_cert.PEM_encode());

	return chain;
}

ATECC608_info ATECC608_TNGTLS_iface::get_info() const
{
	ATECC608_info info;
	info.revision      = m_rev_cache;
	info.serial_number = m_sn_cache;
	info.config        = m_config_cache;
	info.otp0          = m_otp0_cache;
	info.otp1          = m_otp1_cache;

	std::shared_ptr<const Botan::EC_PublicKey> key = get_master_pubkey();
	if(key)
	{
		info.pubkeys.insert(
			std::make_pair(
				"MASTER",
				Botan::base64_encode(Botan::X509::BER_encode(*key))
			)
		);

		auto it = m_key_attestation.find(int(KEY_SLOT_ID::MASTER));
		if(it != m_key_attestation.end())
		{
			info.key_attestation.insert(
				std::make_pair(
					"MASTER",
					it->second
				)
			);
		}
	}

	key = get_attestation_pubkey();
	if(key)
	{
		info.pubkeys.insert(
			std::make_pair(
				"ATTESTATION",
				ATECC_Botan_util::pubkey_to_x509ber_b64(*key)
			)
		);
	}

	key = get_user0_pubkey();
	if(key)
	{
		info.pubkeys.insert(
			std::make_pair(
				"USER0",
				ATECC_Botan_util::pubkey_to_x509ber_b64(*key)
			)
		);

		auto it = m_key_attestation.find(int(KEY_SLOT_ID::USER0));
		if(it != m_key_attestation.end())
		{
			info.key_attestation.insert(
				std::make_pair(
					"USER0",
					it->second
				)
			);
		}
	}

	key = get_user1_pubkey();
	if(key)
	{
		info.pubkeys.insert(
			std::make_pair(
				"USER1",
				ATECC_Botan_util::pubkey_to_x509ber_b64(*key)
			)
		);

		auto it = m_key_attestation.find(int(KEY_SLOT_ID::USER1));
		if(it != m_key_attestation.end())
		{
			info.key_attestation.insert(
				std::make_pair(
					"USER1",
					it->second
				)
			);
		}
	}
	
	key = get_user2_pubkey();
	if(key)
	{
		info.pubkeys.insert(
			std::make_pair(
				"USER2",
				ATECC_Botan_util::pubkey_to_x509ber_b64(*key)
			)
		);

		auto it = m_key_attestation.find(int(KEY_SLOT_ID::USER2));
		if(it != m_key_attestation.end())
		{
			info.key_attestation.insert(
				std::make_pair(
					"USER2",
					it->second
				)
			);
		}
	}

	info.device_cert = ATECC_Botan_util::x509_to_der_b64(m_device_cert);
	info.signer_cert = ATECC_Botan_util::x509_to_der_b64(m_signer_cert);
	info.root_cert   = ATECC_Botan_util::x509_to_der_b64(m_root_cert);

	return info;
}

bool ATECC608_TNGTLS_iface::get_signed_info(Envelope_signature_with_nonce* const out_sig)
{
	if( ! out_sig )
	{
		return false;
	}

	ATECC608_info info = get_info();
	out_sig->msg       = info.to_cbor();

	if( ! sign_ext_with_nonce(out_sig->msg, uint16_t(KEY_SLOT_ID::MASTER), &out_sig->sig) )
	{
		return false;
	}

	return true;
}

bool ATECC608_TNGTLS_iface::generate_master_ca_cert(Botan::X509_Certificate* const out_cert)
{
	if( ! out_cert )
	{
		return false;
	}

	std::shared_ptr<const Botan::ECDSA_PublicKey> master_pub_key = get_master_pubkey();
	if( ! master_pub_key )
	{
		return false;
	}

	ATECC608_ECDSA_PrivateKey master_priv_key(*this, 0, master_pub_key);

	// TODO: long term, we want to set this to more like 30 years
	// Debian on armhf haas 32b time_t
	// Switch to openembedded or debian 13
	uint32_t expire_time = 0;
	if(sizeof(time_t) < 8)
	{
		expire_time = 12*365*24*60*60;
	}
	else
	{
		expire_time = 30*365*24*60*60;
	}

	Botan::X509_Cert_Options ca_cert_opt(
		"",
		expire_time
	);
	ca_cert_opt.common_name  = get_cached_sn_str();
	ca_cert_opt.dns          = get_dns_name_sn();
	ca_cert_opt.more_dns     = { get_dns_name_sn("ca") };
	ca_cert_opt.CA_key(0);

	// clamp start to device cert time
	if(ca_cert_opt.start < m_device_cert.not_before())
	{
		ca_cert_opt.start = m_device_cert.not_before();
	}
	if(m_device_cert.not_after() < ca_cert_opt.end)
	{
		ca_cert_opt.end = m_device_cert.not_after();
	}

	*out_cert = Botan::X509::create_self_signed_cert(ca_cert_opt, master_priv_key, "SHA-256", get_sys_rng());

	if( ! out_cert->check_signature(*master_pub_key) )
	{
		SPDLOG_ERROR("out_cert->check_signature failed");
		return false;
	}

	SPDLOG_DEBUG("Generated MASTER cert:\n{:s}", out_cert->to_string());

	return true;
}
bool ATECC608_TNGTLS_iface::generate_user0_cert(Botan::X509_CA& master_ca, Botan::X509_Certificate* const out_cert)
{
	if( ! out_cert )
	{
		return false;
	}

	std::shared_ptr<const Botan::EC_PublicKey> user_pub_key = get_user0_pubkey();
	if( ! user_pub_key )
	{
		return false;
	}
	ATECC608_ECDSA_PrivateKey user_priv_key(*this, uint16_t(KEY_SLOT_ID::USER0), user_pub_key);

	Botan::X509_Cert_Options user_cert_opt(
		"",
		1*365*24*60*60
	);
	user_cert_opt.common_name  = get_cached_sn_str();
	user_cert_opt.dns          = get_dns_name_sn();
	user_cert_opt.more_dns     = { get_dns_name_sn("user0") };
	user_cert_opt.organization = "Suburban Marine Inc";
	Botan::Key_Constraints ca_cert_constraints = Botan::Key_Constraints(
		Botan::Key_Constraints::DIGITAL_SIGNATURE
	);
	user_cert_opt.add_constraints(ca_cert_constraints);

	// clamp to device cert time
	if(user_cert_opt.start < m_device_cert.not_before())
	{
		user_cert_opt.start = m_device_cert.not_before();
	}	
	if(m_device_cert.not_after() < user_cert_opt.end)
	{
		user_cert_opt.end = m_device_cert.not_after();
	}

	Botan::PKCS10_Request user_req = Botan::X509::create_cert_req(user_cert_opt, user_priv_key, "SHA-256", get_sys_rng());
	*out_cert = master_ca.sign_request(user_req, get_sys_rng(), user_cert_opt.start, user_cert_opt.end);

	if( ! out_cert->check_signature(master_ca.ca_certificate().subject_public_key()) )
	{
		SPDLOG_ERROR("out_cert->check_signature failed");
		return false;
	}

	SPDLOG_DEBUG("Generated USER0 cert:\n{:s}", out_cert->to_string());

	return true;
}
bool ATECC608_TNGTLS_iface::generate_user1_cert(Botan::X509_CA& master_ca, Botan::X509_Certificate* const out_cert)
{
	if( ! out_cert )
	{
		return false;
	}

	std::shared_ptr<const Botan::EC_PublicKey> user_pub_key = get_user1_pubkey();
	if( ! user_pub_key )
	{
		return false;
	}
	ATECC608_ECDSA_PrivateKey user_priv_key(*this, uint16_t(KEY_SLOT_ID::USER1), user_pub_key);

	Botan::X509_Cert_Options user_cert_opt(
		"",
		1*365*24*60*60
	);
	user_cert_opt.common_name  = get_cached_sn_str();
	user_cert_opt.dns          = get_dns_name_sn();
	user_cert_opt.more_dns     = { get_dns_name_sn("user1") };
	user_cert_opt.organization = "Suburban Marine Inc";
	Botan::Key_Constraints ca_cert_constraints = Botan::Key_Constraints(
		Botan::Key_Constraints::DIGITAL_SIGNATURE
	);
	user_cert_opt.add_constraints(ca_cert_constraints);

	// clamp to device cert time
	if(user_cert_opt.start < m_device_cert.not_before())
	{
		user_cert_opt.start = m_device_cert.not_before();
	}	
	if(m_device_cert.not_after() < user_cert_opt.end)
	{
		user_cert_opt.end = m_device_cert.not_after();
	}

	Botan::PKCS10_Request user_req = Botan::X509::create_cert_req(user_cert_opt, user_priv_key, "SHA-256", get_sys_rng());
	*out_cert = master_ca.sign_request(user_req, get_sys_rng(), user_cert_opt.start, user_cert_opt.end);

	if( ! out_cert->check_signature(master_ca.ca_certificate().subject_public_key()) )
	{
		SPDLOG_ERROR("out_cert->check_signature failed");
		return false;
	}

	SPDLOG_DEBUG("Generated USER1 cert:\n{:s}", out_cert->to_string());

	return true;
}
bool ATECC608_TNGTLS_iface::generate_user2_cert(Botan::X509_CA& master_ca, Botan::X509_Certificate* const out_cert)
{
	if( ! out_cert )
	{
		return false;
	}

	std::shared_ptr<const Botan::EC_PublicKey> user_pub_key = get_user2_pubkey();
	if( ! user_pub_key )
	{
		return false;
	}
	ATECC608_ECDSA_PrivateKey user_priv_key(*this, uint16_t(KEY_SLOT_ID::USER2), user_pub_key);

	Botan::X509_Cert_Options user_cert_opt(
		"",
		1*365*24*60*60
	);
	user_cert_opt.common_name  = get_cached_sn_str();
	user_cert_opt.dns          = get_dns_name_sn();
	user_cert_opt.more_dns     = { get_dns_name_sn("user2") };
	user_cert_opt.organization = "Suburban Marine Inc";
	Botan::Key_Constraints ca_cert_constraints = Botan::Key_Constraints(
		Botan::Key_Constraints::DIGITAL_SIGNATURE
		// Botan::Key_Constraints::DIGITAL_SIGNATURE | 
		// Botan::Key_Constraints::KEY_AGREEMENT
	);
	user_cert_opt.add_constraints(ca_cert_constraints);

	// clamp to device cert time
	if(user_cert_opt.start < m_device_cert.not_before())
	{
		user_cert_opt.start = m_device_cert.not_before();
	}	
	if(m_device_cert.not_after() < user_cert_opt.end)
	{
		user_cert_opt.end = m_device_cert.not_after();
	}

	Botan::PKCS10_Request user_req = Botan::X509::create_cert_req(user_cert_opt, user_priv_key, "SHA-256", get_sys_rng());
	*out_cert = master_ca.sign_request(user_req, get_sys_rng(), user_cert_opt.start, user_cert_opt.end);

	if( ! out_cert->check_signature(master_ca.ca_certificate().subject_public_key()) )
	{
		SPDLOG_ERROR("out_cert->check_signature failed");
		return false;
	}

	SPDLOG_DEBUG("Generated USER2 cert:\n{:s}", out_cert->to_string());

	return true;
}

bool ATECC608_TNGTLS_iface::generate_user0_cert(Botan::X509_Certificate* const out_cert)
{
	if( ! master_ca_cert )
	{
		return false;
	}

	ATECC608_ECDSA_PrivateKey master_priv_key(*this, 0, get_master_pubkey());
	Botan::X509_CA master_ca(*master_ca_cert, master_priv_key, "SHA-256", m_rng);

	return generate_user0_cert(master_ca, out_cert);
}
bool ATECC608_TNGTLS_iface::generate_user1_cert(Botan::X509_Certificate* const out_cert)
{
	if(! master_ca_cert )
	{
		return false;
	}

	ATECC608_ECDSA_PrivateKey master_priv_key(*this, 0, get_master_pubkey());
	Botan::X509_CA master_ca(*master_ca_cert, master_priv_key, "SHA-256", m_rng);

	return generate_user1_cert(master_ca, out_cert);
}
bool ATECC608_TNGTLS_iface::generate_user2_cert(Botan::X509_Certificate* const out_cert)
{
	if(! master_ca_cert )
	{
		return false;
	}

	ATECC608_ECDSA_PrivateKey master_priv_key(*this, 0, get_master_pubkey());
	Botan::X509_CA master_ca(*master_ca_cert, master_priv_key, "SHA-256", m_rng);

	return generate_user2_cert(master_ca, out_cert);
}

bool ATECC608_TNGTLS_iface::load_master_ca_cert(const std::string& path)
{
	std::vector<uint8_t> ca_cert_der;
	if( ! File_util::readSmallFile(path, &ca_cert_der) )
	{
		SPDLOG_ERROR("Unable to read to {:s}", path);
		return false;
	}

	return load_master_ca_cert(ca_cert_der);
}

bool ATECC608_TNGTLS_iface::load_master_ca_cert(const std::vector<uint8_t>& ca_cert_der)
{
	if(ca_cert_der.empty())
	{
		SPDLOG_ERROR("Error loading provided ca cert");
		return false;
	}

	std::shared_ptr<Botan::X509_Certificate> ca_cert;
	try
	{
		ca_cert = std::make_shared<Botan::X509_Certificate>(ca_cert_der);
	}
	catch(const std::exception& e)
	{
		SPDLOG_ERROR("Error loading key: {:s}", e.what());
		return false;
	}

	if(! ca_cert )
	{
		SPDLOG_ERROR("Could not load ca_cert");
		return false;		
	}

	return load_master_ca_cert(ca_cert);
}
bool ATECC608_TNGTLS_iface::load_master_ca_cert(const std::shared_ptr<Botan::X509_Certificate>& ca_cert)
{
	if( ! ca_cert->is_CA_cert() )
	{
		SPDLOG_ERROR("not a CA cert");
		return false;		
	}

	// check CN
	std::vector<std::string> cn_vec = ca_cert->subject_info("X520.CommonName");
	if(cn_vec.size() != 1)
	{
		SPDLOG_ERROR("ca_cert CN size wrong");
		return false;		
	}
	if(cn_vec[0] != fmt::format("sn{:02X}", fmt::join(get_cached_sn(), "")))
	{
		SPDLOG_ERROR("ca_cert CN does not match");
		return false;		
	}

	// check pubkey
	std::shared_ptr<const Botan::ECDSA_PublicKey> master_pubkey = get_master_pubkey();
	if( ! master_pubkey )
	{
		return false;
	}

	Botan::ECDSA_PublicKey* ca_cert_pubkey = dynamic_cast<Botan::ECDSA_PublicKey*>(ca_cert->subject_public_key());
	if( ! ca_cert_pubkey )
	{
		SPDLOG_ERROR("ca_cert pubkey not a ECDSA_PublicKey");
		return false;		
	}
	if(ca_cert_pubkey->domain() != master_pubkey->domain())
	{
		SPDLOG_ERROR("ca_cert pubkey domain does not match");
		return false;
	}
	if(ca_cert_pubkey->public_point() != master_pubkey->public_point())
	{
		SPDLOG_ERROR("ca_cert pubkey public_point does not match");
		return false;
	}
	
	// check sig valid
	Botan::Certificate_Status_Code sig_status = ca_cert->verify_signature(*master_pubkey);
	if( sig_status != Botan::Certificate_Status_Code::OK )
	{
		SPDLOG_ERROR("ca_cert->verify_signature failed: {:s}", Botan::to_string(sig_status));
		return false;
	}

	if( ! ca_cert->is_CA_cert() )
	{
		SPDLOG_ERROR("ca_cert is not actually a ca_cert");
		return false;
	}

	// check timestamp
	const auto t_0 = std::chrono::system_clock::now();
	if(t_0 < ca_cert->not_before().to_std_timepoint())
	{
		SPDLOG_ERROR("ca_cert timestamp invalid");
		return false;
	}
	if(ca_cert->not_after().to_std_timepoint() < t_0)
	{
		SPDLOG_ERROR("ca_cert timestamp invalid");
		return false;
	}

	master_ca_cert = ca_cert;
	m_local_cert_store.add_certificate(ca_cert);

	SPDLOG_DEBUG("Loaded master cert:\n{:s}", master_ca_cert->to_string());
	
	return true;
}

bool ATECC608_TNGTLS_iface::load_user_cert(const KEY_SLOT_ID& slot, const std::vector<uint8_t>& cert_der)
{
	if(cert_der.empty())
	{
		SPDLOG_WARN("Error loading provided user cert");
		return false;
	}

	if( ! master_ca_cert )
	{
		SPDLOG_WARN("master_ca_cert is null");
		return false;
	}

	std::shared_ptr<const Botan::ECDSA_PublicKey> cached_user_cert_pubkey;
	switch(slot)
	{
		case KEY_SLOT_ID::USER0:
		{
			cached_user_cert_pubkey = get_user0_pubkey();
			break;
		}
		case KEY_SLOT_ID::USER1:
		{
			cached_user_cert_pubkey = get_user1_pubkey();
			break;
		}
		case KEY_SLOT_ID::USER2:
		{
			cached_user_cert_pubkey = get_user2_pubkey();
			break;
		}
		default:
		{
			cached_user_cert_pubkey.reset();
			break;
		}
	}

	if( ! cached_user_cert_pubkey )
	{
		SPDLOG_WARN("Error loading cached pubkey");
		return false;
	}

	std::shared_ptr<Botan::X509_Certificate> user_cert;
	try
	{
		user_cert = std::make_shared<Botan::X509_Certificate>(cert_der);
	}
	catch(const std::exception& e)
	{
		SPDLOG_WARN("Error loading key: {:s}", e.what());
		return false;
	}
	
	if( ! user_cert )
	{
		SPDLOG_WARN("Error loading provided user cert");
		return false;
	}

	// check pubkey
	Botan::ECDSA_PublicKey* user_cert_pubkey = dynamic_cast<Botan::ECDSA_PublicKey*>(user_cert->subject_public_key());
	if( ! user_cert_pubkey )
	{
		SPDLOG_WARN("user_cert pubkey not a ECDSA_PublicKey");
		return false;		
	}
	if(user_cert_pubkey->domain() != cached_user_cert_pubkey->domain())
	{
		SPDLOG_WARN("user_cert pubkey domain does not match");
		return false;
	}
	if(user_cert_pubkey->public_point() != cached_user_cert_pubkey->public_point())
	{
		SPDLOG_WARN("user_cert pubkey public_point does not match");
		return false;
	}
	
	// check sig valid
	std::vector<Botan::X509_Certificate> chain = {*user_cert};
	Botan::Path_Validation_Result path_ret = Botan::x509_path_validate(
		chain,
		Botan::Path_Validation_Restrictions(false, 128),
		m_local_cert_store,
		get_dns_name_sn()
	);
	if( ! path_ret.successful_validation() )
	{
		SPDLOG_WARN("user_cert x509_path_validate failed: {:s}", path_ret.result_string());
		return false;
	}

	user_cert_cache.insert_or_assign(slot, user_cert);

	return true;
}

std::string ATECC608_TNGTLS_iface::get_dns_name_sn() const
{
	if(m_second_level_domain.empty())
	{
		return get_cached_sn_str();
	}

	return fmt::format("{:s}.{:s}", get_cached_sn_str(), m_second_level_domain);
}
std::string ATECC608_TNGTLS_iface::get_dns_name_sn(const std::string& subdomain) const
{
	if(m_second_level_domain.empty())
	{
		return fmt::format("{:s}.{:s}", subdomain, get_cached_sn_str());
	}

	return fmt::format("{:s}.{:s}.{:s}", subdomain, get_cached_sn_str(), m_second_level_domain);
}
