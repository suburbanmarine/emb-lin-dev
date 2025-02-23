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

#include "emb-lin-dev/crypto/ATECC608_iface.hpp"

#include <botan/x509cert.h>
#include <botan/x509_ca.h>

#include <atcacert/atcacert_def.h>

#include <optional>

class ATECC608_info : public JSON_CBOR_helper<ATECC608_info>
{
public:
	std::array<uint8_t, 4>   revision;      // b64
	std::array<uint8_t, 9>   serial_number; // b64
	std::array<uint8_t, 128> config;        // config area bytes [0, 127] b64
	std::array<uint8_t, 32> otp0;          // otp area bytes [0, 31] b64 (the id of signer_cert)
	std::array<uint8_t, 32> otp1;          // otp area bytes [32, 63] b64 (all zeros)
	std::map<std::string, std::string> pubkeys; // name, pubkey X509 BER b64 (not full cert, just the pubkey)
	std::map<std::string, KeyFingerprintSig> key_attestation; // name, pubkey attestation cbor as b64
	std::string device_cert; // X509 pubkey cert DER b64, for master key in slot0
	std::string signer_cert; // X509 pubkey cert DER b64, signs device_cert
	std::string root_cert;   // X509 pubkey cert DER b64, signs signer_cert
};
void to_json(nlohmann::json& j, const ATECC608_info& val);
void from_json(const nlohmann::json& j, ATECC608_info& val);

class ATECC608_TNGTLS_iface : public ATECC608_iface
{
public:
	ATECC608_TNGTLS_iface()
	{

	}
	~ATECC608_TNGTLS_iface() override
	{

	}

	bool init(const uint8_t bus, const uint8_t address) override;

	enum class KEY_SLOT_ID : uint16_t
	{
		MASTER      = 0, // ECDSA or ECDH
		ATTESTATION = 1, // ECDSA
		USER0       = 2, // ECDSA or ECDH
		USER1       = 3, // ECDSA or ECDH
		USER2       = 4, // ECDSA or ECDH
		SIG_PUB     = 11
	};

	enum class CERT_SLOT_ID : uint16_t
	{
		DEV_CERT    = 10,
		SIG_CERT    = 12
	};

	// Update config byte 85 to a new i2c address
	// Only writable iff b85 == 0, eg is a OTP byte for TNGTLS
	bool write_otp_i2c_address(const uint8_t addr);

	// write hmac or aes key
	bool write_secret_key(const uint8_t slotID, const uint8_t offset, const std::array<uint8_t, 16>& key);

	std::shared_ptr<const Botan::ECDSA_PublicKey> get_master_pubkey() const
	{
		auto it = m_pubkeys.find(int(KEY_SLOT_ID::MASTER));
		if(it == m_pubkeys.end())
		{
			return std::shared_ptr<const Botan::ECDSA_PublicKey>();
		}

		return std::dynamic_pointer_cast<const Botan::ECDSA_PublicKey>(it->second);
	}
	std::shared_ptr<const Botan::ECDSA_PublicKey> get_attestation_pubkey() const
	{
		auto it = m_pubkeys.find(int(KEY_SLOT_ID::ATTESTATION));
		if(it == m_pubkeys.end())
		{
			return std::shared_ptr<const Botan::ECDSA_PublicKey>();
		}

		return std::dynamic_pointer_cast<const Botan::ECDSA_PublicKey>(it->second);
	}
	std::shared_ptr<const Botan::ECDSA_PublicKey> get_user0_pubkey() const
	{
		auto it = m_pubkeys.find(int(KEY_SLOT_ID::USER0));
		if(it == m_pubkeys.end())
		{
			return std::shared_ptr<const Botan::ECDSA_PublicKey>();
		}

		return std::dynamic_pointer_cast<const Botan::ECDSA_PublicKey>(it->second);
	}
	std::shared_ptr<const Botan::ECDSA_PublicKey> get_user1_pubkey() const // ECDH_PublicKey
	{
		auto it = m_pubkeys.find(int(KEY_SLOT_ID::USER1));
		if(it == m_pubkeys.end())
		{
			return std::shared_ptr<const Botan::ECDSA_PublicKey>();
		}

		return std::dynamic_pointer_cast<const Botan::ECDSA_PublicKey>(it->second);
	}
	std::shared_ptr<const Botan::ECDSA_PublicKey> get_user2_pubkey() const
	{
		auto it = m_pubkeys.find(int(KEY_SLOT_ID::USER2));
		if(it == m_pubkeys.end())
		{
			return std::shared_ptr<const Botan::ECDSA_PublicKey>();
		}

		return std::dynamic_pointer_cast<const Botan::ECDSA_PublicKey>(it->second);
	}
	std::shared_ptr<const Botan::ECDSA_PublicKey> get_sig_pubkey() const
	{
		auto it = m_pubkeys.find(int(KEY_SLOT_ID::SIG_PUB));
		if(it == m_pubkeys.end())
		{
			return std::shared_ptr<const Botan::ECDSA_PublicKey>();
		}

		return std::dynamic_pointer_cast<const Botan::ECDSA_PublicKey>(it->second);
	}

	bool read_device_certificate(Botan::X509_Certificate* const out_cert);
	bool read_signer_certificate(Botan::X509_Certificate* const out_cert);
	static bool read_root_certificate(Botan::X509_Certificate* const out_cert);

	const Botan::X509_Certificate& get_device_certificate() const
	{
		return m_device_cert;
	}
	const Botan::X509_Certificate& get_signer_certificate() const
	{
		return m_signer_cert;
	}
	const Botan::X509_Certificate& get_root_certificate() const
	{
		return m_root_cert;
	}

	std::vector<uint8_t> get_cert_chain_der() const;
	std::string          get_cert_chain_pem() const;

	// get x509 format pubkeys, x509 cert for master to mcp root
	ATECC608_info get_info() const;
	// like get_info(), with hash signed by master
	bool get_signed_info(Envelope_signature_with_nonce* const out_sig);

	bool generate_master_ca_cert(Botan::X509_Certificate* out_cert);
	bool generate_user0_cert(Botan::X509_CA& master_ca, Botan::X509_Certificate* out_cert);
	bool generate_user1_cert(Botan::X509_CA& master_ca, Botan::X509_Certificate* out_cert);
	bool generate_user2_cert(Botan::X509_CA& master_ca, Botan::X509_Certificate* out_cert);

	// usually you want to call generate_master_ca_cert once and reuse the cert
	// load it to reuse here
	bool load_master_ca_cert(const std::string& path);
	bool load_master_ca_cert(const std::vector<uint8_t>& ca_cert_der);
	bool load_master_ca_cert(const std::shared_ptr<Botan::X509_Certificate>& ca_cert);

	std::shared_ptr<Botan::X509_Certificate> get_master_ca_cert()
	{
		return master_ca_cert;
	}

protected:

	std::shared_ptr<Botan::X509_Certificate> master_ca_cert;

	std::string get_device_cert_id() const;
	void const * get_device_cert_def(); // returns a atcacert_def_t

	std::map<int, std::shared_ptr<Botan::EC_PublicKey> > m_pubkeys; // TOOD are all of these ECDSA, or should one be used for ECDH and not signature
	std::map<int, KeyFingerprintSig > m_key_attestation;

	Botan::X509_Certificate m_device_cert;
	Botan::X509_Certificate m_signer_cert;
	Botan::X509_Certificate m_root_cert;

	static const std::map<std::string, atcacert_def_t const *> m_cert_def_lookup;
};
