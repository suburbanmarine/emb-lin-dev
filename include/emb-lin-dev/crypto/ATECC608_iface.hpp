/**
 * This file is part of emb-lin-dev, mostly a collection of userspace drivers and helper utilities for embedded linux.
 * 
 * @author Jacob Schloss <jacob.schloss@suburbanmarine.io>
 * @copyright Copyright (c) 2024 Suburban Marine, Inc. All rights reserved.
 * @license Licensed under the LGPL-3.0 license. See LICENSE.txt for details.
 * SPDX-License-Identifier: LGPL-3.0-only
*/

#pragma once

#include "emb-lin-util/JSON_CBOR_helper.hpp"

#include <cryptoauthlib.h>

#include <botan/auto_rng.h>
#include <botan/ecdsa.h>
#include <botan/pubkey.h>

#include <boost/core/noncopyable.hpp>

#include <nlohmann/json.hpp>

#include <array>
#include <stdexcept>
#include <span>

#include <cstdint>

class Signature_with_nonce : public JSON_CBOR_helper<Signature_with_nonce>
{
public:
	uint16_t key_id;
	std::array<uint8_t, 20> hostnonce;
	std::array<uint8_t, 32> devnonce; // devrand might be a better name
	std::array<uint8_t, 32> msg;      // message that was signed, eg the sha-256 hash of a larger message
	std::array<uint8_t, 64> sig;
};
void to_json(nlohmann::json& j, const Signature_with_nonce& val);
void from_json(const nlohmann::json& j, Signature_with_nonce& val);

class KeyFingerprintSig : public JSON_CBOR_helper<KeyFingerprintSig>
{
public:
	uint16_t keyid_to_sign;
	uint16_t signing_keyid;
	std::array<uint8_t, 20> hostnonce;
	std::array<uint8_t, 32> devnonce; // devrand might be a better name
	std::array<uint8_t, 64> pubkey;   // pubkey of keyid_to_sign
	std::array<uint8_t, 64> sig;
};
void to_json(nlohmann::json& j, const KeyFingerprintSig& val);
void from_json(const nlohmann::json& j, KeyFingerprintSig& val);

// TODO replace with Botan::X509_Object::make_signed
class Envelope_signature_with_nonce : public JSON_CBOR_helper<Envelope_signature_with_nonce>
{
public:
	std::vector<uint8_t> msg; // message as b64
	Signature_with_nonce sig; // signature blob, sig.msg is sha256(this->msg)
};
void to_json(nlohmann::json& j, const Envelope_signature_with_nonce& val);
void from_json(const nlohmann::json& j, Envelope_signature_with_nonce& val);

class ATECC608_iface : private boost::noncopyable
{
public:
	ATECC608_iface();
	virtual ~ATECC608_iface();

	virtual bool init(const uint8_t bus, const uint8_t address);
	virtual bool close();

	bool read_pubkey(const uint16_t key_id, std::array<uint8_t, 64>* const out_pubkey);
	bool read_pubkey(const uint16_t key_id, std::shared_ptr<Botan::ECDSA_PublicKey>* const out_pubkey);

	bool read_rand(std::array<uint8_t, 32>* const out_random);
	bool read_rand_feed_sw_rng();

	bool read_counter(uint16_t counter_id, uint32_t* out_val);
	bool increment_counter(uint16_t counter_id, uint32_t* out_val);

	bool read_info(std::array<uint8_t, 4>* const out_revision);
	bool read_serial_number(std::array<uint8_t, 9>* const out_sn);
	bool read_config_zone(std::array<uint8_t, 128>* const out_config);

	// block in [0, 1]
	bool read_otp_zone(const uint8_t block, std::array<uint8_t, 32>* const out_otp);

	bool write_updateextra(const uint8_t mode, const uint16_t new_value);

	bool wake();
	bool idle();
	bool sleep();
	bool selftest(uint8_t* const out_test);

	// sign on device, no extra device state or nonce mixed in
	bool sign_raw(const std::array<uint8_t, 32>& msg, const uint16_t key_id, std::array<uint8_t, 64>* const out_sig);
	// verify on host
	static bool verify_raw(const std::array<uint8_t, 32>& msg, const std::array<uint8_t, 64>& sig, const Botan::Public_Key& pubkey);

	// sign on device, mix in nonce and internal state
	// key must allow internal sig
	bool sign_with_nonce(const std::array<uint8_t, 32>& msg, const uint16_t key_id, Signature_with_nonce* const out_sig);
	// verify on host
	static bool verify_with_nonce(const std::array<uint8_t, 9>& sn, const std::array<uint8_t, 128>& config, const Signature_with_nonce& sig_data, const Botan::Public_Key& signing_pubkey);
	bool verify_with_nonce(const Signature_with_nonce& sig, const Botan::Public_Key& signing_pubkey)
	{
		return verify_with_nonce(get_cached_sn(), get_cached_config(), sig, signing_pubkey);
	}

	// sign hash of message on device, mix in nonce and sn
	// key must allow external sig
	bool sign_ext_with_nonce(const std::span<uint8_t>& msg, const uint16_t key_id, Signature_with_nonce* const out_sig);
	// verify on host
	static bool verify_ext_with_nonce(const std::span<uint8_t>& msg, const std::array<uint8_t, 9>& sn, const std::array<uint8_t, 128>& config, const Signature_with_nonce& sig_data, const Botan::Public_Key& signing_pubkey);
	bool verify_ext_with_nonce(const std::span<uint8_t>& msg, const Signature_with_nonce& sig, const Botan::Public_Key& signing_pubkey)
	{
		return verify_ext_with_nonce(msg, get_cached_sn(), get_cached_config(), sig, signing_pubkey);
	}

	bool ecdh(const uint16_t key_id, std::shared_ptr<const Botan::EC_PublicKey> ext_pubkey, std::array<uint8_t, 32>* const pms);

	bool hmac_sha256(const uint16_t key_id, const std::vector<uint8_t>& msg, std::array<uint8_t, 32>* const out_digest);
	static bool verify_hmac_sha256(const Botan::secure_vector<uint8_t>& key, const std::vector<uint8_t>& msg, const std::array<uint8_t, 32>& digest);

	// sign a fingerprint of a pubkey using a certain privkey on device, mix in internal state
	// aka genkey GENKEY_MODE_DIGEST and sign_internal
	bool sign_key_fingerprint(const uint16_t keyid_to_sign, const uint16_t signing_keyid, KeyFingerprintSig* const out_data);
	// given fingerprint context regenerate signature message & digest and verify on host
	static bool verify_key_fingerprint_signature(const std::array<uint8_t, 9>& sn, const std::array<uint8_t, 128>& config, const KeyFingerprintSig& sig_data, const Botan::EC_PublicKey& signing_pubkey);
	bool verify_key_fingerprint_signature(const KeyFingerprintSig& sig_data, const Botan::EC_PublicKey& signing_pubkey)
	{
		return verify_key_fingerprint_signature(get_cached_sn(), get_cached_config(), sig_data, signing_pubkey);
	}

	ATCADevice get_dev()
	{
		return m_dev;
	}

	Botan::AutoSeeded_RNG& get_sys_rng()
	{
		return m_rng;
	}

	const std::array<uint8_t, 9>& get_cached_sn() const
	{
		return m_sn_cache;
	}

	const std::array<uint8_t, 128>& get_cached_config() const
	{
		return m_config_cache;
	}

	static uint16_t get_slotLocked(const std::array<uint8_t, 128>& config)
	{
		return (uint16_t(config[89]) << 8) | uint16_t(config[88]);
	}

	static uint16_t get_slotConfig(const std::array<uint8_t, 128>& config, const uint16_t key_id)
	{
		if(key_id > 15)
		{
			throw std::domain_error("key_id must be in [0, 15]");
		}

		return (uint16_t(config[20 + key_id*2 + 1]) << 8) | (uint16_t(config[20 + key_id*2 + 0]) << 0);
	}

	static uint16_t get_keyConfig(const std::array<uint8_t, 128>& config, const uint16_t key_id)
	{
		if(key_id > 15)
		{
			throw std::domain_error("key_id must be in [0, 15]");
		}

		return (uint16_t(config[96 + key_id*2 + 1]) << 8) | (uint16_t(config[96 + key_id*2 + 0]) << 0);
	}

	uint16_t get_cached_slotLocked() const
	{
		return get_slotLocked(m_config_cache);
	}

	uint16_t get_cached_slotConfig(const uint16_t key_id) const
	{
		return get_slotConfig(m_config_cache, key_id);
	}

	uint16_t get_cached_keyConfig(const uint16_t key_id) const
	{
		return get_keyConfig(m_config_cache, key_id);
	}

	template<size_t LEN>
	static void copy_vec_to_arr(const std::vector<uint8_t>& v, std::array<uint8_t, LEN>& a)
	{
		if(v.size() == LEN)
		{
			std::copy_n(v.data(), LEN, a.data());
		}
		else
		{
			throw std::out_of_range("LEN mismatch");
		}
	}

protected:

	ATCAIfaceCfg m_cfg;
	ATCADevice   m_dev;

	std::array<uint8_t, 4>   m_rev_cache;
	std::array<uint8_t, 9>   m_sn_cache;
	std::array<uint8_t, 128> m_config_cache;
	std::array<uint8_t, 32>  m_otp0_cache;
	std::array<uint8_t, 32>  m_otp1_cache;

	Botan::AutoSeeded_RNG m_rng;
};
