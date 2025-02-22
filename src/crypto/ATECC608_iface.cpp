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

#include <emb-lin-dev/crypto/ATECC608_iface.hpp>

#include "host/atca_host.h"

#include <botan/base64.h>
#include <botan/ec_group.h>
#include <botan/hash.h>
#include <botan/mac.h>

#include <spdlog/spdlog.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <exception>
#include <thread>

void to_json(nlohmann::json& j, const Signature_with_nonce& val)
{
	j = nlohmann::json {
		{"key_id",    val.key_id},
		{"hostnonce", Botan::base64_encode(val.hostnonce.data(), val.hostnonce.size())},
		{"devnonce",  Botan::base64_encode(val.devnonce.data(),  val.devnonce.size())},
		{"msg",       Botan::base64_encode(val.msg.data(),       val.msg.size())},
		{"sig",       Botan::base64_encode(val.sig.data(),       val.sig.size())}
	};
}
void from_json(const nlohmann::json& j, Signature_with_nonce& val)
{
	std::string tempstr;
	std::vector<uint8_t> tempvec;

	j.at("key_id").get_to(val.key_id);
	
	j.at("hostnonce").get_to(tempstr);
	tempvec = Botan::unlock(Botan::base64_decode(tempstr));
	ATECC608_iface::copy_vec_to_arr(tempvec, val.hostnonce);

	j.at("devnonce").get_to(tempstr);
	tempvec = Botan::unlock(Botan::base64_decode(tempstr));
	ATECC608_iface::copy_vec_to_arr(tempvec, val.devnonce);
	
	j.at("msg").get_to(tempstr);
	tempvec = Botan::unlock(Botan::base64_decode(tempstr));
	ATECC608_iface::copy_vec_to_arr(tempvec, val.msg);

	j.at("sig").get_to(tempstr);
	tempvec = Botan::unlock(Botan::base64_decode(tempstr));
	ATECC608_iface::copy_vec_to_arr(tempvec, val.sig);
}
void to_json(nlohmann::json& j, const KeyFingerprintSig& val)
{
	j = nlohmann::json {
		{"keyid_to_sign", val.keyid_to_sign},
		{"signing_keyid", val.signing_keyid},
		{"hostnonce",     Botan::base64_encode(val.hostnonce.data(), val.hostnonce.size())},
		{"devnonce",      Botan::base64_encode(val.devnonce.data(), val.devnonce.size())},
		{"pubkey",        Botan::base64_encode(val.pubkey.data(), val.pubkey.size())},
		{"sig",           Botan::base64_encode(val.sig.data(), val.sig.size())}
	};
}
void from_json(const nlohmann::json& j, KeyFingerprintSig& val)
{
	std::string tempstr;
	std::vector<uint8_t> tempvec;

	j.at("keyid_to_sign").get_to(val.keyid_to_sign);
	j.at("signing_keyid").get_to(val.signing_keyid);

	j.at("hostnonce").get_to(tempstr);
	tempvec = Botan::unlock(Botan::base64_decode(tempstr));
	ATECC608_iface::copy_vec_to_arr(tempvec, val.hostnonce);

	j.at("devnonce").get_to(tempstr);
	tempvec = Botan::unlock(Botan::base64_decode(tempstr));
	ATECC608_iface::copy_vec_to_arr(tempvec, val.devnonce);

	j.at("pubkey").get_to(tempstr);
	tempvec = Botan::unlock(Botan::base64_decode(tempstr));
	ATECC608_iface::copy_vec_to_arr(tempvec, val.pubkey);

	j.at("sig").get_to(tempstr);
	tempvec = Botan::unlock(Botan::base64_decode(tempstr));
	ATECC608_iface::copy_vec_to_arr(tempvec, val.sig);
}

void to_json(nlohmann::json& j, const Envelope_signature_with_nonce& val)
{
	j = nlohmann::json {
		{"msg", Botan::base64_encode(val.msg)},
		{"sig", val.sig}
	};
}
void from_json(const nlohmann::json& j, Envelope_signature_with_nonce& val)
{
	std::string tempstr;

	j.at("msg").get_to(tempstr);
	val.msg = Botan::unlock(Botan::base64_decode(tempstr));

	j.at("sig").get_to(val.sig);
}

ATECC608_iface::ATECC608_iface()
{
	m_dev = nullptr;
	memset(&m_cfg, 0, sizeof(m_cfg));
}
ATECC608_iface::~ATECC608_iface()
{
	close();
}

bool ATECC608_iface::init(const uint8_t bus, const uint8_t address)
{
	if(m_dev != nullptr)
	{
		close();
	}

	memset(&m_cfg, 0, sizeof(m_cfg));

	m_cfg.devtype    = ATECC608;
#if 1
	m_cfg.iface_type = ATCA_I2C_IFACE;
	m_cfg.atcai2c.address = address << 1;
	m_cfg.atcai2c.bus     = bus;
	m_cfg.atcai2c.baud    = 100000U;
#else
	cfg.iface_type             = ATCA_CUSTOM_IFACE;
	cfg.atcacustom.halinit     = &ATECC608_iface::dispatch_crypto_hal_init;
	cfg.atcacustom.halpostinit = &ATECC608_iface::dispatch_crypto_hal_postinit;
	cfg.atcacustom.halsend     = &ATECC608_iface::dispatch_crypto_hal_send;
	cfg.atcacustom.halreceive  = &ATECC608_iface::dispatch_crypto_hal_receive;
	cfg.atcacustom.halwake     = &ATECC608_iface::dispatch_crypto_hal_wake;
	cfg.atcacustom.halidle     = &ATECC608_iface::dispatch_crypto_hal_idle;
	cfg.atcacustom.halsleep    = &ATECC608_iface::dispatch_crypto_hal_sleep;
	cfg.atcacustom.halrelease  = &ATECC608_iface::dispatch_crypto_hal_release;
	cfg.cfg_data               = this;
#endif
	m_cfg.wake_delay      = 1500; // self test off
	m_cfg.rx_retries      = 20;

	SPDLOG_INFO("atcab_init_ext: bus {:d} dev 0x{:02X}", unsigned(m_cfg.atcai2c.bus), unsigned(m_cfg.atcai2c.address));

	ATCA_STATUS ret;
	for(int i = 0; i < 3; i++)
	{
		ret = atcab_init_ext(&m_dev, &m_cfg);
		calib_exit(m_dev);

		if(ret == ATCA_SUCCESS)
		{
			break;
		}
		SPDLOG_ERROR("error in atcab_init_ext: {:d}", ret);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("error in atcab_init_ext: {:d}", ret);
		return false;
	}

	{
		uint8_t test = 0xFF;
		if( ! selftest(&test) )
		{
			return false;
		}
		if(test != 0x00)
		{
			return false;
		}

		SPDLOG_DEBUG("Self test ok");
	}

	if(m_rng.accepts_input())
	{
		std::array<uint8_t, 32> random;
		if( ! read_rand(&random) )
		{
			return false;
		}

		m_rng.add_entropy(random.data(), random.size());

		SPDLOG_DEBUG("Added input to RNG");
	}
	else
	{
		SPDLOG_WARN("RNG does not accept input");
	}

	if( ! read_info(&m_rev_cache) )
	{
		return false;
	}
	SPDLOG_DEBUG("Revision: {:s}", 
		fmt::format("{:02X}", fmt::join(m_rev_cache, ""))
	);

	if( ! read_serial_number(&m_sn_cache) )
	{
		return false;
	}
	SPDLOG_DEBUG("Serial number: {:s}", 
		fmt::format("{:02X}", fmt::join(m_sn_cache, ""))
	);
	
	if( ! read_config_zone(&m_config_cache) )
	{
		return false;
	}

	{
		std::string str;
		for(size_t i = 0; i < 16; i++)
		{
			str += fmt::format("SlotConfig {:d} : {:02X}{:02X}\n",
					i,
					unsigned(m_config_cache[20+i*2+1]),
					unsigned(m_config_cache[20+i*2])
				);
		}
		SPDLOG_DEBUG("{:s}", str);

		str.clear();
		for(size_t i = 0; i < 16; i++)
		{
			str += fmt::format("KeyConfig {:d} : {:02X}{:02X}\n",
				i,
				unsigned(m_config_cache[96+i*2+1]),
				unsigned(m_config_cache[96+i*2])
			);
		}
		SPDLOG_DEBUG("{:s}", str);
	}

	if( ! read_otp_zone(0, &m_otp0_cache) )
	{
		return false;
	}

	if( ! read_otp_zone(1, &m_otp1_cache) )
	{
		return false;
	}

	return true;
}

bool ATECC608_iface::close()
{
	if( ! m_dev )
	{
		return true;
	}

	bool fret = true;

	ATCA_STATUS ret = atcab_release_ext(&m_dev);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::close error: {:d}", ret);
		fret = false;
	}

	m_dev = nullptr;

	return fret;
}

bool ATECC608_iface::read_pubkey(const uint16_t key_id, std::array<uint8_t, 64>* const out_pubkey)
{
	if( ! out_pubkey )
	{
		return false;
	}

	ATCA_STATUS ret = calib_get_pubkey(m_dev, key_id, out_pubkey->data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::read_pubkey calib_get_pubkey({:d}) error: {:d}", key_id, ret);
		return false;
	}

	return true;
}

bool ATECC608_iface::read_pubkey(const uint16_t key_id, std::shared_ptr<Botan::ECDSA_PublicKey>* const out_pubkey)
{
	if( ! out_pubkey )
	{
		return false;
	}

	bool fret = true;

	std::array<uint8_t, 64> pub_key;
	if( ! read_pubkey(key_id, &pub_key) )
	{
		fret = false;
	}

	if(fret)
	{
		Botan::EC_Group secp256r1_group("secp256r1");
		*out_pubkey = std::make_shared<Botan::ECDSA_PublicKey>(secp256r1_group, secp256r1_group.point(Botan::BigInt(pub_key.data(), 32), Botan::BigInt(pub_key.data()+32, 32)));
	}

	return fret;
}

bool ATECC608_iface::read_rand(std::array<uint8_t, 32>* const out_random)
{
	if( ! out_random )
	{
		return false;
	}

	ATCA_STATUS ret = atcab_random_ext(m_dev, out_random->data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::read_rand atcab_random_ext: error {:d}", ret);
		return false;
	}

	return true;
}

bool ATECC608_iface::read_rand_feed_sw_rng()
{
	if( ! m_rng.accepts_input() )
	{
		return false;
	}

	std::array<uint8_t, 32> random;
	if( ! read_rand(&random) )
	{
		return false;
	}

	m_rng.add_entropy(random.data(), random.size());

	return true;
}

bool ATECC608_iface::read_counter(uint16_t counter_id, uint32_t* out_val)
{
	if( ! out_val )
	{
		return false;
	}

	ATCA_STATUS ret = calib_counter_read(m_dev, counter_id, out_val);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::read_counter calib_counter_read: error {:d}", ret);
		return false;
	}

	return true;
}
bool ATECC608_iface::increment_counter(uint16_t counter_id, uint32_t* out_val)
{
	if( ! out_val )
	{
		return false;
	}

	ATCA_STATUS ret = calib_counter_increment(m_dev, counter_id, out_val);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::increment_counter calib_counter_increment: error {:d}", ret);
		return false;
	}
	return true;
}

bool ATECC608_iface::read_info(std::array<uint8_t, 4>* const out_revision)
{
	if( ! out_revision )
	{
		return false;
	}

	ATCA_STATUS ret = calib_info(m_dev, out_revision->data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::read_info calib_info: error {:d}", ret);
		return false;
	}

	return true;
}

bool ATECC608_iface::read_serial_number(std::array<uint8_t, 9>* const out_sn)
{
	if( ! out_sn )
	{
		return false;
	}

	ATCA_STATUS ret = calib_read_serial_number(m_dev, out_sn->data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::read_serial_number calib_read_serial_number: error {:d}", ret);
		return false;
	}

	return true;
}

bool ATECC608_iface::read_config_zone(std::array<uint8_t, 128>* const out_config)
{
	if( ! out_config )
	{
		return false;
	}

	ATCA_STATUS ret = calib_read_config_zone(m_dev, out_config->data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::read_config_zone calib_read_config_zone: error {:d}", ret);
		return false;
	}

	return true;
}

bool ATECC608_iface::read_otp_zone(const uint8_t block, std::array<uint8_t, 32>* const out_otp)
{
	if( ! out_otp )
	{
		return false;
	}

	ATCA_STATUS ret = calib_read_zone(m_dev, ATCA_ZONE_OTP, 0, block, 0, out_otp->data(), out_otp->size());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::read_config_zone calib_read_config_zone: error {:d}", ret);
		return false;
	}
	return true;
}

bool ATECC608_iface::write_updateextra(const uint8_t mode, const uint16_t new_value)
{
	ATCA_STATUS ret = calib_updateextra(m_dev, mode, new_value);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::write_updateextra calib_updateextra: error {:d}", ret);
		return false;
	}
	return true;
}

bool ATECC608_iface::wake()
{
	ATCA_STATUS ret = calib_wakeup(m_dev);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::wake calib_exit: error {:d}", ret);
		return false;
	}

	return true;
}

bool ATECC608_iface::idle()
{
	ATCA_STATUS ret = calib_idle(m_dev);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::idle calib_idle: error {:d}", ret);
		return false;
	}

	return true;
}

bool ATECC608_iface::sleep()
{
	ATCA_STATUS ret = calib_sleep(m_dev);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::sleep calib_sleep: error {:d}", ret);
		return false;
	}

	return true;
}

bool ATECC608_iface::selftest(uint8_t* const out_test)
{
	if( ! out_test )
	{
		return false;
	}

	ATCA_STATUS ret = calib_selftest(m_dev, SELFTEST_MODE_ALL, 0, out_test);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::selftest calib_selftest: error {:d}", ret);
		return false;
	}

	return true;
}
bool ATECC608_iface::sign_raw(const std::array<uint8_t, 32>& msg, const uint16_t key_id, std::array<uint8_t, 64>* const out_sig)
{
	if( ! out_sig )
	{
		return false;
	}

	ATCA_STATUS ret = calib_sign(m_dev, key_id, msg.data(), out_sig->data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::sign calib_sign: error {:d}", ret);
		return false;
	}

	return true;
}

bool ATECC608_iface::verify_raw(const std::array<uint8_t, 32>& msg, const std::array<uint8_t, 64>& sig, const Botan::Public_Key& pubkey)
{
	Botan::PK_Verifier verifier(pubkey, "Raw(SHA-256)",  Botan::Signature_Format::IEEE_1363);
	verifier.update(msg.data(), msg.size());
	return verifier.check_signature(sig.data(), sig.size());
}

bool ATECC608_iface::sign_with_nonce(const std::array<uint8_t, 32>& msg, const uint16_t key_id, Signature_with_nonce* const out_sig)
{
	if( ! out_sig )
	{
		return false;
	}

	out_sig->key_id = key_id;
	out_sig->msg = msg;

	m_rng.randomize_with_ts_input(out_sig->hostnonce.data(), out_sig->hostnonce.size());

	ATCA_STATUS ret = calib_nonce_rand(m_dev, out_sig->hostnonce.data(), out_sig->devnonce.data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("error in calib_nonce_rand(): {:d}", ret);
		return false;
	}

	ret = calib_gendig(m_dev, GENDIG_ZONE_SHARED_NONCE, 0, msg.data(), msg.size());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("error in calib_genkey_base(): {:d}", ret);
		return false;
	}

	ret = calib_sign_internal(m_dev, key_id, false, true, out_sig->sig.data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("error in calib_sign_base({:d}): {:d}", key_id, ret);
		return false;
	}

	return true;
}

bool ATECC608_iface::verify_with_nonce(const std::array<uint8_t, 9>& sn, const std::array<uint8_t, 128>& config, const Signature_with_nonce& sig_data, const Botan::Public_Key& signing_pubkey)
{
	atca_temp_key_t tempkey;
	memset(&tempkey, 0, sizeof(tempkey));

	atca_nonce_in_out_t host_devnonce;
	memset(&host_devnonce, 0, sizeof(host_devnonce));
	host_devnonce.mode     = NONCE_MODE_SEED_UPDATE;
	host_devnonce.zero     = NONCE_ZERO_CALC_RANDOM;
	host_devnonce.num_in   = sig_data.hostnonce.data();
	host_devnonce.rand_out = sig_data.devnonce.data();
	host_devnonce.temp_key = &tempkey;

	atca_gen_dig_in_out_t gendig;
	memset(&gendig, 0, sizeof(gendig));
	gendig.zone         = GENDIG_ZONE_SHARED_NONCE;
	gendig.key_id       = 0;
	gendig.slot_conf    = 0;
	gendig.key_conf     = 0;
	gendig.slot_locked  = 0;
	gendig.counter      = 0;
	gendig.is_key_nomac = 0;
	gendig.sn           = sn.data();
	gendig.stored_value = nullptr;
	gendig.other_data   = sig_data.msg.data();
	gendig.temp_key     = &tempkey;

	std::array<uint8_t, 55> message;
	std::array<uint8_t, 32> digest;

	atca_sign_internal_in_out_t sig_val_data;
	memset(&sig_val_data, 0, sizeof(sig_val_data));
	sig_val_data.mode              = SIGN_MODE_INTERNAL | SIGN_MODE_INCLUDE_SN;
	sig_val_data.key_id            = sig_data.key_id;
	// sig_val_data.slot_config    // fill by atcah_config_to_sign_internal
	// sig_val_data.key_config     // fill by atcah_config_to_sign_internal
	sig_val_data.use_flag          = 0;
	sig_val_data.update_count      = 0;
	// sig_val_data.is_slot_locked // fill by atcah_config_to_sign_internal
	sig_val_data.for_invalidate    = 0;
	sig_val_data.sn                = sn.data();
	sig_val_data.temp_key          = &tempkey;
	sig_val_data.message           = message.data();
	sig_val_data.verify_other_data = nullptr;
	sig_val_data.digest            = digest.data();

	ATCA_STATUS ret = atcah_nonce(&host_devnonce);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("atcah_nonce: {:d}", ret);
		return false;
	}

	ret = atcah_gen_dig(&gendig);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("atcah_gen_key_msg: {:d}", ret);
		return false;
	}

	ret = atcah_config_to_sign_internal(ATECC608, &sig_val_data, config.data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("atcah_config_to_sign_internal: {:d}", ret);
		return false;
	}

	ret = atcah_sign_internal_msg(ATECC608, &sig_val_data);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("atcah_sign_internal_msg: {:d}", ret);
		return false;
	}

	return verify_raw(digest, sig_data.sig, signing_pubkey);
}

bool ATECC608_iface::sign_ext_with_nonce(const std::span<uint8_t>& msg, const uint16_t key_id, Signature_with_nonce* const out_sig)
{
	if( ! out_sig )
	{
		return false;
	}

	if(key_id > 15)
	{
		return false;
	}

	out_sig->key_id = key_id;

	m_rng.randomize_with_ts_input(out_sig->hostnonce.data(), out_sig->hostnonce.size());
	
	if( ! read_rand(&out_sig->devnonce) )
	{
		return false;
	}

	{
		std::unique_ptr<Botan::HashFunction> hash = Botan::HashFunction::create_or_throw("SHA-256");

		if(out_sig->msg.size() != hash->output_length())
		{
			return false;
		}

		hash->update(out_sig->devnonce.data(), out_sig->devnonce.size());
		hash->update(out_sig->hostnonce.data(), out_sig->hostnonce.size());
		hash->update(m_sn_cache.data(), m_sn_cache.size());
		hash->update_le(static_cast<uint16_t>(key_id));
		hash->update_le(get_cached_slotConfig(key_id));
		hash->update_le(get_cached_keyConfig(key_id));
		
		if(get_cached_slotLocked() & (1U << key_id))
		{
			hash->update(0x01);
		}
		else
		{
			hash->update(0x00);
		}

		hash->update_le(static_cast<uint64_t>(msg.size()));
		hash->update(msg.data(), msg.size());

		hash->final(out_sig->msg.data());
	}

	return sign_raw(out_sig->msg, key_id, &out_sig->sig);
}

bool ATECC608_iface::verify_ext_with_nonce(const std::span<uint8_t>& msg, const std::array<uint8_t, 9>& sn, const std::array<uint8_t, 128>& config, const Signature_with_nonce& sig_data, const Botan::Public_Key& signing_pubkey)
{
	if(sig_data.key_id > 15)
	{
		return false;
	}

	// recalculate signing hash from msg
	std::array<uint8_t, 32> temp_msg;
	{
		std::unique_ptr<Botan::HashFunction> hash = Botan::HashFunction::create_or_throw("SHA-256");

		if(temp_msg.size() != hash->output_length())
		{
			SPDLOG_ERROR("Hash mismatch");
			return false;
		}

		hash->update(sig_data.devnonce.data(), sig_data.devnonce.size());
		hash->update(sig_data.hostnonce.data(), sig_data.hostnonce.size());
		hash->update(sn.data(), sn.size());
		hash->update_le(static_cast<uint16_t>(sig_data.key_id));
		hash->update_le(get_slotConfig(config, sig_data.key_id));
		hash->update_le(get_keyConfig(config, sig_data.key_id));
		
		if(get_slotLocked(config) & (1U << sig_data.key_id))
		{
			hash->update(0x01);
		}
		else
		{
			hash->update(0x00);
		}

		hash->update_le(static_cast<uint64_t>(msg.size()));
		hash->update(msg.data(), msg.size());

		hash->final(temp_msg.data());
	}

	// check if hash matches msg
	if(temp_msg.size() != sig_data.msg.size())
	{
		SPDLOG_ERROR("Hash mismatch");
		return false;
	}

	if( ! std::equal(temp_msg.begin(), temp_msg.end(), sig_data.msg.begin()) )
	{
		SPDLOG_ERROR("Hash mismatch");
		return false;
	}

	return verify_raw(temp_msg, sig_data.sig, signing_pubkey);
}

bool ATECC608_iface::ecdh(const uint16_t key_id, std::shared_ptr<const Botan::EC_PublicKey> ext_pubkey, std::array<uint8_t, 32>* const pms)
{
	// https://botan.randombit.net/handbook/api_ref/pubkey.html#code-example-ecdh-key-agreement

	auto ext_pubkey_point = ext_pubkey->public_point();
	
	std::array<uint8_t, 64> ext_pubkey_buf;
	ext_pubkey_point.get_x().binary_encode(ext_pubkey_buf.data(), 32);
	ext_pubkey_point.get_y().binary_encode(ext_pubkey_buf.data()+32, 32);

	// do ECDH on ext device using privkey at key_id
	ATCA_STATUS ret = calib_ecdh(m_dev, key_id, ext_pubkey_buf.data(), pms->data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::sign calib_sign: error {:d}", ret);
		return false;
	}

	return true;
}

// bool ecdh_kdf(std::shared_ptr<const Botan::ECDH_PrivateKey> host_privkey, std::shared_ptr<const Botan::EC_PublicKey> ext_pubkey, const std::array<uint8_t, 32>& pms)
// {
// 	Botan::PK_Key_Agreement ka(host_privkey, m_rng, "KDF2(SHA-256)");
// 
// 	Botan::SymmetricKey key = ka.derive_key(32, ext_pubkey);
// }

bool ATECC608_iface::hmac_sha256(const uint16_t key_id, const std::vector<uint8_t>& msg, std::array<uint8_t, 32>* const out_digest)
{
	if( ! out_digest )
	{
		return false;
	}

	ATCA_STATUS ret = calib_sha_hmac(m_dev, msg.data(), msg.size(), key_id, out_digest->data(), SHA_MODE_TARGET_OUT_ONLY);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("ATECC608_iface::hmac_sha256 calib_sha_hmac: error {:d}", ret);
		return false;
	}

	return true;
}

bool ATECC608_iface::verify_hmac_sha256(const Botan::secure_vector<uint8_t>& key, const std::vector<uint8_t>& msg, const std::array<uint8_t, 32>& digest)
{
	auto hmac = Botan::MessageAuthenticationCode::create("HMAC(SHA-256)");
	if( ! hmac )
	{
		return false;
	}

	hmac->set_key(key);
	hmac->update(msg);
	auto host_mac = hmac->final();

	if(host_mac.size() != digest.size())
	{
		return false;
	}

	return std::equal(host_mac.begin(), host_mac.end(), digest.begin());
}


bool ATECC608_iface::sign_key_fingerprint(const uint16_t keyid_to_sign, const uint16_t signing_keyid, KeyFingerprintSig* const out_data)
{
	if( ! out_data )
	{
		return false;
	}

	out_data->keyid_to_sign = keyid_to_sign;
	out_data->signing_keyid = signing_keyid;

	m_rng.randomize_with_ts_input(out_data->hostnonce.data(), out_data->hostnonce.size());

	ATCA_STATUS ret = calib_nonce_rand(m_dev, out_data->hostnonce.data(), out_data->devnonce.data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("error in calib_nonce_rand(): {:d}", ret);
		return false;
	}

	ret = calib_genkey_base(m_dev, GENKEY_MODE_DIGEST, keyid_to_sign, nullptr, out_data->pubkey.data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("error in calib_genkey_base(): {:d}", ret);
		return false;
	}

	ret = calib_sign_internal(m_dev, signing_keyid, false, true, out_data->sig.data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("error in calib_sign_internal({:d}): {:d}", signing_keyid, ret);
		return false;
	}

	return true;
}

bool ATECC608_iface::verify_key_fingerprint_signature(const std::array<uint8_t, 9>& sn, const std::array<uint8_t, 128>& config, const KeyFingerprintSig& sig_data, const Botan::EC_PublicKey& signing_pubkey)
{
	SPDLOG_TRACE("Verifying signature {:s} for pubkey {:d}: {:s} / {:s}", 
			Botan::base64_encode(sig_data.sig.data(), sig_data.sig.size()),
			sig_data.keyid_to_sign,
			Botan::base64_encode(sig_data.pubkey.data(),    32),
			Botan::base64_encode(sig_data.pubkey.data()+32, 32)
		);

	atca_temp_key_t tempkey;
	memset(&tempkey, 0, sizeof(tempkey));

	atca_nonce_in_out_t host_devnonce;
	memset(&host_devnonce, 0, sizeof(host_devnonce));
	host_devnonce.mode     = NONCE_MODE_SEED_UPDATE;
	host_devnonce.zero     = NONCE_ZERO_CALC_RANDOM;
	host_devnonce.num_in   = sig_data.hostnonce.data();
	host_devnonce.rand_out = sig_data.devnonce.data();
	host_devnonce.temp_key = &tempkey;

	atca_gen_key_in_out_t genkey;
	memset(&genkey, 0, sizeof(genkey));
	genkey.mode              = GENKEY_MODE_DIGEST;
	genkey.key_id            = sig_data.keyid_to_sign;
	genkey.public_key        = sig_data.pubkey.data();
	genkey.public_key_size   = sig_data.pubkey.size();
	genkey.other_data        = nullptr;
	genkey.sn                = sn.data();
	genkey.temp_key          = &tempkey;

	std::array<uint8_t, 55> message;
	std::array<uint8_t, 32> digest;

	atca_sign_internal_in_out_t sig_val_data;
	memset(&sig_val_data, 0, sizeof(sig_val_data));
	sig_val_data.mode              = SIGN_MODE_INTERNAL | SIGN_MODE_INCLUDE_SN;
	sig_val_data.key_id            = sig_data.signing_keyid;
	// sig_val_data.slot_config    // fill by atcah_config_to_sign_internal
	// sig_val_data.key_config     // fill by atcah_config_to_sign_internal
	sig_val_data.use_flag          = 0;
	sig_val_data.update_count      = 0;
	// sig_val_data.is_slot_locked // fill by atcah_config_to_sign_internal
	sig_val_data.for_invalidate    = 0;
	sig_val_data.sn                = sn.data();
	sig_val_data.temp_key          = &tempkey;
	sig_val_data.message           = message.data();
	sig_val_data.verify_other_data = nullptr;
	sig_val_data.digest            = digest.data();

	ATCA_STATUS ret = atcah_nonce(&host_devnonce);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("atcah_nonce: {:d}", ret);
		return false;
	}

	ret = atcah_gen_key_msg(&genkey);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("atcah_gen_key_msg: {:d}", ret);
		return false;
	}

	ret = atcah_config_to_sign_internal(ATECC608, &sig_val_data, config.data());
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("atcah_config_to_sign_internal: {:d}", ret);
		return false;
	}

	ret = atcah_sign_internal_msg(ATECC608, &sig_val_data);
	if(ret != ATCA_SUCCESS)
	{
		SPDLOG_ERROR("atcah_sign_internal_msg: {:d}", ret);
		return false;
	}

	return verify_raw(digest, sig_data.sig, signing_pubkey);
}