#pragma once

#include <emb-lin-dev/crypto/ATECC608_iface.hpp>

#include <botan/rng.h>
#include <botan/hash.h>
#include <botan/pk_ops.h>
#include <botan/pk_keys.h>


#include <memory>

#include <spdlog/spdlog.h>

class ATECC608_ECDSA_SHA256_Signer : public Botan::PK_Ops::Signature
{
public:
	ATECC608_ECDSA_SHA256_Signer(ATECC608_iface& atecc, const uint16_t& priv_key_id) : m_atecc(atecc), m_priv_key_id(priv_key_id)
	{
		m_hash_func = Botan::HashFunction::create_or_throw("SHA-256");
	}
	~ATECC608_ECDSA_SHA256_Signer() override
	{

	}

	void update(const uint8_t msg[], size_t msg_len) override
	{
		m_hash_func->update(msg, msg_len);
	}

	Botan::secure_vector<uint8_t> sign(Botan::RandomNumberGenerator& rng) override;

	size_t signature_length() const override
	{
		return 64;
	}

	Botan::secure_vector<uint8_t> der_encode_signature(const std::array<uint8_t, 64>& sig);

protected:
	ATECC608_iface& m_atecc;
	uint16_t m_priv_key_id;
	std::unique_ptr<Botan::HashFunction> m_hash_func;
};

class ATECC608_ECDSA_PrivateKey : public Botan::Private_Key
{
public:

	ATECC608_ECDSA_PrivateKey(ATECC608_iface& atecc, const uint16_t& priv_key_id, const std::shared_ptr<const Botan::Public_Key>& pub_key) : m_atecc(atecc), m_priv_key_id(priv_key_id)
	{
		m_pub_key = pub_key;
	}
	~ATECC608_ECDSA_PrivateKey() override
	{

	}

	std::unique_ptr<Botan::PK_Ops::Signature> create_signature_op(Botan::RandomNumberGenerator& rng, const std::string& params, const std::string& provider) const override
	{
		return std::make_unique<ATECC608_ECDSA_SHA256_Signer>(m_atecc, m_priv_key_id);
	}

	std::string algo_name() const override
	{
		return m_pub_key->algo_name();
	}
	size_t estimated_strength() const override
	{
		return m_pub_key->estimated_strength();
	}

	size_t key_length() const override
	{
		return m_pub_key->key_length();
	}
	bool check_key(Botan::RandomNumberGenerator& rng, bool strong) const override
	{
		throw Botan::Invalid_State("Can not check private key");
	}

	Botan::AlgorithmIdentifier algorithm_identifier() const override
	{
		return m_pub_key->algorithm_identifier();
	}

	std::vector<uint8_t> public_key_bits() const override
	{
		return m_pub_key->public_key_bits();
	}

	Botan::secure_vector<uint8_t> private_key_bits() const override
	{
		throw Botan::Invalid_State("Can not read private key from module");
	}

protected:
	ATECC608_iface& m_atecc;
	uint16_t m_priv_key_id;
	std::shared_ptr<const Botan::Public_Key> m_pub_key;
};
