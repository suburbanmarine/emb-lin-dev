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

#include <emb-lin-dev/crypto/ATECC608_botan.hpp>

#include <botan/der_enc.h>
#include <botan/base64.h>
#include <botan/x509_key.h>

Botan::secure_vector<uint8_t> ATECC608_ECDSA_SHA256_Signer::sign(Botan::RandomNumberGenerator& rng)
{
	std::array<uint8_t, 32> msg;

	{
		Botan::secure_vector<uint8_t> temp = m_hash_func->final();
		if(temp.size() != msg.size())
		{
			throw Botan::Invalid_Argument("Length of hash output is incorrect");
		}

		std::copy_n(temp.begin(), msg.size(), msg.begin());
	}

	std::array<uint8_t, 64> sig;
	if( ! m_atecc.sign_raw(msg, m_priv_key_id, &sig) )
	{
		throw Botan::System_Error("m_atecc.sign_raw failed");
	}

	return der_encode_signature(sig);
}

Botan::secure_vector<uint8_t> ATECC608_ECDSA_SHA256_Signer::der_encode_signature(const std::array<uint8_t, 64>& sig)
{
	// convert raw to DerSequence
	
	std::vector<Botan::BigInt> sig_chunk;
	sig_chunk.reserve(2);
	sig_chunk.emplace_back(Botan::BigInt(sig.data() +  0, 32));
	sig_chunk.emplace_back(Botan::BigInt(sig.data() + 32, 32));

	Botan::secure_vector<uint8_t> der_sig;
	der_sig.reserve(sig.size() + 8 + 4*sig_chunk.size());

	Botan::DER_Encoder der_enc(der_sig);
	der_enc.start_cons(Botan::SEQUENCE);
	der_enc.encode_list(sig_chunk);
	der_enc.end_cons();

	return der_sig;
}

std::string ATECC_Botan_util::x509_to_der_b64(const std::shared_ptr<const Botan::X509_Certificate>& cert)
{
	if( ! cert)
	{
		return std::string();
	}

	return x509_to_der_b64(*cert);
}

std::string ATECC_Botan_util::x509_to_der_b64(const Botan::X509_Certificate& cert)
{
	std::vector<uint8_t> cert_der;
	cert_der.reserve(1024);

	Botan::DER_Encoder der_enc(cert_der);
	cert.encode_into(der_enc);

	return Botan::base64_encode(cert_der);
}

std::string ATECC_Botan_util::pubkey_to_x509ber_b64(const Botan::Public_Key& key)
{
	return Botan::base64_encode(Botan::X509::BER_encode(key));
}
