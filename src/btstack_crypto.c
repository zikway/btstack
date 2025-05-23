/*
 * Copyright (C) 2017 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#define BTSTACK_FILE__ "btstack_crypto.c"

/*
 * btstack_crypto.h
 *
 * Central place for all crypto-related functions with completion callbacks to allow
 * using of MCU crypto peripherals or the Bluetooth controller
 */

#include "btstack_crypto.h"

#include "btstack_debug.h"
#include "btstack_event.h"
#include "btstack_linked_list.h"
#include "btstack_util.h"
#include "btstack_bool.h"
#include "hci.h"

//
// AES128 Configuration
//

// By default, AES128 is computed by Bluetooth Controller using HCI Command/Event asynchronously
// as fallback/alternative, a software implementation can be used
// configure ECC implementations
#if defined(HAVE_AES128) && defined(ENABLE_SOFTWARE_AES128)
#error "If you have custom AES128 implementation (HAVE_AES128), please disable software AES128 (ENABLE_SOFTWARE_AES128) in bstack_config.h"
#endif

#ifdef ENABLE_SOFTWARE_AES128
#define HAVE_AES128
#include "rijndael.h"
#endif

#ifdef HAVE_AES128
#define USE_BTSTACK_AES128
#endif

//
// ECC Configuration
// 

// backwards-compatitility ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS -> ENABLE_MICRO_ECC_P256
#if defined(ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS) && !defined(ENABLE_MICRO_ECC_P256)
#define ENABLE_MICRO_ECC_P256
#endif

// configure ECC implementations
#if defined(ENABLE_MICRO_ECC_P256) && defined(HAVE_MBEDTLS_ECC_P256)
#error "If you have mbedTLS (HAVE_MBEDTLS_ECC_P256), please disable uECC (ENABLE_MICRO_ECC_P256) in bstack_config.h"
#endif

// Software ECC-P256 implementation provided by micro-ecc
#ifdef ENABLE_MICRO_ECC_P256
#define ENABLE_ECC_P256
#define USE_MICRO_ECC_P256
#define USE_SOFTWARE_ECC_P256_IMPLEMENTATION
#include "uECC.h"
#endif

// Software ECC-P256 implementation provided by mbedTLS, allow config via MBEDTLS_CONFIG_FILE
#ifdef HAVE_MBEDTLS_ECC_P256
#define ENABLE_ECC_P256
#define USE_MBEDTLS_ECC_P256
#define USE_SOFTWARE_ECC_P256_IMPLEMENTATION
#ifdef MBEDTLS_CONFIG_FILE
// cppcheck-suppress preprocessorErrorDirective
#include MBEDTLS_CONFIG_FILE
#else
#include "mbedtls/mbedtls_config.h"
#endif
#include "mbedtls/platform.h"
#include "mbedtls/ecp.h"
#endif

#if defined(ENABLE_LE_SECURE_CONNECTIONS) && !defined(ENABLE_ECC_P256)
#define ENABLE_ECC_P256
#endif

// debugging
// #define DEBUG_CCM

typedef enum {
    CMAC_IDLE,
    CMAC_CALC_SUBKEYS,
    CMAC_W4_SUBKEYS,
    CMAC_CALC_MI,
    CMAC_W4_MI,
    CMAC_CALC_MLAST,
    CMAC_W4_MLAST
} btstack_crypto_cmac_state_t;

typedef enum {
    ECC_P256_KEY_GENERATION_IDLE,
    ECC_P256_KEY_GENERATION_GENERATING_RANDOM,
    ECC_P256_KEY_GENERATION_ACTIVE,
    ECC_P256_KEY_GENERATION_W4_KEY,
    ECC_P256_KEY_GENERATION_DONE,
} btstack_crypto_ecc_p256_key_generation_state_t;

static void btstack_crypto_run(void);
static void btstack_crypto_state_reset(void);

static const uint8_t zero[16] = { 0 };

static bool btstack_crypto_initialized;
static bool btstack_crypto_wait_for_hci_result;
static btstack_linked_list_t btstack_crypto_operations;
static btstack_packet_callback_registration_t hci_event_callback_registration;

// state for AES-CMAC
#ifndef USE_BTSTACK_AES128
static btstack_crypto_cmac_state_t btstack_crypto_cmac_state;
static sm_key_t btstack_crypto_cmac_k;
static sm_key_t btstack_crypto_cmac_x;
static sm_key_t btstack_crypto_cmac_subkey;
static uint8_t  btstack_crypto_cmac_block_current;
static uint8_t  btstack_crypto_cmac_block_count;
#endif

// state for AES-CCM
static uint8_t btstack_crypto_ccm_s[16];

#ifdef ENABLE_ECC_P256

static uint8_t  btstack_crypto_ecc_p256_public_key[64];
static uint8_t  btstack_crypto_ecc_p256_random[64];
static uint8_t  btstack_crypto_ecc_p256_random_len;
static uint8_t  btstack_crypto_ecc_p256_random_offset;
static btstack_crypto_ecc_p256_key_generation_state_t btstack_crypto_ecc_p256_key_generation_state;

#ifdef USE_SOFTWARE_ECC_P256_IMPLEMENTATION
static uint8_t btstack_crypto_ecc_p256_d[32];
#endif

// Software ECDH implementation provided by mbedtls
#ifdef USE_MBEDTLS_ECC_P256
static mbedtls_ecp_group   mbedtls_ec_group;
#endif

#endif /* ENABLE_ECC_P256 */

#ifdef ENABLE_SOFTWARE_AES128
// AES128 using public domain rijndael implementation
void btstack_aes128_calc(const uint8_t * key, const uint8_t * plaintext, uint8_t * ciphertext){
    uint32_t rk[RKLENGTH(KEYBITS)];
    int nrounds = rijndaelSetupEncrypt(rk, &key[0], KEYBITS);
    rijndaelEncrypt(rk, nrounds, plaintext, ciphertext);
}
#endif

static void btstack_crypto_done(btstack_crypto_t * btstack_crypto){
    btstack_linked_list_pop(&btstack_crypto_operations);
    (*btstack_crypto->context_callback.callback)(btstack_crypto->context_callback.context);
}

static void btstack_crypto_cmac_shift_left_by_one_bit_inplace(int len, uint8_t * data){
    int i;
    int carry = 0;
    for (i=len-1; i >= 0 ; i--){
        int new_carry = data[i] >> 7;
        data[i] = (data[i] << 1) | carry;
        carry = new_carry;
    }
}

static uint8_t btstack_crypto_cmac_get_byte(btstack_crypto_aes128_cmac_t * btstack_crypto_cmac, uint16_t pos){
    if (btstack_crypto_cmac->btstack_crypto.operation == BTSTACK_CRYPTO_CMAC_GENERATOR){
        return (*btstack_crypto_cmac->data.get_byte_callback)(pos);
    } else {
        return btstack_crypto_cmac->data.message[pos]; 
    }
}

#ifdef USE_BTSTACK_AES128

static void btstack_crypto_cmac_calc_subkeys(sm_key_t k0, sm_key_t k1, sm_key_t k2){
    memcpy(k1, k0, 16);
    btstack_crypto_cmac_shift_left_by_one_bit_inplace(16, k1);
    if (k0[0] & 0x80){
        k1[15] ^= 0x87;
    }
    memcpy(k2, k1, 16);
    btstack_crypto_cmac_shift_left_by_one_bit_inplace(16, k2);
    if (k1[0] & 0x80){
        k2[15] ^= 0x87;
    } 
}

static void btstack_crypto_cmac_calc(btstack_crypto_aes128_cmac_t * btstack_crypto_cmac) {
    sm_key_t k0, k1, k2;
    uint16_t i;

    btstack_aes128_calc(btstack_crypto_cmac->key, zero, k0);
    btstack_crypto_cmac_calc_subkeys(k0, k1, k2);

    uint16_t cmac_block_count = (btstack_crypto_cmac->size + 15) / 16;

    // step 3: ..
    if (cmac_block_count==0){
        cmac_block_count = 1;
    }

    // Step 5
    sm_key_t cmac_x;
    memset(cmac_x, 0, 16);

    // Step 6
    sm_key_t cmac_y;
    int block;
    for (block = 0 ; block < cmac_block_count-1 ; block++){
        for (i=0;i<16;i++){
            cmac_y[i] = cmac_x[i] ^ btstack_crypto_cmac_get_byte(btstack_crypto_cmac, (block*16) + i);
        }
        btstack_aes128_calc(btstack_crypto_cmac->key, cmac_y, cmac_x);
    }

    // step 4: set m_last
    sm_key_t cmac_m_last;
    bool last_block_complete = btstack_crypto_cmac->size != 0 && (btstack_crypto_cmac->size & 0x0f) == 0;
    if (last_block_complete){
        for (i=0;i<16;i++){
            cmac_m_last[i] = btstack_crypto_cmac_get_byte(btstack_crypto_cmac, btstack_crypto_cmac->size - 16 + i) ^ k1[i];
        }
    } else {
        uint16_t valid_octets_in_last_block = btstack_crypto_cmac->size & 0x0f;
        for (i=0;i<16;i++){
            if (i < valid_octets_in_last_block){
                cmac_m_last[i] = btstack_crypto_cmac_get_byte(btstack_crypto_cmac, (btstack_crypto_cmac->size & 0xfff0) + i) ^ k2[i];
                continue;
            }
            if (i == valid_octets_in_last_block){
                cmac_m_last[i] = 0x80 ^ k2[i];
                continue;
            }
            cmac_m_last[i] = k2[i];
        }
    }

    for (i=0;i<16;i++){
        cmac_y[i] = cmac_x[i] ^ cmac_m_last[i];
    }

    // Step 7
    btstack_aes128_calc(btstack_crypto_cmac->key, cmac_y, btstack_crypto_cmac->hash);
}
#else

static void btstack_crypto_aes128_start(const sm_key_t key, const sm_key_t plaintext){
    uint8_t key_flipped[16];
    uint8_t plaintext_flipped[16];
    reverse_128(key, key_flipped);
    reverse_128(plaintext, plaintext_flipped);
    btstack_crypto_wait_for_hci_result = 1;
    hci_cmd_send(&hci_le_encrypt, key_flipped, plaintext_flipped);
}

static inline void btstack_crypto_cmac_next_state(void){
    btstack_crypto_cmac_state = (btstack_crypto_cmac_state_t) (((int)btstack_crypto_cmac_state) + 1);
}

static int btstack_crypto_cmac_last_block_complete(btstack_crypto_aes128_cmac_t * btstack_crypto_cmac){
	uint16_t len = btstack_crypto_cmac->size;
    if (len == 0u) return 0u;
    return (len & 0x0fu) == 0u;
}

static void btstack_crypto_cmac_handle_aes_engine_ready(btstack_crypto_aes128_cmac_t * btstack_crypto_cmac){
    switch (btstack_crypto_cmac_state){
        case CMAC_CALC_SUBKEYS: {
            btstack_crypto_cmac_next_state();
            btstack_crypto_aes128_start(btstack_crypto_cmac_k, zero);
            break;
        }
        case CMAC_CALC_MI: {
            int j;
            sm_key_t y;
            for (j=0;j<16;j++){
                y[j] = btstack_crypto_cmac_x[j] ^ btstack_crypto_cmac_get_byte(btstack_crypto_cmac, (btstack_crypto_cmac_block_current*16u) + j);
            }
            btstack_crypto_cmac_block_current++;
            btstack_crypto_cmac_next_state();
            btstack_crypto_aes128_start(btstack_crypto_cmac_k, y);
            break;
        }
        case CMAC_CALC_MLAST: {
            sm_key_t k1;
            (void)memcpy(k1, btstack_crypto_cmac_subkey, 16);
            btstack_crypto_cmac_shift_left_by_one_bit_inplace(16, k1);
            if (btstack_crypto_cmac_subkey[0u] & 0x80u){
                k1[15u] ^= 0x87u;
            }
            sm_key_t k2;
            (void)memcpy(k2, k1, 16);
            btstack_crypto_cmac_shift_left_by_one_bit_inplace(16, k2);
            if (k1[0u] & 0x80u){
                k2[15u] ^= 0x87u;
            }

            log_info_key("k", btstack_crypto_cmac_k);
            log_info_key("k1", k1);
            log_info_key("k2", k2);

            // step 4: set m_last
            int i;
            sm_key_t btstack_crypto_cmac_m_last;
            if (btstack_crypto_cmac_last_block_complete(btstack_crypto_cmac)){
                for (i=0;i<16;i++){
                    btstack_crypto_cmac_m_last[i] = btstack_crypto_cmac_get_byte(btstack_crypto_cmac, btstack_crypto_cmac->size - 16u + i) ^ k1[i];
                }
            } else {
                int valid_octets_in_last_block = btstack_crypto_cmac->size & 0x0fu;
                for (i=0;i<16;i++){
                    if (i < valid_octets_in_last_block){
                        btstack_crypto_cmac_m_last[i] = btstack_crypto_cmac_get_byte(btstack_crypto_cmac, (btstack_crypto_cmac->size & 0xfff0u) + i) ^ k2[i];
                        continue;
                    }
                    if (i == valid_octets_in_last_block){
                        btstack_crypto_cmac_m_last[i] = 0x80u ^ k2[i];
                        continue;
                    }
                    btstack_crypto_cmac_m_last[i] = k2[i];
                }
            }
            sm_key_t y;
            for (i=0;i<16;i++){
                y[i] = btstack_crypto_cmac_x[i] ^ btstack_crypto_cmac_m_last[i];
            }
            btstack_crypto_cmac_block_current++;
            btstack_crypto_cmac_next_state();
            btstack_crypto_aes128_start(btstack_crypto_cmac_k, y);
            break;
        }
        default:
            log_info("btstack_crypto_cmac_handle_aes_engine_ready called in state %u", btstack_crypto_cmac_state);
            break;
    }
}

static void btstack_crypto_cmac_handle_encryption_result(btstack_crypto_aes128_cmac_t * btstack_crypto_cmac, sm_key_t data){
    switch (btstack_crypto_cmac_state){
        case CMAC_W4_SUBKEYS:
            (void)memcpy(btstack_crypto_cmac_subkey, data, 16);
            // next
            btstack_crypto_cmac_state = (btstack_crypto_cmac_block_current < (btstack_crypto_cmac_block_count - 1u)) ? CMAC_CALC_MI : CMAC_CALC_MLAST;
            break;
        case CMAC_W4_MI:
            (void)memcpy(btstack_crypto_cmac_x, data, 16);
            btstack_crypto_cmac_state = (btstack_crypto_cmac_block_current < (btstack_crypto_cmac_block_count - 1u)) ? CMAC_CALC_MI : CMAC_CALC_MLAST;
            break;
        case CMAC_W4_MLAST:
            // done
            log_info("Setting CMAC Engine to IDLE");
            btstack_crypto_cmac_state = CMAC_IDLE;
            log_info_key("CMAC", data);
            (void)memcpy(btstack_crypto_cmac->hash, data, 16);
			btstack_linked_list_pop(&btstack_crypto_operations);
			(*btstack_crypto_cmac->btstack_crypto.context_callback.callback)(btstack_crypto_cmac->btstack_crypto.context_callback.context);
            break;
        default:
            log_info("btstack_crypto_cmac_handle_encryption_result called in state %u", btstack_crypto_cmac_state);
            break;
    }
}

static void btstack_crypto_cmac_start(btstack_crypto_aes128_cmac_t * btstack_crypto_cmac){

    (void)memcpy(btstack_crypto_cmac_k, btstack_crypto_cmac->key, 16);
    memset(btstack_crypto_cmac_x, 0, 16);
    btstack_crypto_cmac_block_current = 0;

    // step 2: n := ceil(len/const_Bsize);
    btstack_crypto_cmac_block_count = (btstack_crypto_cmac->size + 15u) / 16u;

    // step 3: ..
    if (btstack_crypto_cmac_block_count==0u){
        btstack_crypto_cmac_block_count = 1;
    }
    log_info("btstack_crypto_cmac_start: len %u, block count %u", btstack_crypto_cmac->size, btstack_crypto_cmac_block_count);

    // first, we need to compute l for k1, k2, and m_last
    btstack_crypto_cmac_state = CMAC_CALC_SUBKEYS;

    // let's go
    btstack_crypto_cmac_handle_aes_engine_ready(btstack_crypto_cmac);
}
#endif

/*
  To encrypt the message data we use Counter (CTR) mode.  We first
  define the key stream blocks by:

      S_i := E( K, A_i )   for i=0, 1, 2, ...

  The values A_i are formatted as follows, where the Counter field i is
  encoded in most-significant-byte first order:

  Octet Number   Contents
  ------------   ---------
  0              Flags
  1 ... 15-L     Nonce N
  16-L ... 15    Counter i

  Bit Number   Contents
  ----------   ----------------------
  7            Reserved (always zero)
  6            Reserved (always zero)
  5 ... 3      Zero
  2 ... 0      L'
*/

static void btstack_crypto_ccm_setup_a_i(btstack_crypto_ccm_t * btstack_crypto_ccm, uint16_t counter){
    btstack_crypto_ccm_s[0] = 1;  // L' = L - 1
    (void)memcpy(&btstack_crypto_ccm_s[1], btstack_crypto_ccm->nonce, 13);
    big_endian_store_16(btstack_crypto_ccm_s, 14, counter);
#ifdef DEBUG_CCM
    printf("btstack_crypto_ccm_setup_a_%u\n", counter);
    printf("%16s: ", "ai");
    printf_hexdump(btstack_crypto_ccm_s, 16);
#endif
}

/*
 The first step is to compute the authentication field T.  This is
   done using CBC-MAC [MAC].  We first define a sequence of blocks B_0,
   B_1, ..., B_n and then apply CBC-MAC to these blocks.

   The first block B_0 is formatted as follows, where l(m) is encoded in
   most-significant-byte first order:

      Octet Number   Contents
      ------------   ---------
      0              Flags
      1 ... 15-L     Nonce N
      16-L ... 15    l(m)

   Within the first block B_0, the Flags field is formatted as follows:

      Bit Number   Contents
      ----------   ----------------------
      7            Reserved (always zero)
      6            Adata
      5 ... 3      M'
      2 ... 0      L'
 */

static void btstack_crypto_ccm_setup_b_0(btstack_crypto_ccm_t * btstack_crypto_ccm, uint8_t * b0){
    uint8_t m_prime = (btstack_crypto_ccm->auth_len - 2u) / 2u;
    uint8_t Adata   = btstack_crypto_ccm->aad_len ? 1 : 0;
    b0[0u] = (Adata << 6u) | (m_prime << 3u) | 1u ;  // Adata, M', L' = L - 1
    (void)memcpy(&b0[1], btstack_crypto_ccm->nonce, 13);
    big_endian_store_16(b0, 14, btstack_crypto_ccm->message_len);
#ifdef DEBUG_CCM
    printf("%16s: ", "B0");
    printf_hexdump(b0, 16);
#endif
}

#ifdef ENABLE_ECC_P256

static void btstack_crypto_log_ec_publickey(const uint8_t * ec_q){
    log_info("Elliptic curve: X");
    log_info_hexdump(&ec_q[0],32);
    log_info("Elliptic curve: Y");
    log_info_hexdump(&ec_q[32],32);
}

#if (defined(USE_MICRO_ECC_P256) && !defined(WICED_VERSION)) || defined(USE_MBEDTLS_ECC_P256)
// @return OK
static int sm_generate_f_rng(unsigned char * buffer, unsigned size){
    if (btstack_crypto_ecc_p256_key_generation_state != ECC_P256_KEY_GENERATION_ACTIVE) return 0;
    log_info("sm_generate_f_rng: size %u - offset %u", (int) size, btstack_crypto_ecc_p256_random_offset);
    btstack_assert((btstack_crypto_ecc_p256_random_offset + size) <= btstack_crypto_ecc_p256_random_len);
    uint16_t remaining_size = size;
    uint8_t * buffer_ptr = buffer;
    while (remaining_size) {
        *buffer_ptr++ = btstack_crypto_ecc_p256_random[btstack_crypto_ecc_p256_random_offset++];
        remaining_size--;
    }
    return 1;
}
#endif
#ifdef USE_MBEDTLS_ECC_P256
// @return error - just wrap sm_generate_f_rng
static int sm_generate_f_rng_mbedtls(void * context, unsigned char * buffer, size_t size){
    UNUSED(context);
    return sm_generate_f_rng(buffer, size) == 0;
}
#endif /* USE_MBEDTLS_ECC_P256 */

static void btstack_crypto_ecc_p256_generate_key_software(void){

    btstack_crypto_ecc_p256_random_offset = 0;
    
    // generate EC key
#ifdef USE_MICRO_ECC_P256

#ifndef WICED_VERSION
    log_info("set uECC RNG for initial key generation with 64 random bytes");
    // micro-ecc from WICED SDK uses its wiced_crypto_get_random by default - no need to set it
    uECC_set_rng(&sm_generate_f_rng);
#endif /* WICED_VERSION */

#if uECC_SUPPORTS_secp256r1
    // standard version
    uECC_make_key(btstack_crypto_ecc_p256_public_key, btstack_crypto_ecc_p256_d, uECC_secp256r1());

    // disable RNG again, as returning no randmon data lets shared key generation fail
    log_info("disable uECC RNG in standard version after key generation");
    uECC_set_rng(NULL);
#else
    // static version
    uECC_make_key(btstack_crypto_ecc_p256_public_key, btstack_crypto_ecc_p256_d);
#endif
#endif /* USE_MICRO_ECC_P256 */

#ifdef USE_MBEDTLS_ECC_P256
    mbedtls_mpi d;
    mbedtls_ecp_point P;
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&P);
    int res = mbedtls_ecp_gen_keypair(&mbedtls_ec_group, &d, &P, &sm_generate_f_rng_mbedtls, NULL);
    log_info("gen keypair %x", res);
    mbedtls_mpi_write_binary(&P.X, &btstack_crypto_ecc_p256_public_key[0],  32);
    mbedtls_mpi_write_binary(&P.Y, &btstack_crypto_ecc_p256_public_key[32], 32);
    mbedtls_mpi_write_binary(&d, btstack_crypto_ecc_p256_d, 32);
    mbedtls_ecp_point_free(&P);
    mbedtls_mpi_free(&d);
#endif  /* USE_MBEDTLS_ECC_P256 */
}

#ifdef USE_SOFTWARE_ECC_P256_IMPLEMENTATION
static void btstack_crypto_ecc_p256_calculate_dhkey_software(btstack_crypto_ecc_p256_t * btstack_crypto_ec_p192){
    memset(btstack_crypto_ec_p192->dhkey, 0, 32);

#ifdef USE_MICRO_ECC_P256
#if uECC_SUPPORTS_secp256r1
    // standard version
    uECC_shared_secret(btstack_crypto_ec_p192->public_key, btstack_crypto_ecc_p256_d, btstack_crypto_ec_p192->dhkey, uECC_secp256r1());
#else
    // static version
    uECC_shared_secret(btstack_crypto_ec_p192->public_key, btstack_crypto_ecc_p256_d, btstack_crypto_ec_p192->dhkey);
#endif
#endif

#ifdef USE_MBEDTLS_ECC_P256
    // da * Pb
    mbedtls_mpi d;
    mbedtls_ecp_point Q;
    mbedtls_ecp_point DH;
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&Q);
    mbedtls_ecp_point_init(&DH);
    mbedtls_mpi_read_binary(&d, btstack_crypto_ecc_p256_d, 32);
    mbedtls_mpi_read_binary(&Q.X, &btstack_crypto_ec_p192->public_key[0] , 32);
    mbedtls_mpi_read_binary(&Q.Y, &btstack_crypto_ec_p192->public_key[32], 32);
    mbedtls_mpi_lset(&Q.Z, 1);
    mbedtls_ecp_mul(&mbedtls_ec_group, &DH, &d, &Q, NULL, NULL);
    mbedtls_mpi_write_binary(&DH.X, btstack_crypto_ec_p192->dhkey, 32);
    mbedtls_ecp_point_free(&DH);
    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&Q);
#endif

    log_info("dhkey");
    log_info_hexdump(btstack_crypto_ec_p192->dhkey, 32);
}
#endif

#endif

static void btstack_crypto_ccm_next_block(btstack_crypto_ccm_t * btstack_crypto_ccm, btstack_crypto_ccm_state_t state_when_done){
    uint16_t bytes_to_process = btstack_min(btstack_crypto_ccm->block_len, 16);
    // next block
    btstack_crypto_ccm->counter++;
    btstack_crypto_ccm->input       += bytes_to_process;
    btstack_crypto_ccm->output      += bytes_to_process;
    btstack_crypto_ccm->block_len   -= bytes_to_process;
    btstack_crypto_ccm->message_len -= bytes_to_process;
#ifdef DEBUG_CCM
    printf("btstack_crypto_ccm_next_block (message len %u, block_len %u)\n", btstack_crypto_ccm->message_len, btstack_crypto_ccm->block_len);
#endif
    if (btstack_crypto_ccm->message_len == 0u){
        btstack_crypto_ccm->state = CCM_CALCULATE_S0;
    } else {
        btstack_crypto_ccm->state = state_when_done;
        if (btstack_crypto_ccm->block_len == 0u){
            btstack_crypto_done(&btstack_crypto_ccm->btstack_crypto);
        }
    }
}

// If Controller is used for AES128, data is little endian
static void btstack_crypto_ccm_handle_s0(btstack_crypto_ccm_t * btstack_crypto_ccm, const uint8_t * data){
    int i;
    for (i=0;i<16;i++){
#ifdef USE_BTSTACK_AES128
        btstack_crypto_ccm->x_i[i] = btstack_crypto_ccm->x_i[i] ^ data[i];
#else
        btstack_crypto_ccm->x_i[i] = btstack_crypto_ccm->x_i[i] ^ data[15-i];
#endif
    }
    btstack_crypto_done(&btstack_crypto_ccm->btstack_crypto);
}

// If Controller is used for AES128, data is little endian
static void btstack_crypto_ccm_handle_sn(btstack_crypto_ccm_t * btstack_crypto_ccm, const uint8_t * data){
    int i;
    uint16_t bytes_to_process = btstack_min(btstack_crypto_ccm->block_len, 16);
    for (i=0;i<bytes_to_process;i++){
#ifdef USE_BTSTACK_AES128
        btstack_crypto_ccm->output[i] = btstack_crypto_ccm->input[i] ^ data[i];
#else
        btstack_crypto_ccm->output[i] = btstack_crypto_ccm->input[i] ^ data[15-i];
#endif
    }
    switch (btstack_crypto_ccm->btstack_crypto.operation){
        case BTSTACK_CRYPTO_CCM_DECRYPT_BLOCK:
            btstack_crypto_ccm->state = CCM_CALCULATE_XN;
            break;
        case BTSTACK_CRYPTO_CCM_ENCRYPT_BLOCK:
            btstack_crypto_ccm_next_block(btstack_crypto_ccm, CCM_CALCULATE_XN);
            break;
        default:
            btstack_assert(false);
            break;
    }
}

static void btstack_crypto_ccm_handle_aad_xn(btstack_crypto_ccm_t * btstack_crypto_ccm) {
#ifdef DEBUG_CCM
    printf("%16s: ", "Xn+1 AAD");
    printf_hexdump(btstack_crypto_ccm->x_i, 16);
#endif
    // more aad?
    if (btstack_crypto_ccm->aad_offset < (btstack_crypto_ccm->aad_len + 2u)){
        btstack_crypto_ccm->state = CCM_CALCULATE_AAD_XN;
    } else {
        // done
        btstack_crypto_done((btstack_crypto_t *) btstack_crypto_ccm);
    }
}

static void btstack_crypto_ccm_handle_x1(btstack_crypto_ccm_t * btstack_crypto_ccm) {
#ifdef DEBUG_CCM
    printf("%16s: ", "Xi");
    printf_hexdump(btstack_crypto_ccm->x_i, 16);
#endif
    switch (btstack_crypto_ccm->btstack_crypto.operation){
        case BTSTACK_CRYPTO_CCM_DIGEST_BLOCK:
            btstack_crypto_ccm->aad_remainder_len = 0;
            btstack_crypto_ccm->state = CCM_CALCULATE_AAD_XN;
            break;
        case BTSTACK_CRYPTO_CCM_DECRYPT_BLOCK:
            btstack_crypto_ccm->state = CCM_CALCULATE_SN;
            break;
        case BTSTACK_CRYPTO_CCM_ENCRYPT_BLOCK:
            btstack_crypto_ccm->state = CCM_CALCULATE_XN;
            break;
        default:
        btstack_assert(false);
            break;
    }
}

static void btstack_crypto_ccm_handle_xn(btstack_crypto_ccm_t * btstack_crypto_ccm) {
#ifdef DEBUG_CCM
    printf("%16s: ", "Xn+1");
    printf_hexdump(btstack_crypto_ccm->x_i, 16);
#endif
    switch (btstack_crypto_ccm->btstack_crypto.operation){
        case BTSTACK_CRYPTO_CCM_DECRYPT_BLOCK:
            btstack_crypto_ccm_next_block(btstack_crypto_ccm, CCM_CALCULATE_SN);
            break;
        case BTSTACK_CRYPTO_CCM_ENCRYPT_BLOCK:
            btstack_crypto_ccm->state = CCM_CALCULATE_SN;
            break;
        default:
        btstack_assert(false);
            break;
    }
}

static void btstack_crypto_ccm_calc_s0(btstack_crypto_ccm_t * btstack_crypto_ccm){
#ifdef DEBUG_CCM
    printf("btstack_crypto_ccm_calc_s0\n");
#endif
    btstack_crypto_ccm->state = CCM_W4_S0;
    btstack_crypto_ccm_setup_a_i(btstack_crypto_ccm, 0);
#ifdef USE_BTSTACK_AES128
    uint8_t data[16];
    btstack_aes128_calc(btstack_crypto_ccm->key, btstack_crypto_ccm_s, data);
    btstack_crypto_ccm_handle_s0(btstack_crypto_ccm, data);
#else
    btstack_crypto_aes128_start(btstack_crypto_ccm->key, btstack_crypto_ccm_s);
#endif
}

static void btstack_crypto_ccm_calc_sn(btstack_crypto_ccm_t * btstack_crypto_ccm){
#ifdef DEBUG_CCM
    printf("btstack_crypto_ccm_calc_s%u\n", btstack_crypto_ccm->counter);
#endif
    btstack_crypto_ccm->state = CCM_W4_SN;
    btstack_crypto_ccm_setup_a_i(btstack_crypto_ccm, btstack_crypto_ccm->counter);
#ifdef USE_BTSTACK_AES128
    uint8_t data[16];
    btstack_aes128_calc(btstack_crypto_ccm->key, btstack_crypto_ccm_s, data);
    btstack_crypto_ccm_handle_sn(btstack_crypto_ccm, data);
#else
    btstack_crypto_aes128_start(btstack_crypto_ccm->key, btstack_crypto_ccm_s);
#endif
}

static void btstack_crypto_ccm_calc_x1(btstack_crypto_ccm_t * btstack_crypto_ccm){
    uint8_t btstack_crypto_ccm_buffer[16];
    btstack_crypto_ccm->state = CCM_W4_X1;
    btstack_crypto_ccm_setup_b_0(btstack_crypto_ccm, btstack_crypto_ccm_buffer);
#ifdef USE_BTSTACK_AES128
    btstack_aes128_calc(btstack_crypto_ccm->key, btstack_crypto_ccm_buffer, btstack_crypto_ccm->x_i);
    btstack_crypto_ccm_handle_x1(btstack_crypto_ccm);
#else
    btstack_crypto_aes128_start(btstack_crypto_ccm->key, btstack_crypto_ccm_buffer);
#endif
}

static void btstack_crypto_ccm_calc_xn(btstack_crypto_ccm_t * btstack_crypto_ccm, const uint8_t * plaintext){
    uint8_t btstack_crypto_ccm_buffer[16];
    btstack_crypto_ccm->state = CCM_W4_XN;

#ifdef DEBUG_CCM
    printf("%16s: ", "bn");
    printf_hexdump(plaintext, 16);
#endif
    uint8_t i;
    uint16_t bytes_to_decrypt = btstack_crypto_ccm->block_len;
    // use explicit min implementation as c-stat worried about out-of-bounds-reads
    if (bytes_to_decrypt > 16u) {
        bytes_to_decrypt = 16;
    }
    for (i = 0; i < bytes_to_decrypt ; i++){
        btstack_crypto_ccm_buffer[i] =  btstack_crypto_ccm->x_i[i] ^ plaintext[i];
    }
    (void)memcpy(&btstack_crypto_ccm_buffer[i], &btstack_crypto_ccm->x_i[i],
                 16u - bytes_to_decrypt);
#ifdef DEBUG_CCM
    printf("%16s: ", "Xn XOR bn");
    printf_hexdump(btstack_crypto_ccm_buffer, 16);
#endif

#ifdef USE_BTSTACK_AES128
    btstack_aes128_calc(btstack_crypto_ccm->key, btstack_crypto_ccm_buffer, btstack_crypto_ccm->x_i);
    btstack_crypto_ccm_handle_xn(btstack_crypto_ccm);
#else
    btstack_crypto_aes128_start(btstack_crypto_ccm->key, btstack_crypto_ccm_buffer);
#endif
}

static void btstack_crypto_ccm_calc_aad_xn(btstack_crypto_ccm_t * btstack_crypto_ccm){
    // store length
    if (btstack_crypto_ccm->aad_offset == 0u){
        uint8_t len_buffer[2];
        big_endian_store_16(len_buffer, 0, btstack_crypto_ccm->aad_len);
        btstack_crypto_ccm->x_i[0] ^= len_buffer[0];
        btstack_crypto_ccm->x_i[1] ^= len_buffer[1];
        btstack_crypto_ccm->aad_remainder_len += 2u;
        btstack_crypto_ccm->aad_offset        += 2u;
    }

    // fill from input
    uint16_t bytes_free = 16u - btstack_crypto_ccm->aad_remainder_len;
    uint16_t bytes_to_copy = btstack_min(bytes_free, btstack_crypto_ccm->block_len);
    while (bytes_to_copy){
        btstack_crypto_ccm->x_i[btstack_crypto_ccm->aad_remainder_len++] ^= *btstack_crypto_ccm->input++;
        btstack_crypto_ccm->aad_offset++;
        btstack_crypto_ccm->block_len--;
        bytes_to_copy--;
        bytes_free--;
    }

    // if last block, fill with zeros
    if (btstack_crypto_ccm->aad_offset == (btstack_crypto_ccm->aad_len + 2u)){
        btstack_crypto_ccm->aad_remainder_len = 16;
    }
    // if not full, notify done
    if (btstack_crypto_ccm->aad_remainder_len < 16u){
        btstack_crypto_done(&btstack_crypto_ccm->btstack_crypto);
        return;
    }

    // encrypt block
#ifdef DEBUG_CCM
    printf("%16s: ", "Xn XOR Bn (aad)");
    printf_hexdump(btstack_crypto_ccm->x_i, 16);
#endif

    btstack_crypto_ccm->aad_remainder_len = 0;
    btstack_crypto_ccm->state = CCM_W4_AAD_XN;
#ifdef USE_BTSTACK_AES128
    btstack_aes128_calc(btstack_crypto_ccm->key, btstack_crypto_ccm->x_i, btstack_crypto_ccm->x_i);
    btstack_crypto_ccm_handle_aad_xn(btstack_crypto_ccm);
#else
    btstack_crypto_aes128_start(btstack_crypto_ccm->key, btstack_crypto_ccm->x_i);
#endif
}

static void btstack_crypto_run(void){

    btstack_crypto_aes128_t        * btstack_crypto_aes128;
    btstack_crypto_ccm_t           * btstack_crypto_ccm;
    btstack_crypto_aes128_cmac_t   * btstack_crypto_cmac;
#ifdef ENABLE_ECC_P256
    btstack_crypto_ecc_p256_t      * btstack_crypto_ec_p192;
#endif

    // stack up and running?
    if (hci_get_state() != HCI_STATE_WORKING) return;

    // try to do as much as possible
    while (true){

        // anything to do?
        if (btstack_linked_list_empty(&btstack_crypto_operations)) return;

        // already active?
        if (btstack_crypto_wait_for_hci_result) return;

        // can send a command?
        if (!hci_can_send_command_packet_now()) return;

        // ok, find next task
    	btstack_crypto_t * btstack_crypto = (btstack_crypto_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
    	switch (btstack_crypto->operation){
    		case BTSTACK_CRYPTO_RANDOM:
    			btstack_crypto_wait_for_hci_result = true;
    		    hci_cmd_send(&hci_le_rand);
    		    break;
    		case BTSTACK_CRYPTO_AES128:
                btstack_crypto_aes128 = (btstack_crypto_aes128_t *) btstack_crypto;
#ifdef USE_BTSTACK_AES128
                btstack_aes128_calc(btstack_crypto_aes128->key, btstack_crypto_aes128->plaintext, btstack_crypto_aes128->ciphertext);
                btstack_crypto_done(btstack_crypto);
#else
                btstack_crypto_aes128_start(btstack_crypto_aes128->key, btstack_crypto_aes128->plaintext);
#endif
    		    break;

    		case BTSTACK_CRYPTO_CMAC_MESSAGE:
    		case BTSTACK_CRYPTO_CMAC_GENERATOR:
                btstack_crypto_cmac = (btstack_crypto_aes128_cmac_t *) btstack_crypto;
#ifdef USE_BTSTACK_AES128
                btstack_crypto_cmac_calc( btstack_crypto_cmac );
                btstack_crypto_done(btstack_crypto);
#else
    			btstack_crypto_wait_for_hci_result = 1;
    			if (btstack_crypto_cmac_state == CMAC_IDLE){
    				btstack_crypto_cmac_start(btstack_crypto_cmac);
    			} else {
    				btstack_crypto_cmac_handle_aes_engine_ready(btstack_crypto_cmac);
    			}
#endif
    			break;

            case BTSTACK_CRYPTO_CCM_DIGEST_BLOCK:
            case BTSTACK_CRYPTO_CCM_ENCRYPT_BLOCK:
            case BTSTACK_CRYPTO_CCM_DECRYPT_BLOCK:
                btstack_crypto_ccm = (btstack_crypto_ccm_t *) btstack_crypto;
                switch (btstack_crypto_ccm->state){
                    case CCM_CALCULATE_AAD_XN:
#ifdef DEBUG_CCM
                        printf("CCM_CALCULATE_AAD_XN\n");
#endif
                        btstack_crypto_ccm_calc_aad_xn(btstack_crypto_ccm);
                        break;
                    case CCM_CALCULATE_X1:
#ifdef DEBUG_CCM
                        printf("CCM_CALCULATE_X1\n");
#endif
                        btstack_crypto_ccm_calc_x1(btstack_crypto_ccm);
                        break;
                    case CCM_CALCULATE_S0:
#ifdef DEBUG_CCM
                        printf("CCM_CALCULATE_S0\n");
#endif
                        btstack_crypto_ccm_calc_s0(btstack_crypto_ccm);
                        break;
                    case CCM_CALCULATE_SN:
#ifdef DEBUG_CCM
                        printf("CCM_CALCULATE_SN\n");
#endif
                        btstack_crypto_ccm_calc_sn(btstack_crypto_ccm);
                        break;
                    case CCM_CALCULATE_XN:
#ifdef DEBUG_CCM
                        printf("CCM_CALCULATE_XN\n");
#endif
                        btstack_crypto_ccm_calc_xn(btstack_crypto_ccm, (btstack_crypto->operation == BTSTACK_CRYPTO_CCM_ENCRYPT_BLOCK) ? btstack_crypto_ccm->input : btstack_crypto_ccm->output);
                        break;
                    default:
                        break;
                }
                break;

#ifdef ENABLE_ECC_P256
            case BTSTACK_CRYPTO_ECC_P256_GENERATE_KEY:
                btstack_crypto_ec_p192 = (btstack_crypto_ecc_p256_t *) btstack_crypto;
                switch (btstack_crypto_ecc_p256_key_generation_state){
                    case ECC_P256_KEY_GENERATION_DONE:
                        // done
                        btstack_crypto_log_ec_publickey(btstack_crypto_ecc_p256_public_key);
                        (void)memcpy(btstack_crypto_ec_p192->public_key,
                                     btstack_crypto_ecc_p256_public_key, 64);
                        btstack_linked_list_pop(&btstack_crypto_operations);
                        (*btstack_crypto_ec_p192->btstack_crypto.context_callback.callback)(btstack_crypto_ec_p192->btstack_crypto.context_callback.context);
                        break;
                    case ECC_P256_KEY_GENERATION_IDLE:
#ifdef USE_SOFTWARE_ECC_P256_IMPLEMENTATION
                        log_info("start ecc random");
                        btstack_crypto_ecc_p256_key_generation_state = ECC_P256_KEY_GENERATION_GENERATING_RANDOM;
                        btstack_crypto_ecc_p256_random_len = 0;
                        btstack_crypto_wait_for_hci_result = true;
                        hci_cmd_send(&hci_le_rand);
#else
                        btstack_crypto_ecc_p256_key_generation_state = ECC_P256_KEY_GENERATION_W4_KEY;
                        btstack_crypto_wait_for_hci_result = 1;
                        hci_cmd_send(&hci_le_read_local_p256_public_key);
#endif
                        break;
#ifdef USE_SOFTWARE_ECC_P256_IMPLEMENTATION
                    case ECC_P256_KEY_GENERATION_GENERATING_RANDOM:
                        log_info("more ecc random");
                        btstack_crypto_wait_for_hci_result = true;
                        hci_cmd_send(&hci_le_rand);
                        break;
#endif
                    default:
                        break;
                }
                break;
            case BTSTACK_CRYPTO_ECC_P256_CALCULATE_DHKEY:
                btstack_crypto_ec_p192 = (btstack_crypto_ecc_p256_t *) btstack_crypto;
#ifdef USE_SOFTWARE_ECC_P256_IMPLEMENTATION
                btstack_crypto_ecc_p256_calculate_dhkey_software(btstack_crypto_ec_p192);
                // done
                btstack_linked_list_pop(&btstack_crypto_operations);
                (*btstack_crypto_ec_p192->btstack_crypto.context_callback.callback)(btstack_crypto_ec_p192->btstack_crypto.context_callback.context);
#else
                btstack_crypto_wait_for_hci_result = 1;
                hci_cmd_send(&hci_le_generate_dhkey, &btstack_crypto_ec_p192->public_key[0], &btstack_crypto_ec_p192->public_key[32]);
#endif
                break;

#endif /* ENABLE_ECC_P256 */

            default:
                break;
        }
    }
}

static void btstack_crypto_handle_random_data(const uint8_t * data, uint16_t len){
    btstack_crypto_random_t * btstack_crypto_random;
    btstack_crypto_t * btstack_crypto = (btstack_crypto_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
    uint16_t bytes_to_copy;
	if (!btstack_crypto) return;
    switch (btstack_crypto->operation){
        case BTSTACK_CRYPTO_RANDOM:
            btstack_crypto_random = (btstack_crypto_random_t*) btstack_crypto;
            bytes_to_copy = btstack_min(btstack_crypto_random->size, len);
            (void)memcpy(btstack_crypto_random->buffer, data, bytes_to_copy);
            btstack_crypto_random->buffer += bytes_to_copy;
            btstack_crypto_random->size   -= bytes_to_copy;
            // data processed, more?
            if (!btstack_crypto_random->size) {
                // done
                btstack_linked_list_pop(&btstack_crypto_operations);
                (*btstack_crypto_random->btstack_crypto.context_callback.callback)(btstack_crypto_random->btstack_crypto.context_callback.context);
            }
            break;
#ifdef ENABLE_ECC_P256
        case BTSTACK_CRYPTO_ECC_P256_GENERATE_KEY:
            btstack_assert((btstack_crypto_ecc_p256_random_len + 8) <= 64);
            (void)memcpy(&btstack_crypto_ecc_p256_random[btstack_crypto_ecc_p256_random_len], data, 8);
            btstack_crypto_ecc_p256_random_len += 8u;
            if (btstack_crypto_ecc_p256_random_len >= 64u) {
                btstack_crypto_ecc_p256_key_generation_state = ECC_P256_KEY_GENERATION_ACTIVE;
                btstack_crypto_ecc_p256_generate_key_software();
                btstack_crypto_ecc_p256_key_generation_state = ECC_P256_KEY_GENERATION_DONE;
            }
            break;
#endif
        default:
            break;
    }
	// more work?
	btstack_crypto_run();
}

#ifndef USE_BTSTACK_AES128
static void btstack_crypto_handle_encryption_result(const uint8_t * data){
	btstack_crypto_aes128_t      * btstack_crypto_aes128;
	btstack_crypto_aes128_cmac_t * btstack_crypto_cmac;
    btstack_crypto_ccm_t         * btstack_crypto_ccm;
	uint8_t result[16];

    btstack_crypto_t * btstack_crypto = (btstack_crypto_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
	if (!btstack_crypto) return;
	switch (btstack_crypto->operation){
		case BTSTACK_CRYPTO_AES128:
			btstack_crypto_aes128 = (btstack_crypto_aes128_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
		    reverse_128(data, btstack_crypto_aes128->ciphertext);
            btstack_crypto_done(btstack_crypto);
			break;
		case BTSTACK_CRYPTO_CMAC_GENERATOR:
		case BTSTACK_CRYPTO_CMAC_MESSAGE:
			btstack_crypto_cmac = (btstack_crypto_aes128_cmac_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
		    reverse_128(data, result);
		    btstack_crypto_cmac_handle_encryption_result(btstack_crypto_cmac, result);
			break;
        case BTSTACK_CRYPTO_CCM_DIGEST_BLOCK:
        case BTSTACK_CRYPTO_CCM_ENCRYPT_BLOCK:
        case BTSTACK_CRYPTO_CCM_DECRYPT_BLOCK:
            btstack_crypto_ccm = (btstack_crypto_ccm_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
            switch (btstack_crypto_ccm->state){
                case CCM_W4_X1:
                    reverse_128(data, btstack_crypto_ccm->x_i);
                    btstack_crypto_ccm_handle_x1(btstack_crypto_ccm);
                    break;
                case CCM_W4_XN:
                    reverse_128(data, btstack_crypto_ccm->x_i);
                    btstack_crypto_ccm_handle_xn(btstack_crypto_ccm);
                    break;
                case CCM_W4_AAD_XN:
                    reverse_128(data, btstack_crypto_ccm->x_i);
                    btstack_crypto_ccm_handle_aad_xn(btstack_crypto_ccm);
                    break;
                case CCM_W4_S0:
                    btstack_crypto_ccm_handle_s0(btstack_crypto_ccm, data);
                    break;
                case CCM_W4_SN:
                    btstack_crypto_ccm_handle_sn(btstack_crypto_ccm, data);
                    break;
                default:
                    break;
            }
            break;
		default:
			break;
	}
}
#endif

static void btstack_crypto_event_handler(uint8_t packet_type, uint16_t cid, uint8_t *packet, uint16_t size){
    UNUSED(cid);         // ok: there is no channel
    UNUSED(size);        // ok: fixed format events read from HCI buffer

#ifdef ENABLE_ECC_P256
#ifndef USE_SOFTWARE_ECC_P256_IMPLEMENTATION
    btstack_crypto_ecc_p256_t * btstack_crypto_ec_p192;
#endif
#endif
    bool ecdh_operations_supported;

    if (packet_type != HCI_EVENT_PACKET)  return;

    switch (hci_event_packet_get_type(packet)){
        case BTSTACK_EVENT_STATE:
            switch(btstack_event_state_get_state(packet)){
                case HCI_STATE_HALTING:
                    // as stack is shutting down, reset state
                    btstack_crypto_state_reset();
                    break;
                default:
                    break;
            }
            if (btstack_event_state_get_state(packet) != HCI_STATE_HALTING) break;
            break;

        case HCI_EVENT_COMMAND_COMPLETE:
            switch (hci_event_command_complete_get_command_opcode(packet)){
#ifndef USE_BTSTACK_AES128
                case HCI_OPCODE_HCI_LE_ENCRYPT:
                    if (!btstack_crypto_wait_for_hci_result) return;
                    btstack_crypto_wait_for_hci_result = 0;
    	            btstack_crypto_handle_encryption_result(&packet[6]);
                    break;
#endif
                case HCI_OPCODE_HCI_LE_RAND:
                    if (!btstack_crypto_wait_for_hci_result) return;
                    btstack_crypto_wait_for_hci_result = false;
                    btstack_crypto_handle_random_data(&packet[6], 8);
                    break;
                case HCI_OPCODE_HCI_READ_LOCAL_SUPPORTED_COMMANDS:
                    ecdh_operations_supported = (packet[OFFSET_OF_DATA_IN_COMMAND_COMPLETE+1u+34u] & 0x06u) == 0x06u;
                    UNUSED(ecdh_operations_supported);
                    log_info("controller supports ECDH operation: %u", ecdh_operations_supported);
#ifdef ENABLE_ECC_P256
#ifndef USE_SOFTWARE_ECC_P256_IMPLEMENTATION
                    // Assert controller supports ECDH operation if we don't implement them ourselves
                    // Please add ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS to btstack_config.h and add 3rd-party/micro-ecc to your port
                    btstack_assert(ecdh_operations_supported != 0);
#endif
#endif
                    break;
                default:
                    break;
            }
            break;

#ifdef ENABLE_ECC_P256
#ifndef USE_SOFTWARE_ECC_P256_IMPLEMENTATION
        case HCI_EVENT_LE_META:
            btstack_crypto_ec_p192 = (btstack_crypto_ecc_p256_t*) btstack_linked_list_get_first_item(&btstack_crypto_operations);
            if (!btstack_crypto_ec_p192) break;
            switch (hci_event_le_meta_get_subevent_code(packet)){
                case HCI_SUBEVENT_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE:
                    if (btstack_crypto_ec_p192->btstack_crypto.operation != BTSTACK_CRYPTO_ECC_P256_GENERATE_KEY) break;
                    if (!btstack_crypto_wait_for_hci_result) return;
                    btstack_crypto_wait_for_hci_result = 0;
                    if (hci_subevent_le_read_local_p256_public_key_complete_get_status(packet)){
                        log_error("Read Local P256 Public Key failed");
                    }
                    hci_subevent_le_read_local_p256_public_key_complete_get_dhkey_x(packet, &btstack_crypto_ecc_p256_public_key[0]);
                    hci_subevent_le_read_local_p256_public_key_complete_get_dhkey_y(packet, &btstack_crypto_ecc_p256_public_key[32]);
                    btstack_crypto_ecc_p256_key_generation_state = ECC_P256_KEY_GENERATION_DONE;
                    break;
                case HCI_SUBEVENT_LE_GENERATE_DHKEY_COMPLETE:
                    if (btstack_crypto_ec_p192->btstack_crypto.operation != BTSTACK_CRYPTO_ECC_P256_CALCULATE_DHKEY) break;
                    if (!btstack_crypto_wait_for_hci_result) return;
                    btstack_crypto_wait_for_hci_result = 0;
                    if (hci_subevent_le_generate_dhkey_complete_get_status(packet)){
                        log_error("Generate DHKEY failed -> abort");
                        // set DHKEY to 0xff..ff
                        memset(btstack_crypto_ec_p192->dhkey, 0xff, 32);
                    } else {
                        hci_subevent_le_generate_dhkey_complete_get_dhkey(packet, btstack_crypto_ec_p192->dhkey);
                    }
                    // done
                    btstack_linked_list_pop(&btstack_crypto_operations);
                    (*btstack_crypto_ec_p192->btstack_crypto.context_callback.callback)(btstack_crypto_ec_p192->btstack_crypto.context_callback.context);                    
                    break;
                default:
                    break;                
            }
            break;
#endif
#endif
        default:
            break;
    }

    // try processing
	btstack_crypto_run();    
}

void btstack_crypto_random_generate(btstack_crypto_random_t * request, uint8_t * buffer, uint16_t size, void (* callback)(void * arg), void * callback_arg){
	request->btstack_crypto.context_callback.callback  = callback;
	request->btstack_crypto.context_callback.context   = callback_arg;
	request->btstack_crypto.operation         		   = BTSTACK_CRYPTO_RANDOM;
	request->buffer = buffer;
	request->size   = size;
	btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
	btstack_crypto_run();
}

void btstack_crypto_aes128_encrypt(btstack_crypto_aes128_t * request, const uint8_t * key, const uint8_t * plaintext, uint8_t * ciphertext, void (* callback)(void * arg), void * callback_arg){
	request->btstack_crypto.context_callback.callback  = callback;
	request->btstack_crypto.context_callback.context   = callback_arg;
	request->btstack_crypto.operation         		   = BTSTACK_CRYPTO_AES128;
	request->key 									   = key;
	request->plaintext      					       = plaintext;
	request->ciphertext 							   = ciphertext;
	btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
	btstack_crypto_run();
}

void btstack_crypto_aes128_cmac_generator(btstack_crypto_aes128_cmac_t * request, const uint8_t * key, uint16_t size, uint8_t (*get_byte_callback)(uint16_t pos), uint8_t * hash, void (* callback)(void * arg), void * callback_arg){
	request->btstack_crypto.context_callback.callback  = callback;
	request->btstack_crypto.context_callback.context   = callback_arg;
	request->btstack_crypto.operation         		   = BTSTACK_CRYPTO_CMAC_GENERATOR;
	request->key 									   = key;
	request->size 									   = size;
	request->data.get_byte_callback					   = get_byte_callback;
	request->hash 									   = hash;
	btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
	btstack_crypto_run();
}

void btstack_crypto_aes128_cmac_message(btstack_crypto_aes128_cmac_t * request, const uint8_t * key, uint16_t size, const uint8_t * message, uint8_t * hash, void (* callback)(void * arg), void * callback_arg){
	request->btstack_crypto.context_callback.callback  = callback;
	request->btstack_crypto.context_callback.context   = callback_arg;
	request->btstack_crypto.operation         		   = BTSTACK_CRYPTO_CMAC_MESSAGE;
	request->key 									   = key;
	request->size 									   = size;
	request->data.message      						   = message;
	request->hash 									   = hash;
	btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
	btstack_crypto_run();
}

void btstack_crypto_aes128_cmac_zero(btstack_crypto_aes128_cmac_t * request, uint16_t size, const uint8_t * message,  uint8_t * hash, void (* callback)(void * arg), void * callback_arg){
    request->btstack_crypto.context_callback.callback  = callback;
    request->btstack_crypto.context_callback.context   = callback_arg;
    request->btstack_crypto.operation                  = BTSTACK_CRYPTO_CMAC_MESSAGE;
    request->key                                       = zero;
    request->size                                      = size;
    request->data.message                              = message;
    request->hash                                      = hash;
    btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
    btstack_crypto_run();
}

#ifdef ENABLE_ECC_P256
void btstack_crypto_ecc_p256_generate_key(btstack_crypto_ecc_p256_t * request, uint8_t * public_key, void (* callback)(void * arg), void * callback_arg){
    // reset key generation
    if (btstack_crypto_ecc_p256_key_generation_state == ECC_P256_KEY_GENERATION_DONE){
        btstack_crypto_ecc_p256_key_generation_state = ECC_P256_KEY_GENERATION_IDLE;
    }
    request->btstack_crypto.context_callback.callback  = callback;
    request->btstack_crypto.context_callback.context   = callback_arg;
    request->btstack_crypto.operation                  = BTSTACK_CRYPTO_ECC_P256_GENERATE_KEY;
    request->public_key                                = public_key;
    btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
    btstack_crypto_run();
}

void btstack_crypto_ecc_p256_calculate_dhkey(btstack_crypto_ecc_p256_t * request, const uint8_t * public_key, uint8_t * dhkey, void (* callback)(void * arg), void * callback_arg){
    request->btstack_crypto.context_callback.callback  = callback;
    request->btstack_crypto.context_callback.context   = callback_arg;
    request->btstack_crypto.operation                  = BTSTACK_CRYPTO_ECC_P256_CALCULATE_DHKEY;
    request->public_key                                = (uint8_t *) public_key;
    request->dhkey                                     = dhkey;
    btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
    btstack_crypto_run();
}

int btstack_crypto_ecc_p256_validate_public_key(const uint8_t * public_key){

    int err = 0;

#ifdef USE_MICRO_ECC_P256
    // validate public key using micro-ecc

#if uECC_SUPPORTS_secp256r1
    // standard version
    err = uECC_valid_public_key(public_key, uECC_secp256r1()) == 0;
#else
    // static version
    err = uECC_valid_public_key(public_key) == 0;
#endif
#endif

#ifdef USE_MBEDTLS_ECC_P256
    // validate public using mbedtls_ecc

    mbedtls_ecp_point Q;
    mbedtls_ecp_point_init( &Q );
    mbedtls_mpi_read_binary(&Q.X, &public_key[0], 32);
    mbedtls_mpi_read_binary(&Q.Y, &public_key[32], 32);
    mbedtls_mpi_lset(&Q.Z, 1);
    err = mbedtls_ecp_check_pubkey(&mbedtls_ec_group, &Q);
    mbedtls_ecp_point_free( & Q);
#endif

    if (err != 0){
        log_info("public key invalid %x", err);
    }
    return  err;
}
#endif

void btstack_crypto_ccm_init(btstack_crypto_ccm_t * request, const uint8_t * key, const uint8_t * nonce, uint16_t message_len, uint16_t additional_authenticated_data_len, uint8_t auth_len){
    request->key         = key;
    request->nonce       = nonce;
    request->message_len = message_len;
    request->aad_len     = additional_authenticated_data_len;
    request->aad_offset  = 0;
    request->auth_len    = auth_len;
    request->counter     = 1;
    request->state       = CCM_CALCULATE_X1;
}

void btstack_crypto_ccm_digest(btstack_crypto_ccm_t * request, uint8_t * additional_authenticated_data, uint16_t additional_authenticated_data_len, void (* callback)(void * arg), void * callback_arg){
    // not implemented yet
    request->btstack_crypto.context_callback.callback  = callback;
    request->btstack_crypto.context_callback.context   = callback_arg;
    request->btstack_crypto.operation                  = BTSTACK_CRYPTO_CCM_DIGEST_BLOCK;
    request->block_len                                 = additional_authenticated_data_len;
    request->input                                     = additional_authenticated_data;
    btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
    btstack_crypto_run();
}

void btstack_crypto_ccm_get_authentication_value(btstack_crypto_ccm_t * request, uint8_t * authentication_value){
    (void)memcpy(authentication_value, request->x_i, request->auth_len);
}

void btstack_crypto_ccm_encrypt_block(btstack_crypto_ccm_t * request, uint16_t len, const uint8_t * plaintext, uint8_t * ciphertext, void (* callback)(void * arg), void * callback_arg){
#ifdef DEBUG_CCM
    printf("\nbtstack_crypto_ccm_encrypt_block, len %u\n", len);
#endif
    request->btstack_crypto.context_callback.callback  = callback;
    request->btstack_crypto.context_callback.context   = callback_arg;
    request->btstack_crypto.operation                  = BTSTACK_CRYPTO_CCM_ENCRYPT_BLOCK;
    request->block_len                                 = len;
    request->input                                     = plaintext;
    request->output                                    = ciphertext;
    if (request->state != CCM_CALCULATE_X1){
        request->state  = CCM_CALCULATE_XN;
    }
    btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
    btstack_crypto_run();
}

void btstack_crypto_ccm_decrypt_block(btstack_crypto_ccm_t * request, uint16_t len, const uint8_t * ciphertext, uint8_t * plaintext, void (* callback)(void * arg), void * callback_arg){
    request->btstack_crypto.context_callback.callback  = callback;
    request->btstack_crypto.context_callback.context   = callback_arg;
    request->btstack_crypto.operation                  = BTSTACK_CRYPTO_CCM_DECRYPT_BLOCK;
    request->block_len                                 = len;
    request->input                                     = ciphertext;
    request->output                                    = plaintext;
    if (request->state != CCM_CALCULATE_X1){
        request->state  = CCM_CALCULATE_SN;
    }
    btstack_linked_list_add_tail(&btstack_crypto_operations, (btstack_linked_item_t*) request);
    btstack_crypto_run();
}


static void btstack_crypto_state_reset(void) {
#ifndef USE_BTSTACK_AES128
    btstack_crypto_cmac_state = CMAC_IDLE;
#endif
#ifdef ENABLE_ECC_P256
    btstack_crypto_ecc_p256_key_generation_state = ECC_P256_KEY_GENERATION_IDLE;
#endif
    btstack_crypto_wait_for_hci_result = false;
    btstack_crypto_operations = NULL;
}

void btstack_crypto_init(void){
    if (btstack_crypto_initialized) return;
    btstack_crypto_initialized = true;

    // register with HCI
    hci_event_callback_registration.callback = &btstack_crypto_event_handler;
    hci_add_event_handler(&hci_event_callback_registration);

#ifdef USE_MBEDTLS_ECC_P256
    mbedtls_ecp_group_init(&mbedtls_ec_group);
	mbedtls_ecp_group_load(&mbedtls_ec_group, MBEDTLS_ECP_DP_SECP256R1);
#endif

    // reset state
    btstack_crypto_state_reset();
}

// De-Init
void btstack_crypto_deinit(void) {
    btstack_crypto_initialized = false;
}

// PTS only
void btstack_crypto_ecc_p256_set_key(const uint8_t * public_key, const uint8_t * private_key){
#ifdef USE_SOFTWARE_ECC_P256_IMPLEMENTATION
    (void)memcpy(btstack_crypto_ecc_p256_d, private_key, 32);
    (void)memcpy(btstack_crypto_ecc_p256_public_key, public_key, 64);
    btstack_crypto_ecc_p256_key_generation_state = ECC_P256_KEY_GENERATION_DONE;
#else
    UNUSED(public_key);
    UNUSED(private_key);
#endif
}

// Unit testing
int btstack_crypto_idle(void){
    return btstack_linked_list_empty(&btstack_crypto_operations);
}
void btstack_crypto_reset(void){
    btstack_crypto_deinit();
    btstack_crypto_init();
}
