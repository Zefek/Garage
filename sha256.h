#ifndef SHA256_H
#define SHA256_H

#include <stddef.h>
#include <stdint.h>

struct Sha256Ctx {
  uint8_t data[64];
  uint32_t datalen;
  uint64_t bitlen;
  uint32_t state[8];
};

void sha256_init(Sha256Ctx* ctx);
void sha256_update(Sha256Ctx* ctx, const uint8_t* data, size_t len);
void sha256_final(Sha256Ctx* ctx, uint8_t hash[32]);
void hmac_sha256(const uint8_t* key, size_t keylen, const uint8_t* msg, size_t msglen, uint8_t out[32]);

#endif
