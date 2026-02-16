#ifndef DMO_SKUS_H
#define DMO_SKUS_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
  const char* name;
  const uint8_t* blocks;
} DmoSku;

extern const DmoSku DMO_SKUS[];
extern const size_t DMO_SKUS_COUNT;

const DmoSku* DMO_FindSkuByName(const char* name);

#endif /* DMO_SKUS_H */
