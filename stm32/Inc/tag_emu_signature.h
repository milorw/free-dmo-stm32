#ifndef TAG_EMU_SIGNATURE_H
#define TAG_EMU_SIGNATURE_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint8_t uid[8];
  uint8_t signature[32];
} DmoTagEmuPair;

extern const DmoTagEmuPair DMO_TAG_EMU_PAIRS[];
extern const size_t DMO_TAG_EMU_PAIRS_COUNT;

#endif /* TAG_EMU_SIGNATURE_H */
