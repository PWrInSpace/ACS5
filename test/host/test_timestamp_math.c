// Wrap behavior of the 32->64-bit DWT counter extension.
#include <assert.h>
#include <stdio.h>

#include "utils/timestamp_math.h"

int main(void)
{
    counter64_t c = {0};

    assert(counter64_extend(&c, 1000u) == 1000u);
    assert(counter64_extend(&c, 5000u) == 5000u);

    /* Counter wraps between calls: 0xFFFFF000 -> 0x100 is 0x1100 cycles. */
    c = (counter64_t){0};
    counter64_extend(&c, 0xFFFFF000u);
    uint64_t after = counter64_extend(&c, 0x100u);
    assert(after == 0xFFFFF000ull + 0x1100ull);

    /* Repeated wraps stay monotonic. */
    uint64_t prev = after;
    uint32_t raw  = 0x100u;
    for (int i = 0; i < 10; i++) {
        raw += 0x80000000u;
        after = counter64_extend(&c, raw);
        assert(after > prev);
        prev = after;
    }

    printf("timestamp_math: OK\n");
    return 0;
}
