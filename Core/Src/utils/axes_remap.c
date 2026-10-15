#include "utils/axes_remap.h"

bool axes_remap_is_rotation(const axes_remap_t *r)
{
    bool seen[3] = {false, false, false};
    int  parity  = 1;

    for (int i = 0; i < 3; i++) {
        int8_t a = r->map[i];
        if (a < 0 || a > 2 || seen[a]) return false;
        seen[a] = true;
        if (r->sign[i] != 1 && r->sign[i] != -1) return false;
    }

    for (int i = 0; i < 3; i++)
        for (int j = i + 1; j < 3; j++)
            if (r->map[i] > r->map[j]) parity = -parity;

    int det = parity * r->sign[0] * r->sign[1] * r->sign[2];
    return det == 1;
}

void axes_remap_apply(const axes_remap_t *r, const float in[3], float out[3])
{
    float t[3];
    for (int i = 0; i < 3; i++)
        t[i] = (float)r->sign[i] * in[r->map[i]];
    for (int i = 0; i < 3; i++)
        out[i] = t[i];
}
