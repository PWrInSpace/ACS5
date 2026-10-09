#pragma once

#if defined(__GNUC__)
#define WARN_UNUSED __attribute__((warn_unused_result))
#else
#define WARN_UNUSED
#endif
