#pragma once

#if defined(__GNUC__)
#define WARN_UNUSED         __attribute__((warn_unused_result))
#define ATTR_SECTION(name)  __attribute__((section(name)))
#define ATTR_ALIGNED(n)     __attribute__((aligned(n)))
#else
#define WARN_UNUSED
#define ATTR_SECTION(name)
#define ATTR_ALIGNED(n)
#endif
