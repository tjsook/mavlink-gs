#include "core/version.hpp"

#ifndef MGS_VERSION
#define MGS_VERSION "0.0.0-unknown"
#endif

namespace mgs {

const char* version() noexcept { return MGS_VERSION; }

}  // namespace mgs
