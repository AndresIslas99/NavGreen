// Name validation for operator-supplied identifiers (map names, zone ids).
//
// Map names become filesystem paths AND are interpolated into the
// map_saver_cli command line executed via popen; zone ids are embedded
// into hand-built JSON and matched by substring search. Rejecting
// everything outside a strict identifier charset closes shell injection,
// path traversal, and JSON-breaking quotes in one place.

#pragma once

#include <string>

namespace agv_map_manager {

// Accepts 1-64 characters from [A-Za-z0-9_-]. Anything else (slashes,
// dots, quotes, spaces, shell metacharacters) is rejected.
inline bool is_safe_name(const std::string& name) {
  if (name.empty() || name.size() > 64) return false;
  for (const char c : name) {
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                    (c >= '0' && c <= '9') || c == '_' || c == '-';
    if (!ok) return false;
  }
  return true;
}

}  // namespace agv_map_manager
