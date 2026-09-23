#pragma once
#include <p3d/reader.hpp>

namespace p3d {
// Decodes exactly eight bytes of the native binary64 modification-time field.
// The SDK defines milliseconds from 1970-01-01 local time, not a UTC instant.
// Keeps fractional values and all source bits; performs no date conversion.
Json decode_native_modification_time(const Bytes &);
} // namespace p3d
