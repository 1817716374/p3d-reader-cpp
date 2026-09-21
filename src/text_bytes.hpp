#pragma once
#include <p3d/reader.hpp>
namespace p3d {
Json decode_text_bytes(const Bytes &);
Json decode_native_text_record(const Bytes &);
Json decode_native_font_record(const Bytes &);
} // namespace p3d
