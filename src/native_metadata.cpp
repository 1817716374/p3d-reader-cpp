#include "internal.hpp"
#include <p3d/native_metadata.hpp>

namespace p3d {
Json decode_native_modification_time(const Bytes &bytes) {
    Json out = {{"status", "not_evaluated"}, {"source_storage", rawbytes(bytes)}};
    if (bytes.size() != 8) {
        out["reason"] = "native_modification_time_requires_8_bytes";
        return out;
    }
    const auto value = Reader(bytes).f64();
    out.update({{"status", "decoded"},
                {"milliseconds", std::isfinite(value) ? Json(value) : Json(nullptr)},
                {"finite", std::isfinite(value)},
                {"epoch", "1970-01-01T00:00:00"},
                {"time_basis", "local"},
                {"source_offset", 0}});
    return out;
}
} // namespace p3d
