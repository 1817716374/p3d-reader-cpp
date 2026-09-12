#include "internal.hpp"

namespace p3d {
Json native_application_record(const Bytes &base) {
    Json out = {{"encoding", "native_application_record"},
                {"reader_profile", "bimbase_2025_application_input"},
                {"status", "invalid"},
                {"runtime_resolution", "not_evaluated"}};
    try {
        require(base.size() >= 38 && Reader(base, 4).u16() == 47 && Reader(base, 16).u32() == 20,
                "application record requires type 47/subtype 20 and a signature");
        const auto signature = Reader(base, 36).u16();
        out["signature"] = signature;
        out["signature_source_offset"] = 36;
        if (signature == 0x5704) {
            require(base.size() >= 44, "owner application marker header is truncated");
            out["role"] = "owner_application_marker";
            out["status"] = "partial";
            out["unassigned_ranges"] = Json::array(
                {{{"source_offset", 38}, {"data", rawbytes(slice(base, 38, base.size() - 38))}}});
            return out;
        }
        require(signature == 0x56df, "unsupported application signature");
        require(base.size() >= 42, "versioned application header is truncated");
        out["role"] = "versioned_application_payload";
        // These are signed in the input routine. The second value is passed to
        // the version adapter, not used as the resource decoder's byte bound.
        const auto version = Reader(base).at<std::int16_t>(38);
        out["version"] = version;
        out["declared_storage_length"] = Reader(base).at<std::int16_t>(40);
        const std::uint64_t end = std::uint64_t(Reader(base, 12).u32()) * 2 + 4;
        require(end >= 42 && end <= base.size(), "application native base extent is invalid");
        out["decoder_input"] = {{"source_offset", 42},
                                {"data", rawbytes(slice(base, 42, std::size_t(end - 42)))}};
        if (end < base.size())
            out["outside_native_base"] = {
                {"source_offset", end},
                {"data", rawbytes(slice(base, std::size_t(end), base.size() - std::size_t(end)))}};
        out["conversion"] = {{"status", "not_evaluated"},
                             {"attempt_versions", Json::array()},
                             {"stop_after_success", true},
                             {"on_failure", "initialize_defaults"},
                             {"payload_field_semantics", "unassigned"}};
        if (version < 0 || version >= 8) {
            out["status"] = "unsupported";
            out["conversion"]["dispatch"] = "initialize_defaults";
        } else {
            out["status"] = "partial";
            out["conversion"]["dispatch"] = "resource_decoder_then_version_adapter";
            out["conversion"]["attempt_versions"].push_back(version);
            if (version != 0)
                out["conversion"]["attempt_versions"].push_back(version - 1);
        }
    } catch (const std::exception &e) {
        out["error"] = e.what();
    }
    return out;
}
} // namespace p3d
