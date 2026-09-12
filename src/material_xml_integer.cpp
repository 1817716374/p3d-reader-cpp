#include "internal.hpp"

namespace p3d {
Json material_xml_integer(const Json &attributes, const std::string &key, bool signed_value) {
    Json out = {{"source_keys", Json::array({key})},
                {"status", "missing"},
                {"value", nullptr},
                {"conversion",
                 {{"reader", "ucrt_decimal_integer"},
                  {"format", signed_value ? "%d" : "%u"},
                  {"locale", "C"},
                  {"status", "not_present"}}}};
    if (!attributes.contains(key))
        return out;
    auto &conversion = out["conversion"];
    if (!attributes.at(key).is_string()) {
        out["status"] = "unresolved";
        conversion["status"] = "requires_xml_attribute_text";
        return out;
    }
    const auto text = attributes.at(key).get<std::string>();
    out["source_text"] = text;
    std::size_t pos = 0;
    auto space = [](unsigned char c) { return c == ' ' || (c >= 9 && c <= 13); };
    while (pos < text.size() && space(text[pos]))
        ++pos;
    bool negative = pos < text.size() && text[pos] == '-';
    if (pos < text.size() && (text[pos] == '+' || negative))
        ++pos;
    const auto first = pos;
    const auto limit = signed_value ? std::uint64_t(INT64_MAX) + unsigned(negative) : UINT64_MAX;
    std::uint64_t magnitude = 0;
    bool saturated = false;
    while (pos < text.size() && text[pos] >= '0' && text[pos] <= '9') {
        const auto digit = unsigned(text[pos++] - '0');
        if (magnitude > (limit - digit) / 10) {
            magnitude = limit;
            saturated = true;
        } else if (!saturated)
            magnitude = magnitude * 10 + digit;
    }
    if (first == pos) {
        out["status"] = "invalid";
        conversion["status"] = "no_integer_assignment";
        return out;
    }
    // UCRT saturates its 64-bit intermediate before narrowing the destination.
    // Unsigned overflow saturates to UINT64_MAX without applying a minus sign.
    const auto bits64 = negative && (signed_value || !saturated) ? 0 - magnitude : magnitude;
    const auto bits32 = std::uint32_t(bits64);
    const auto signed32 =
        bits32 <= INT32_MAX ? std::int64_t(bits32) : std::int64_t(bits32) - (std::int64_t(1) << 32);
    out["status"] = "decoded";
    out["value"] = signed_value ? Json(signed32) : Json(bits32);
    conversion.update({{"status", "assigned"},
                       {"consumed_bytes", pos},
                       {"ignored_suffix", text.substr(pos)},
                       {"saturated_64_bit_intermediate", saturated},
                       {"destination_bits", bits32}});
    return out;
}
} // namespace p3d
