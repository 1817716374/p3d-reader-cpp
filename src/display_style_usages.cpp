#include "display_style_xml.hpp"

namespace p3d {
namespace {
std::size_t wide_space(std::string_view text, std::size_t pos) {
    if (pos == text.size()) return 0;
    const auto c = static_cast<unsigned char>(text[pos]);
    if (c == ' ' || (c >= 9 && c <= 13)) return 1;
    // R1.18 P3DDC ordinal472 calls UCRT iswspace. These are the
    // non-ASCII UTF-16 code units accepted in the observed C locale.
    constexpr std::string_view spaces[] = {
        u8"\u0085", u8"\u00a0", u8"\u1680", u8"\u180e", u8"\u2000", u8"\u2001",
        u8"\u2002", u8"\u2003", u8"\u2004", u8"\u2005", u8"\u2006", u8"\u2007",
        u8"\u2008", u8"\u2009", u8"\u200a", u8"\u2028", u8"\u2029", u8"\u202f",
        u8"\u205f", u8"\u3000"};
    for (const auto space : spaces)
        if (text.substr(pos, space.size()) == space) return space.size();
    return 0;
}

struct UnsignedRead {
    bool assigned;
    std::uint32_t value;
    std::size_t consumed;
};
UnsignedRead scan_unsigned(std::string_view token) {
    std::size_t pos = 0;
    while (const auto size = wide_space(token, pos)) pos += size;
    const auto read = material_xml_integer({{"v", std::string(token.substr(pos))}}, "v", false);
    if (read.at("status") != "decoded") return {false, 0, pos};
    return {true, read.at("value").get<std::uint32_t>(),
            pos + read.at("conversion").at("consumed_bytes").get<std::size_t>()};
}
} // namespace

Json decode_display_style_usages(const Json &attrs) {
    Json out = {{"status", "decoded"}, {"scope", "initial_style_usage_bitset"},
                {"read_status", "missing"}, {"native_execution", "not_evaluated"},
                {"index_meanings", "unresolved"}, {"whitespace_profile", "R1.18_UCRT_C"},
                {"tokens", Json::array()}};
    std::string source;
    if (attrs.contains("Usages")) {
        if (!attrs.at("Usages").is_string()) {
            out["status"] = out["read_status"] = "unresolved";
            out["reason"] = "requires_xml_attribute_text";
            return out;
        }
        source = attrs.at("Usages").get<std::string>();
        out["read_status"] = "parsed";
        out["source_text"] = source;
    }
    const auto nul = source.find('\0');
    const std::string_view text(source.data(), nul == std::string::npos ? source.size() : nul);
    std::size_t pos = 0;
    // The first pass skips only space and tab. After each token the native
    // reader trims the whole remaining string with a different helper.
    while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\t')) ++pos;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> intervals;
    while (pos < text.size() && text[pos] >= '0' && text[pos] <= '9') {
        const auto end = text.find_first_of(" ,", pos);
        const auto token = text.substr(pos, end == std::string_view::npos ? text.size() - pos : end - pos);
        const auto first = scan_unsigned(token);
        require(first.assigned, "digit-prefixed usage token must assign its first unsigned value");
        auto last = first;
        if (first.consumed < token.size() && token[first.consumed] == '-') {
            const auto second = scan_unsigned(token.substr(first.consumed + 1));
            if (second.assigned) last = second;
        }
        const auto lo = std::min(first.value, last.value), hi = std::max(first.value, last.value);
        out["tokens"].push_back({{"source_text", std::string(token)}, {"source_byte_offset", pos},
                                 {"first", lo}, {"last", hi}});
        // In the original inclusive uint32 loop, incrementing UINT32_MAX
        // wraps to zero and cannot terminate. Do not publish a final bitset.
        if (hi == UINT32_MAX) {
            out["status"] = "unresolved";
            out["reason"] = "native_uint32_range_loop_wrap";
            break;
        }
        intervals.emplace_back(lo, hi);
        pos = end == std::string_view::npos ? text.size() : end + 1;
        while (const auto size = wide_space(text, pos)) pos += size;
    }
    std::sort(intervals.begin(), intervals.end());
    std::vector<std::pair<std::uint32_t, std::uint32_t>> merged;
    for (const auto interval : intervals) {
        if (!merged.empty() && std::uint64_t(interval.first) <= std::uint64_t(merged.back().second) + 1)
            merged.back().second = std::max(merged.back().second, interval.second);
        else merged.push_back(interval);
    }
    Json ranges = Json::array();
    std::uint64_t count = 0;
    for (const auto &[lo, hi] : merged) {
        ranges.push_back({{"first", lo}, {"last", hi}});
        count += std::uint64_t(hi) - lo + 1;
    }
    out["ignored_suffix"] = source.substr(pos);
    if (out.at("status") == "decoded") {
        out["ranges"] = std::move(ranges);
        out["set_bit_count"] = count;
        out["bit_length"] = merged.empty() ? 0 : std::uint64_t(merged.back().second) + 1;
        out["stop_reason"] = pos == text.size() ? "end_of_input" : "non_digit_prefix";
    } else {
        out["completed_prefix_ranges"] = std::move(ranges);
    }
    return out;
}
} // namespace p3d
