#include "internal.hpp"
#include <codecvt>
#include <locale>

namespace p3d {
Json material_resource_reference(const Json &source) {
    Json out = {{"source_value", source},
                {"status", "invalid"},
                {"value", nullptr},
                {"scope", "native_default_resource_service"},
                {"lookup_status", "not_performed"}};
    if (!source.is_string())
        return out;
    std::u16string text;
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> codec;
    try {
        const auto bytes = source.get<std::string>();
        text = codec.from_bytes(bytes);
        if (codec.converted() != bytes.size() || text.find(u'\0') != std::u16string::npos)
            return out;
    } catch (const std::range_error &) {
        return out;
    }
    out["status"] = "decoded";
    out["value"] = source;
    out["normalization"] = "identity";
    const auto open = text.find(u'<');
    const auto close = open == std::u16string::npos ? open : text.find(u'>', open);
    if (close == std::u16string::npos)
        return out;
    const auto token = text.substr(open + 1, close - open - 1);
    Json marker = {{"source_value", codec.to_bytes(token)},
                   {"prefix", codec.to_bytes(text.substr(0, open))},
                   {"suffix", codec.to_bytes(text.substr(close + 1))},
                   {"status", "invalid_integer"},
                   {"value", nullptr}};
    std::int64_t index = -1;
    if (!token.empty()) {
        // The source reader uses scanf("%d"), accepting a decimal prefix.
        // Keep range/locale-dependent results unresolved rather than relying
        // on this platform's scanf overflow or locale behavior.
        std::size_t i = token.find_first_not_of(u" \t\n\r\f\v");
        if (i == std::u16string::npos) {
            out["marker"] = marker;
            return out;
        }
        const bool negative = token[i] == u'-';
        if (negative || token[i] == u'+')
            ++i;
        const auto start = i;
        std::uint64_t value = 0;
        bool overflow = false;
        while (i < token.size() && token[i] >= u'0' && token[i] <= u'9') {
            value = value * 10 + unsigned(token[i++] - u'0');
            if (value > std::uint64_t(INT32_MAX) + unsigned(negative)) {
                overflow = true;
                break;
            }
        }
        if (overflow || (i == start && i < token.size() && token[i] >= 128)) {
            marker["status"] = overflow ? "integer_out_of_range" : "locale_dependent";
            out["marker"] = marker;
            out["status"] = "unresolved";
            out["value"] = nullptr;
            out["normalization"] = nullptr;
            return out;
        }
        if (i == start) {
            out["marker"] = marker;
            return out;
        }
        index = negative ? -std::int64_t(value) : std::int64_t(value);
        marker["unconsumed_text"] = codec.to_bytes(token.substr(i));
    }
    marker["status"] = "decoded";
    marker["value"] = index;
    out["marker"] = marker;
    const auto prefix = text.substr(0, open);
    auto result = prefix;
    // The http: test is case-sensitive and does not include https:.
    if (prefix.compare(0, 5, u"http:") == 0) {
        out["normalization"] = "http_prefix_without_marker_or_suffix";
    } else if (prefix.empty()) {
        out["normalization"] = "empty_prefix_omits_marker_and_suffix";
    } else if (index == 0) {
        out["normalization"] = "zero_marker_omits_marker_and_suffix";
    } else {
        result += index < 0 ? u"<>" : codec.from_bytes("<" + std::to_string(index) + ">");
        result += text.substr(close + 1);
        out["normalization"] = "canonical_integer_marker";
    }
    out["value"] = codec.to_bytes(result);
    return out;
}
} // namespace p3d
