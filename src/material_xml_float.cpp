#include "internal.hpp"
#include <charconv>

namespace p3d {
namespace {
char lower(char c) {
    return c >= 'A' && c <= 'Z' ? char(c + ('a' - 'A')) : c;
}
bool digit(char c, bool hex) {
    return (c >= '0' && c <= '9') || (hex && lower(c) >= 'a' && lower(c) <= 'f');
}
bool payload_char(char c) {
    return digit(c, false) || (lower(c) >= 'a' && lower(c) <= 'z') || c == '_';
}
std::string bits_text(std::uint64_t bits) {
    std::string text(16, '0');
    for (std::size_t i = text.size(); i; bits >>= 4)
        text[--i] = "0123456789abcdef"[bits & 15];
    return text;
}
} // namespace

Json material_xml_float(const Json &attributes, const std::string &key) {
    static_assert(sizeof(double) == sizeof(std::uint64_t) && std::numeric_limits<double>::is_iec559,
                  "material XML requires IEEE 754 binary64");
    Json out = {{"source_keys", Json::array({key})},
                {"status", "missing"},
                {"value", nullptr},
                {"conversion",
                 {{"reader", "ucrt_floating_point"},
                  {"format", "%lg"},
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
    out["status"] = "invalid";
    conversion["status"] = "no_floating_assignment";
    const auto nul = text.find('\0');
    const auto end = nul == std::string::npos ? text.size() : nul;
    std::size_t pos = 0;
    auto at = [&](std::size_t i) { return i < end ? text[i] : '\0'; };
    while (at(pos) == ' ' || (at(pos) >= 9 && at(pos) <= 13))
        ++pos;
    const bool negative = at(pos) == '-';
    if (negative || at(pos) == '+')
        ++pos;
    auto word = [&](const char *s) {
        for (; *s; ++s, ++pos)
            if (lower(at(pos)) != *s)
                return false;
        return true;
    };
    std::uint64_t bits = 0;
    bool nonfinite = false;
    const char *range = "representable";
    if (lower(at(pos)) == 'i') {
        if (!word("inf"))
            return out;
        if (lower(at(pos)) == 'i' && !word("inity"))
            return out;
        bits = UINT64_C(0x7ff0000000000000);
        nonfinite = true;
        range = "explicit_nonfinite";
    } else if (lower(at(pos)) == 'n') {
        if (!word("nan"))
            return out;
        bits = UINT64_C(0x7fffffffffffffff);
        if (at(pos) == '(') {
            const auto first = ++pos;
            while (payload_char(at(pos)))
                ++pos;
            if (at(pos) != ')')
                return out;
            auto payload = text.substr(first, pos - first);
            for (auto &c : payload)
                c = lower(c);
            if (payload == "snan")
                bits = UINT64_C(0x7ff0000000000001);
            if (payload == "ind")
                bits = UINT64_C(0xfff8000000000000);
            ++pos;
        }
        nonfinite = true;
        range = "explicit_nonfinite";
    } else {
        const bool hex = at(pos) == '0' && lower(at(pos + 1)) == 'x';
        if (hex)
            pos += 2;
        const auto first = pos;
        std::size_t digits = 0, before_dot = 0, first_nonzero = std::string::npos;
        auto mantissa = [&] {
            while (digit(at(pos), hex)) {
                if (at(pos) != '0' && first_nonzero == std::string::npos)
                    first_nonzero = digits;
                ++digits;
                ++pos;
            }
        };
        mantissa();
        before_dot = digits;
        if (at(pos) == '.') {
            ++pos;
            mantissa();
        }
        if (!digits)
            return out;
        std::int64_t exponent = 0;
        if (lower(at(pos)) == (hex ? 'p' : 'e')) {
            ++pos;
            const bool minus = at(pos) == '-';
            if (minus || at(pos) == '+')
                ++pos;
            const auto exponent_first = pos;
            while (digit(at(pos), false)) {
                exponent = std::min<std::int64_t>(1000000000000LL, exponent * 10 + (at(pos) - '0'));
                ++pos;
            }
            // scanf commits to an exponent marker; it does not retreat to the
            // mantissa when exponent digits are missing, unlike strtod.
            if (pos == exponent_first)
                return out;
            if (minus)
                exponent = -exponent;
        }
        double value = 0;
        if (first_nonzero != std::string::npos) {
            const auto result =
                std::from_chars(text.data() + first, text.data() + pos, value,
                                hex ? std::chars_format::hex : std::chars_format::general);
            if (result.ec == std::errc::result_out_of_range) {
                const auto bounded = [](std::size_t n) {
                    return std::int64_t(std::min<std::size_t>(n, 1000000000));
                };
                const auto position = before_dot >= first_nonzero
                                          ? bounded(before_dot - first_nonzero)
                                          : -bounded(first_nonzero - before_dot);
                const auto order = (position - 1) * (hex ? 4 : 1) + exponent;
                if (order < 0) {
                    value = 0;
                    range = "underflow_to_zero";
                } else {
                    value = std::numeric_limits<double>::infinity();
                    range = "overflow_to_infinity";
                    nonfinite = true;
                }
            } else if (result.ec != std::errc() || result.ptr != text.data() + pos) {
                out["status"] = "unresolved";
                conversion["status"] = "unresolved_binary_conversion";
                return out;
            }
        }
        std::memcpy(&bits, &value, sizeof bits);
        conversion["radix"] = hex ? 16 : 10;
    }
    if (negative)
        bits |= UINT64_C(0x8000000000000000);
    const auto encoded = bits_text(bits);
    if (nonfinite) {
        const bool infinity = (bits & UINT64_C(0x000fffffffffffff)) == 0;
        out["value"] = {{"floating_point", infinity ? "infinity" : "nan"},
                        {"negative", bool(bits >> 63)},
                        {"ieee754_hex", encoded}};
    } else {
        double value;
        std::memcpy(&value, &bits, sizeof value);
        out["value"] = value;
    }
    out["status"] = "decoded";
    conversion.update({{"status", "assigned"},
                       {"consumed_bytes", pos},
                       {"ignored_suffix", text.substr(pos)},
                       {"destination_bits_hex", encoded},
                       {"range", range}});
    return out;
}

bool material_xml_numeric_failed(const Json &read) {
    if (read.at("status") == "missing")
        return true;
    const auto conversion = read.value("conversion", Json::object()).value("status", Json());
    if (conversion == "no_floating_assignment" || conversion == "no_integer_assignment")
        return true;
    if (read.contains("components"))
        for (const auto &c : read.at("components"))
            if (material_xml_numeric_failed(c))
                return true;
    return false;
}
} // namespace p3d
