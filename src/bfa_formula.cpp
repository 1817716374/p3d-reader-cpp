#include "blob_internal.hpp"
#include <regex>
namespace p3d {
Json decode_bfa_formula_expression(const Bytes &encoded) {
    Json out = {{"status", "not_converted"}, {"evaluation_status", "not_performed"}};
    if (encoded.size() > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
        out["status"] = "native_length_out_of_range";
        return out;
    }
    std::string expanded;
    std::size_t position = 0;
    try {
        while (position < encoded.size()) {
            const auto c = encoded[position++];
            expanded.push_back(static_cast<char>(c));
            if (c != '>')
                continue;
            // Native peeks append a byte before checking it. On a mismatch
            // it is read again on the next iteration; preserve that behavior.
            require(position < encoded.size(), "native formula peek exceeds encoded field");
            expanded.push_back(static_cast<char>(encoded[position]));
            if (encoded[position] != '<')
                continue;
            ++position;
            require(position < encoded.size(), "native formula peek exceeds encoded field");
            expanded.push_back(static_cast<char>(encoded[position]));
            if (encoded[position] != '@')
                continue;
            require(encoded.size() - position >= 9,
                    "native formula reference exceeds encoded field");
            std::uint64_t id = 0;
            for (unsigned i = 0; i < 8; ++i)
                id |= std::uint64_t(encoded[position + 1 + i]) << (8 * i);
            expanded += std::to_string(id);
            position += 9;
        }
    } catch (const std::exception &e) {
        out["status"] = "native_read_outside_formula";
        out["encoded_offset"] = position;
        out["message"] = e.what();
        return out;
    }
    out["expanded_bytes"] = rawbytes(Bytes(expanded.begin(), expanded.end()));
    std::string result = expanded;
    Json conversions = Json::array();
    try {
        // These are exact byte signatures from the native converter, not a
        // chosen code page for arbitrary user formula text.
        const std::string names[] = {std::string("\xcd\xbc\xd4\xaa", 4),
                                     std::string("\xd7\xd3\xd7\xe9\xbc\xfe", 6)};
        for (unsigned family = 0; family < 2; ++family) {
            const auto &name = names[family];
            const std::regex matcher("\\{" + name + ".+?\\#.+?\\}");
            const std::regex strip("\\{" + name + "><@|\\}");
            // Both native passes enumerate the original expanded input while
            // applying global replacements to the evolving output string.
            for (std::sregex_iterator i(expanded.begin(), expanded.end(), matcher), end; i != end;
                 ++i) {
                const auto matched = i->str();
                const auto inner = std::regex_replace(matched, strip, std::string());
                const auto replacement = "{" + name + inner + "}";
                std::string pattern;
                for (const char c : matched) {
                    if (c == '{' || c == '}')
                        pattern += static_cast<char>(92);
                    pattern += c;
                }
                Json step = {
                    {"kind", family == 0 ? "primitive" : "subcomponent"},
                    {"expanded_offset", i->position()},
                    {"matched_bytes", rawbytes(Bytes(matched.begin(), matched.end()))},
                    {"replacement_bytes", rawbytes(Bytes(replacement.begin(), replacement.end()))}};
                // Native only escapes braces in the source and uses default
                // regex replacement formatting, including '$' substitutions.
                result = std::regex_replace(result, std::regex(pattern), replacement);
                conversions.push_back(std::move(step));
            }
        }
        out["status"] = "converted";
        out["expression_bytes"] = rawbytes(Bytes(result.begin(), result.end()));
    } catch (const std::regex_error &e) {
        out["status"] = "regex_error";
        out["message"] = e.what();
    }
    out["conversions"] = std::move(conversions);
    return out;
}
} // namespace p3d
