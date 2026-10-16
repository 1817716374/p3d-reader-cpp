#include "tree_entity_refs.hpp"
#include <limits>
#include <stdexcept>

namespace p3d {
namespace {
// The source reader calls Windows UCRT _wtoi64 on UTF-16 text. Its C-locale
// digit/space tables differ from both ASCII and current Unicode categories.
unsigned codepoint(const std::string &text, std::size_t &position) {
    const auto first = static_cast<unsigned char>(text[position++]);
    if (first < 0x80)
        return first;
    const unsigned count = first < 0xe0 ? 1 : first < 0xf0 ? 2 : 3;
    unsigned value = first & (0x7f >> (count + 1));
    for (unsigned i = 0; i < count; ++i)
        value = (value << 6) | (static_cast<unsigned char>(text[position++]) & 0x3f);
    return value;
}
int digit(unsigned value) {
    static const unsigned zero[] = {0x30,  0x660, 0x6f0,  0x966,  0x9e6,  0xa66,
                                    0xae6, 0xb66, 0xc66,  0xce6,  0xd66,  0xe50,
                                    0xed0, 0xf20, 0x1040, 0x17e0, 0x1810, 0xff10};
    for (auto start : zero)
        if (value >= start && value - start < 10)
            return static_cast<int>(value - start);
    return -1;
}
bool space(unsigned value) {
    return (value >= 9 && value <= 13) || value == 0x20 || value == 0x85 || value == 0xa0 ||
           value == 0x1680 || value == 0x180e || (value >= 0x2000 && value <= 0x200a) ||
           value == 0x2028 || value == 0x2029 || value == 0x202f || value == 0x205f ||
           value == 0x3000;
}
struct Integer {
    std::int64_t value = 0;
    bool digits = false, saturated = false;
};
Integer integer(const std::string &text) {
    Integer result;
    std::size_t position = 0;
    while (position < text.size()) {
        auto next = position;
        if (!space(codepoint(text, next)))
            break;
        position = next;
    }
    bool negative = false;
    if (position < text.size() && (text[position] == '+' || text[position] == '-'))
        negative = text[position++] == '-';
    const auto limit = negative ? UINT64_C(0x8000000000000000) : UINT64_C(0x7fffffffffffffff);
    std::uint64_t value = 0;
    while (position < text.size()) {
        const auto next = digit(codepoint(text, position));
        if (next < 0)
            break;
        result.digits = true;
        if (result.saturated)
            continue;
        if (value > (limit - static_cast<unsigned>(next)) / 10) {
            value = limit;
            result.saturated = true;
        } else
            value = value * 10 + static_cast<unsigned>(next);
    }
    result.value = negative ? value == UINT64_C(0x8000000000000000)
                                  ? std::numeric_limits<std::int64_t>::min()
                                  : -static_cast<std::int64_t>(value)
                            : static_cast<std::int64_t>(value);
    return result;
}
} // namespace

Json append_tree_entity_references(const Json &resources, const std::string &node,
                                   const std::set<std::string> &live, const Json &binding_index,
                                   Json &references) {
    Json out = {{"scope", "stored_tree_resource_slot_before_runtime_project_and_set_evaluation"},
                {"query_profile", "native_project_tree_entity_ids"},
                {"status", "not_evaluated"},
                {"source_slot", 3},
                {"numeric_reader", "windows_ucrt_wtoi64"},
                {"numeric_locale", "C"},
                {"reference_indices", Json::array()},
                {"runtime_project_resolution", "not_performed"},
                {"runtime_set_evaluation", "not_performed"}};
    if (!resources.is_array()) {
        out["reason"] = "resource_array_required";
        return out;
    }
    if (resources.size() <= 3) {
        out["status"] = "absent";
        return out;
    }
    if (!resources[3].is_string()) {
        out["reason"] = "resource_slot_text_required";
        return out;
    }
    const auto &text = resources[3].get_ref<const std::string &>();
    const auto nul = text.find('\0');
    const auto end = nul == std::string::npos ? text.size() : nul;
    try {
        Json(text.substr(0, end)).dump(); // Only the C-string prefix is read.
        if (end > std::size_t(std::numeric_limits<std::int32_t>::max()))
            throw std::length_error("resource_utf8_index_budget_exceeded");
    } catch (const std::exception &e) {
        out["reason"] = e.what();
        return out;
    }
    out["input_prefix_byte_count"] = end;
    out["ignored_suffix_byte_count"] = text.size() - end;
    out["status"] = "decoded";
    std::size_t start = 0, token_index = 0;
    while (start < end) {
        const auto comma = text.find(',', start);
        if (comma == std::string::npos || comma >= end)
            break; // The native loop never consumes an unterminated tail.
        const auto token = text.substr(start, comma - start);
        const auto number = integer(token);
        const auto bits = static_cast<std::uint64_t>(number.value);
        const auto model_bits = static_cast<std::uint32_t>(bits >> 32);
        const auto native_model = model_bits <= UINT32_C(0x7fffffff)
                                      ? std::int64_t(model_bits)
                                      : std::int64_t(model_bits) - INT64_C(4294967296);
        Json reference = {{"node", node},
                          {"raw_value", token},
                          {"source_slot", 3},
                          {"source_token_index", token_index++},
                          {"source_byte_offset", start},
                          {"source_byte_count", comma - start},
                          {"basis", "native_tree_slot_3_packed_entity_id"},
                          {"packed_value", number.value},
                          {"conversion_status", number.saturated ? "saturated"
                                                : number.digits  ? "converted"
                                                                 : "no_digits"},
                          {"bindings", Json::array()}};
        if (number.value == -1) {
            // The compiled consumer retains a runtime temporary for the low
            // component when setValue(-1) does not assign. It is not file data.
            reference.update({{"native_model_id", -2},
                              {"model_id", UINT32_C(0xfffffffe)},
                              {"element_id", nullptr},
                              {"native_entity_id_validity", false},
                              {"status", "indeterminate_native_temporary"}});
            out["status"] = "partial";
        } else {
            const auto element = static_cast<std::uint32_t>(bits);
            const auto key = std::to_string(model_bits) + ":" + std::to_string(element);
            reference.update({{"native_model_id", native_model},
                              {"model_id", model_bits},
                              {"element_id", element},
                              {"element_key", key},
                              {"native_entity_id_validity", native_model > -2},
                              {"status", live.count(key) ? "resolved" : "missing"},
                              {"bindings", binding_index.value(key, Json::array())}});
        }
        out["reference_indices"].push_back(references.size());
        references.push_back(std::move(reference));
        start = comma + 1;
    }
    out["ignored_tail_byte_offset"] = start;
    out["ignored_tail_byte_count"] = end - start;
    return out;
}
} // namespace p3d
