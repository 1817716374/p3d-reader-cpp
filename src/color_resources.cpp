#include "layer_name_key.hpp"
#include <p3d/color_resources.hpp>
#include <pugixml/pugixml.hpp>

namespace p3d {
namespace {
using Hsv = std::array<std::uint32_t, 3>;
using HsvPalette = std::array<Hsv, 256>;
void validate_entities(const std::string &xml) {
    for (std::size_t i = 0; i < xml.size();) {
        const char *end_marker = nullptr;
        std::size_t prefix = 0;
        if (xml.compare(i, 4, "<!--") == 0) {
            end_marker = "-->";
            prefix = 4;
        } else if (xml.compare(i, 9, "<![CDATA[") == 0) {
            end_marker = "]]>";
            prefix = 9;
        } else if (xml.compare(i, 2, "<?") == 0) {
            end_marker = "?>";
            prefix = 2;
        }
        if (end_marker) {
            const auto end = xml.find(end_marker, i + prefix);
            require(end != std::string::npos, "extended_color_xml_unclosed_markup");
            i = end + std::strlen(end_marker);
            continue;
        }
        require(static_cast<unsigned char>(xml[i]) >= 32 || xml[i] == '\t' || xml[i] == '\r' ||
                    xml[i] == '\n',
                "extended_color_xml_control_character");
        if (xml[i++] != '&')
            continue;
        const auto end = xml.find(';', i);
        require(end != std::string::npos, "extended_color_xml_entity_unterminated");
        const auto entity = xml.substr(i, end - i);
        if (entity != "amp" && entity != "lt" && entity != "gt" && entity != "quot" &&
            entity != "apos") {
            require(!entity.empty() && entity[0] == '#', "extended_color_xml_entity_unknown");
            std::size_t p = 1;
            const bool hex = p < entity.size() && entity[p] == 'x';
            if (hex)
                ++p;
            require(p < entity.size(), "extended_color_xml_entity_empty_number");
            std::uint32_t value = 0;
            for (; p < entity.size(); ++p) {
                const auto c = entity[p];
                const unsigned digit = c >= '0' && c <= '9'          ? c - '0'
                                       : hex && c >= 'a' && c <= 'f' ? c - 'a' + 10
                                       : hex && c >= 'A' && c <= 'F' ? c - 'A' + 10
                                                                     : 16;
                const unsigned base = hex ? 16 : 10;
                require(digit < base && value <= (0x10ffffu - digit) / base,
                        "extended_color_xml_entity_number_invalid");
                value = value * base + digit;
            }
            require(
                value == 9 || value == 10 || value == 13 || (value >= 0x20 && value <= 0xd7ff) ||
                    (value >= 0xe000 && value <= 0xfffd) || (value >= 0x10000 && value <= 0x10ffff),
                "extended_color_xml_entity_codepoint_invalid");
        }
        i = end + 1;
    }
}
Json units(const std::u16string &text) {
    return std::vector<std::uint16_t>(text.begin(), text.end());
}
std::uint32_t rgb_key(const NativeRgb &rgb) {
    return (std::uint32_t(rgb[0]) << 16) | (std::uint32_t(rgb[1]) << 8) | rgb[2];
}
HsvPalette prepare(const NativeColorPalette &palette) {
    HsvPalette out;
    for (std::size_t i = 0; i < out.size(); ++i)
        out[i] = native_color_hsv(palette[i]);
    return out;
}
Json nearest(const NativeRgb &rgb, const HsvPalette &palette) {
    const auto query = native_color_hsv(rgb);
    std::uint32_t winner = 0, visited = 0;
    std::optional<std::uint32_t> gray;
    int best = INT32_MAX, darkest_gray = 255;
    for (std::uint32_t i = 0; i < 255; ++i) {
        ++visited;
        const auto &candidate = palette[i];
        int hue = 0;
        if (query[1] > 2) {
            hue = int(candidate[0]) - int(query[0]);
            if (hue > 180)
                hue = 360 - hue;
            else if (hue < -180)
                hue += 360;
        }
        const int saturation = int(candidate[1]) - int(query[1]);
        const int value = int(candidate[2]) - int(query[2]);
        const int distance = 2 * hue * hue + saturation * saturation + value * value;
        if (distance < best) {
            best = distance;
            winner = i;
            if (distance == 0)
                break; // Also bypasses this and later gray-fallback candidates.
        }
        if (candidate[1] == 0 && candidate[2] != 0 && int(candidate[2]) < darkest_gray) {
            darkest_gray = int(candidate[2]);
            gray = i;
        }
    }
    const auto nearest_index = winner;
    const bool nonblack = rgb[0] != 0 || rgb[1] != 0 || rgb[2] != 0;
    const bool fallback = palette[winner][2] == 0 && nonblack && gray.has_value();
    if (fallback)
        winner = *gray;
    return {{"status", "resolved"},
            {"palette_index", winner},
            {"nearest_index_before_fallback", nearest_index},
            {"squared_distance", best},
            {"query_hsv", query},
            {"selected_hsv", palette[winner]},
            {"candidates_visited", visited},
            {"gray_fallback", fallback}};
}
std::optional<std::int32_t> decimal(const std::u16string &text, std::size_t &position) {
    auto space = [](char16_t c) {
        return c == u' ' || c == u'\t' || c == u'\r' || c == u'\n' || c == u'\v' || c == u'\f';
    };
    while (position < text.size() && space(text[position]))
        ++position;
    bool negative = false;
    if (position < text.size() && (text[position] == u'-' || text[position] == u'+'))
        negative = text[position++] == u'-';
    const auto first = position;
    const std::uint64_t limit = negative ? 0x80000000ull : 0x7fffffffull;
    std::uint64_t magnitude = 0;
    while (position < text.size() && text[position] >= u'0' && text[position] <= u'9') {
        const unsigned digit = text[position++] - u'0';
        require(magnitude <= (limit - digit) / 10, "extended_color_decimal_overflow");
        magnitude = magnitude * 10 + digit;
    }
    require(position == text.size() || text[position] < 128,
            "extended_color_decimal_source_locale_required");
    if (position == first)
        return {};
    return std::int32_t(negative ? -std::int64_t(magnitude) : std::int64_t(magnitude));
}
NativeRgb xml_rgb(const std::string &value, unsigned &conversions) {
    NativeRgb rgb{{255, 255, 255}};
    const auto text = detail::layer_name_wide(value);
    conversions = 0;
    if (text.empty() || text[0] != u'(')
        return rgb;
    std::size_t position = 1;
    for (unsigned i = 0; i < 3; ++i) {
        const auto number = decimal(text, position);
        if (!number)
            break;
        rgb[i] = std::uint8_t(std::uint32_t(*number) & 255u);
        ++conversions;
        if (i == 2 || position == text.size() || text[position++] != u',')
            break;
    }
    return rgb;
}
struct ColorEntry {
    NativeRgb rgb;
    std::optional<std::pair<std::u16string, std::u16string>> names = {};
    Json attributes = nullptr, source = nullptr;
};
} // namespace
Hsv native_color_hsv(const NativeRgb &rgb) {
    const double r = rgb[0], g = rgb[1], b = rgb[2];
    const double maximum = std::max(b, std::max(r, g));
    const double minimum = std::min(b, std::min(r, g));
    const double delta = maximum - minimum;
    Hsv hsv{{0, 0, std::uint32_t((maximum / 255.0) * 100.0 + 0.5)}};
    if (maximum == 0)
        return hsv;
    hsv[1] = std::uint32_t((delta / maximum) * 100.0 + 0.5);
    if (!hsv[1])
        return hsv;
    const double red = (maximum - r) / delta;
    const double green = (maximum - g) / delta;
    const double blue = (maximum - b) / delta;
    double hue;
    if (r == maximum)
        hue = blue - green;
    else if (g == maximum)
        hue = red + 2.0 - blue;
    else
        hue = green + 4.0 - red;
    hue *= 60.0;
    if (hue < 0)
        hue += 360.0;
    hsv[0] = std::uint32_t(hue + 0.5);
    if (hsv[0] >= 360)
        hsv[0] = 0;
    return hsv;
}
Json nearest_native_palette_color(const NativeRgb &rgb, const NativeColorPalette &palette) {
    return nearest(rgb, prepare(palette));
}
struct NativeExtendedColorCache::Impl {
    Json data = {{"status", "resolved"},        {"scope", "fresh_extended_rgb_cache"},
                 {"entries", Json::array()},    {"rgb_index", Json::array()},
                 {"load_steps", Json::array()}, {"operations", Json::array()}};
    std::vector<ColorEntry> entries;
    std::map<std::uint32_t, std::uint32_t> index;
    void publish() {
        Json rows = Json::array(), keys = Json::array();
        for (std::size_t i = 0; i < entries.size(); ++i) {
            const auto &e = entries[i];
            rows.push_back({{"extended_color_id", i + 1},
                            {"ordinal", i},
                            {"rgb", e.rgb},
                            {"names_present", e.names.has_value()},
                            {"book_utf16", e.names ? units(e.names->first) : Json()},
                            {"name_utf16", e.names ? units(e.names->second) : Json()},
                            {"source_entry_index", e.source},
                            {"source_attributes", e.attributes}});
        }
        for (const auto &[key, id] : index)
            keys.push_back({{"rgb_key", key}, {"extended_color_id", id}});
        data.update(
            {{"status", "resolved"}, {"entries", std::move(rows)}, {"rgb_index", std::move(keys)}});
    }
    std::uint32_t append(ColorEntry entry) {
        require(entries.size() < UINT32_MAX, "extended_color_cache_capacity_unknown");
        entries.push_back(std::move(entry));
        const auto id = std::uint32_t(entries.size());
        index[rgb_key(entries.back().rgb)] = id;
        return id;
    }
};
NativeExtendedColorCache::NativeExtendedColorCache() : impl_(std::make_shared<Impl>()) {}
NativeExtendedColorCache::NativeExtendedColorCache(std::shared_ptr<const Impl> impl)
    : impl_(std::move(impl)) {}
NativeExtendedColorCache::NativeExtendedColorCache(const std::string &xml) {
    auto state = std::make_shared<Impl>();
    try {
        Json(xml).dump();
        state->data["source_xml"] = xml;
        require(xml.find('\0') == std::string::npos, "extended_color_xml_contains_nul");
        validate_entities(xml);
        pugi::xml_document document;
        const auto parsed =
            document.load_buffer(xml.data(), xml.size(), pugi::parse_full, pugi::encoding_utf8);
        require(bool(parsed), "extended_color_xml_parse_failed");
        unsigned roots = 0;
        for (auto node : document.children()) {
            require(node.type() != pugi::node_doctype, "extended_color_xml_dtd_not_supported");
            if (node.type() == pugi::node_element)
                ++roots;
            else if (node.type() == pugi::node_pcdata || node.type() == pugi::node_cdata)
                require(std::string(node.value()).find_first_not_of(" \t\r\n") == std::string::npos,
                        "extended_color_xml_text_outside_root");
        }
        require(roots == 1, "extended_color_xml_single_root_required");
        std::size_t source = 0;
        std::function<void(pugi::xml_node, unsigned)> read = [&](pugi::xml_node node,
                                                                 unsigned depth) {
            require(depth <= 64, "extended_color_xml_depth_limit");
            require(std::string(node.name()).find(':') == std::string::npos,
                    "extended_color_xml_namespace_unknown");
            Json attributes = Json::object();
            for (auto attribute : node.attributes()) {
                const std::string name = attribute.name();
                require(name != "xmlns" && name.find(':') == std::string::npos,
                        "extended_color_xml_namespace_unknown");
                require(!attributes.contains(name), "extended_color_xml_duplicate_attribute");
                attributes[name] = attribute.value();
            }
            if (std::string(node.name()) == "Entry") {
                Json step = {{"source_entry_index", source++}, {"source_attributes", attributes}};
                const std::string color = node.attribute("Color").value();
                if (color.empty())
                    step["selection"] = "skipped_missing_or_empty_color";
                else {
                    unsigned conversions = 0;
                    ColorEntry entry{xml_rgb(color, conversions)};
                    const auto book = detail::layer_name_wide(node.attribute("Book").value());
                    const auto name = detail::layer_name_wide(node.attribute("Name").value());
                    if (!book.empty() && !name.empty())
                        entry.names = {{book, name}};
                    entry.attributes = attributes;
                    entry.source = source - 1;
                    const auto id = state->append(std::move(entry));
                    step.update({{"selection", "loaded"},
                                 {"extended_color_id", id},
                                 {"decimal_conversions", conversions}});
                }
                state->data["load_steps"].push_back(std::move(step));
            }
            for (auto child : node.children())
                if (child.type() == pugi::node_element)
                    read(child, depth + 1);
        };
        read(document.document_element(), 0);
        state->publish();
    } catch (const std::exception &e) {
        if (!state->data.contains("source_xml"))
            state->data["source_xml_bytes"] = rawbytes(Bytes(xml.begin(), xml.end()));
        state->data.update({{"status", "unresolved"},
                            {"reason", e.what()},
                            {"entries", nullptr},
                            {"rgb_index", nullptr}});
    }
    impl_ = std::move(state);
}
const Json &NativeExtendedColorCache::metadata() const noexcept {
    return impl_->data;
}
Json NativeExtendedColorCache::lookup(const NativeRgb &rgb) const {
    if (impl_->data.at("status") != "resolved")
        return {{"status", "unresolved"}, {"reason", impl_->data.at("reason")}};
    const auto found = impl_->index.find(rgb_key(rgb));
    if (found == impl_->index.end())
        return {{"status", "not_found"}};
    return {{"status", "matched"}, {"extended_color_id", found->second}};
}
NativeExtendedColorCache
NativeExtendedColorCache::resolve(const std::vector<NativeColorRequest> &requests,
                                  const std::optional<NativeColorPalette> &palette,
                                  bool palette_confirmed_absent) const {
    auto state = std::make_shared<Impl>(*impl_);
    state->data["operations"] = Json::array();
    try {
        require(impl_->data.at("status") == "resolved", "loaded_extended_color_cache_required");
        require(!palette || !palette_confirmed_absent, "contradictory_color_palette_context");
        const auto prepared = palette ? std::optional<HsvPalette>(prepare(*palette)) : std::nullopt;
        for (const auto &request : requests) {
            auto book = request.book, name = request.name;
            if (book)
                *book = detail::layer_name_prefix(*book);
            if (name)
                *name = detail::layer_name_prefix(*name);
            const auto found = state->index.find(rgb_key(request.rgb));
            std::uint32_t id;
            bool added = false, attached = false;
            if (found == state->index.end()) {
                ColorEntry entry{request.rgb};
                if (book && name)
                    entry.names = {{*book, *name}};
                id = state->append(std::move(entry));
                added = true;
            } else {
                id = found->second;
                auto &entry = state->entries.at(id - 1);
                if (!entry.names && book && name && !book->empty() && !name->empty()) {
                    entry.names = {{*book, *name}};
                    attached = true;
                }
            }
            Json operation = {{"rgb", request.rgb},      {"extended_color_id", id},
                              {"added", added},          {"names_attached", attached},
                              {"packed_color", nullptr}, {"assign_to_layer", nullptr}};
            if (prepared || palette_confirmed_absent) {
                Json match = prepared ? nearest(request.rgb, *prepared)
                                      : Json{{"status", "absent"}, {"palette_index", 0}};
                const auto color = (id << 8) | match.at("palette_index").get<std::uint32_t>();
                operation.update({{"palette_lookup", std::move(match)},
                                  {"packed_color", color},
                                  {"assign_to_layer", color != 0xffffff00u},
                                  {"color_status", "resolved"}});
            } else {
                operation.update(
                    {{"palette_lookup",
                      {{"status", "unresolved"}, {"reason", "actual_owner_palette_required"}}},
                     {"color_status", "unresolved"}});
            }
            state->data["operations"].push_back(std::move(operation));
        }
        state->publish();
        state->data["color_resolution_status"] =
            prepared || palette_confirmed_absent || requests.empty() ? "evaluated"
                                                                     : "partially_evaluated";
    } catch (const std::exception &e) {
        state->data.update({{"status", "unresolved"},
                            {"reason", e.what()},
                            {"entries", nullptr},
                            {"rgb_index", nullptr}});
    }
    return NativeExtendedColorCache(std::move(state));
}
} // namespace p3d
