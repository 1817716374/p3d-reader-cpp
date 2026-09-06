#include "internal.hpp"
#include <charconv>

namespace p3d {
namespace {
Json material_part_number(const std::string &key) {
    std::int32_t value = 0;
    auto parsed = std::from_chars(key.data(), key.data() + key.size(), value);
    // Native lookup formats a signed int with %d; aliases such as 01 or -0 are distinct keys.
    if (parsed.ec != std::errc() || parsed.ptr != key.data() + key.size() ||
        std::to_string(value) != key)
        return nullptr;
    return value;
}
void material_index_entries(Json &out, unsigned index) {
    if (index != 1 && index != 2)
        return;
    const auto &value = out.at("value");
    out["entries"] = Json::array();
    out["index_shape_status"] = value.is_object() ? "recognized" : "unexpected_shape";
    if (!value.is_object())
        return;
    auto add = [&](const std::string &key, const Json &source, const Json &name, bool reverse) {
        auto number = material_part_number(key);
        const bool recognized =
            !number.is_null() && source.is_string() && (!reverse || source == key);
        if (!recognized)
            out["index_shape_status"] = "unexpected_entries";
        Json entry = {{"source_key", key},
                      {"part_index", number},
                      {"source_value", source},
                      {"material_name", name},
                      {"status", recognized ? "recognized" : "unexpected_entry"}};
        if (reverse)
            entry["repeated_index_matches_key"] = source.is_string() && source == key;
        out["entries"].push_back(std::move(entry));
    };
    if (index == 1) {
        for (auto i = value.begin(); i != value.end(); ++i)
            add(i.key(), i.value(), i.value().is_string() ? i.value() : Json(), false);
    } else {
        out["unrecognized_material_groups"] = Json::array();
        for (auto i = value.begin(); i != value.end(); ++i) {
            if (!i.value().is_object()) {
                out["index_shape_status"] = "unexpected_entries";
                out["unrecognized_material_groups"].push_back(
                    {{"material_name", i.key()}, {"source_value", i.value()}});
                continue;
            }
            for (auto j = i.value().begin(); j != i.value().end(); ++j)
                add(j.key(), j.value(), i.key(), true);
        }
    }
    out["part_index_basis"] = "zero_based_graphics_entry_order";
    out["geometry_mapping_status"] = "not_established";
}
} // namespace
Json native_display_state(unsigned type, const Bytes &b) {
    // These supported graphic records contain the common display header.
    // Other record classes may store unrelated data at the same byte position.
    static const std::set<unsigned> types = {19, 20, 21, 37, 40, 44, 52, 54, 62, 97};
    if (!types.count(type) || b.size() < 38)
        return {{"status", "unsupported_header"}, {"view_visibility", "not_evaluated"}};
    auto word = Reader(b, 36).u16();
    return {{"status", "decoded"},
            {"source_offset", 36},
            {"display_word", word},
            {"permanently_invisible", bool(word & 0x400)},
            {"unassigned_bits", word & ~0x400u},
            {"view_visibility", "not_evaluated"}};
}
Json decode_material_assignment(unsigned index, const Bytes &b) {
    if (index == 0) {
        // The native setter writes exactly 2 * character_count bytes, without a terminator.
        auto name = utf16(b);
        Json(name).dump(); // Reject malformed Unicode while preserving the source attribute.
        return {{"encoding", "advanced_material_name"}, {"material_name", name}};
    }
    Json out = {{"encoding", "advanced_material_json"},
                {"role", index == 1   ? "part_material_names"
                         : index == 2 ? "material_part_index"
                                      : "unassigned"},
                {"source_bytes", rawbytes(b)}};
    auto text = utf8(b);
    out["text"] = text;
    try {
        out["value"] = Json::parse(text);
        out["json_status"] = "parsed";
    } catch (const std::exception &) {
        out["json_status"] = "invalid_json";
    }
    if (out["json_status"] == "parsed")
        material_index_entries(out, index);
    return out;
}
Json decode_layer_group_attribute(unsigned group, const Bytes &b) {
    Reader r(b);
    if (group == 1) {
        auto mode = r.u32();
        r.finish();
        return {{"encoding", "layer_group_sync_state"},
                {"sync_state", mode},
                {"sync_state_name", mode == 0   ? Json("sync_use_override")
                                    : mode == 1 ? Json("always_unsync")
                                    : mode == 2 ? Json("always_sync")
                                                : Json()}};
    }
    auto count = r.u32();
    require(count <= r.left() / 8, "layer override entry count");
    Json entries = Json::array();
    const std::map<unsigned, const char *> properties = {
        {11, "color"}, {12, "line_style"}, {14, "line_weight"}, {25, "display"},
        {26, "print"}, {32, "frozen"},     {35, "transparency"}};
    for (std::uint32_t i = 0; i < count; ++i) {
        auto offset = r.p;
        auto id = r.u32(), bits = r.u32();
        // This native serializer writes bit_count WORDs, not ceil(bit_count / 16).
        // Only the leading packed words are consumed by the native bitmap reader.
        require(bits <= r.left() / 2, "layer override bitmap length");
        auto storage = r.take(std::size_t(bits) * 2);
        Json words = Json::array(), set = Json::array(), unassigned = Json::array();
        Reader packed(storage);
        auto word_count = (std::uint64_t(bits) + 15) / 16;
        for (std::uint64_t w = 0; w < word_count; ++w) {
            auto word = packed.u16();
            words.push_back(word);
            for (unsigned k = 0; k < 16 && w * 16 + k < bits; ++k)
                if (word & (1u << k)) {
                    auto bit = unsigned(w * 16 + k);
                    set.push_back(bit);
                    if (!properties.count(bit))
                        unassigned.push_back(bit);
                }
        }
        Json overrides = Json::object();
        for (const auto &property : properties) {
            auto bit = property.first;
            overrides[property.second] =
                bit < bits && bool(words[bit / 16].get<unsigned>() & (1u << (bit % 16)));
        }
        entries.push_back({{"source_offset", offset},
                           {"layer_id", id},
                           {"bit_count", bits},
                           {"storage_words", bits},
                           {"packed_words", words},
                           {"source_storage", rawbytes(storage)},
                           {"set_property_bits", set},
                           {"overrides", overrides},
                           {"unassigned_property_bits", unassigned}});
    }
    r.finish();
    return {{"encoding", "layer_group_overrides"},
            {"entry_count", count},
            {"entries", entries},
            {"view_visibility", "not_evaluated"}};
}
Json material_assignment_records(const Json &graphics) {
    Json out = Json::array();
    for (const auto &g : graphics)
        for (std::size_t i = 0; i < g.at("attributes").size(); ++i) {
            const auto &a = g["attributes"][i];
            if ((a["group"] != 4 && a["group"] != 2) || a["key"] != 10001)
                continue;
            out.push_back({{"stream", g["stream"]},
                           {"record_id", g["id"]},
                           {"record_offset", g["offset"]},
                           {"attribute_ordinal", i},
                           {"attribute_index", a["index"]},
                           {"attribute_offset", a["offset"]},
                           {"group", a["group"]},
                           {"key", a["key"]},
                           {"payload", a["payload"]},
                           {"decoded", a["decoded"]}});
        }
    return out;
}
Json decode_display_attribute(unsigned group, unsigned key, const Bytes &b) {
    if (key == 109 && group == 0) {
        Reader r(b);
        auto flags = r.u32(), unassigned = r.u32();
        Json out = {{"encoding", "section_clip_data"},
                    {"flags", flags},
                    {"unassigned_word", unassigned},
                    {"unassigned_flag_bits", flags & ~0x7fu}};
        Json crop = Json::object();
        const char *names[] = {"left", "right", "front", "back", "top", "bottom"};
        for (unsigned i = 0; i < 6; ++i)
            crop[names[i]] = bool(flags & (1u << i));
        out["crop"] = std::move(crop);
        out["perspective_up"] = bool(flags & 0x40);
        for (const auto *name : {"top_height", "bottom_height", "front_depth", "back_depth"})
            out[name] = r.f64();
        Json serialized = Json::array(), rotation = Json::array();
        // The native reader postmultiplies the serialized basis by Rx(+pi/2).
        const auto angle = std::acos(-1.) * .5, c = std::cos(angle), s = std::sin(angle);
        for (unsigned i = 0; i < 3; ++i) {
            auto row = r.doubles(3);
            double y = row[1], z = row[2];
            serialized.push_back(row);
            rotation.push_back({row[0], y * c + z * s, -y * s + z * c});
        }
        r.finish();
        out["serialized_rotation"] = std::move(serialized);
        out["rotation"] = std::move(rotation);
        out["rotation_conversion"] = "serialized_times_rotation_x_pi_over_2";
        return out;
    }
    if (key == 109 && group == 1) {
        Reader r(b);
        Json out = {{"encoding", "section_clip_frame"},
                    {"origin", r.doubles(3)},
                    {"direction", r.doubles(3)}};
        r.finish();
        return out;
    }
    if (group == 0 && key == 22634) {
        Reader r(b);
        auto id = r.u32();
        r.finish();
        return {{"encoding", "display_style_reference"}, {"display_style_entry_id", id}};
    }
    if (group == 0 && key == 22626) {
        Reader r(b);
        Json out = {{"encoding", "clip_volume_settings"}, {"header_word", r.u32()}};
        Json regions = Json::object();
        for (const auto *name : {"forward", "back", "cut", "outside"}) {
            auto flags = r.u32(), style = r.u32();
            regions[name] = {{"flags", flags},
                             {"display", bool(flags & 1)},
                             {"snap", !(flags & 2)},
                             {"locate", !(flags & 4)},
                             {"unassigned_flag_bits", flags & ~7u},
                             {"display_style_entry_id", style}};
        }
        r.finish();
        out["regions"] = std::move(regions);
        return out;
    }
    if (group == 1 && key == 22295) {
        Reader r(b);
        auto header = r.u32();
        require(header == 0, "unsupported auxiliary coordinate system header");
        auto type = r.u32();
        auto independent = r.i32();
        Json out = {{"encoding", "auxiliary_coordinate_system"},
                    {"header_word", header},
                    {"coordinate_system_type", type},
                    {"view_independent_value", independent},
                    {"view_independent", independent > 0},
                    {"origin", r.doubles(3)},
                    {"scale", r.f64()}};
        Json rotation = Json::array();
        for (unsigned i = 0; i < 3; ++i)
            rotation.push_back(r.doubles(3));
        out["rotation"] = std::move(rotation);
        out["element_id"] = r.u64();
        out["extra_data_offset"] = r.p;
        out["extra_data_status"] = r.left() ? "opaque" : "absent";
        out["extra_data"] = rawbytes(r.take(r.left()));
        // An auxiliary drawing frame is metadata, not an element placement or a CRS.
        return out;
    }
    return nullptr;
}
} // namespace p3d
