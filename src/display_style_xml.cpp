#include "display_style_xml.hpp"
#include <limits>

namespace p3d {
namespace {
struct Scalar {
    std::uint64_t value = 0;
    const char *status = "missing";
};

Scalar read_scalar(const Json &attributes, const char *name, bool boolean) {
    if (!attributes.contains(name))
        return {};
    if (!attributes.at(name).is_string())
        return {0, "invalid"};
    const auto &text = attributes.at(name).get_ref<const std::string &>();
    std::string_view s(text.data(), text.find('\0') == std::string::npos
                                       ? text.size() : text.find('\0'));
    if (boolean) {
        auto equal = [&](std::string_view word) {
            if (s.size() != word.size()) return false;
            for (std::size_t i = 0; i < s.size(); ++i) {
                const char c = s[i] >= 'A' && s[i] <= 'Z' ? char(s[i] + ('a' - 'A')) : s[i];
                if (c != word[i]) return false;
            }
            return true;
        };
        if (equal("true")) return {1, "parsed"};
        if (equal("false")) return {0, "parsed"};
        return {0, "invalid"};
    }
    // R1.18 p3dlibxml2 uses the Windows CRT's decimal unsigned scan.
    // Numeric prefixes are accepted, magnitudes saturate at UINT64_MAX,
    // and non-overflowing negative magnitudes are negated modulo 2^64.
    // Explicit arithmetic avoids scanf overflow and host-locale differences.
    std::size_t pos = 0;
    while (pos < s.size() && (s[pos] == ' ' || (s[pos] >= '\t' && s[pos] <= '\r')))
        ++pos;
    bool negative = false;
    if (pos < s.size() && (s[pos] == '+' || s[pos] == '-'))
        negative = s[pos++] == '-';
    const auto start = pos;
    constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
    std::uint64_t value = 0;
    bool overflow = false;
    while (pos < s.size() && s[pos] >= '0' && s[pos] <= '9') {
        const auto digit = unsigned(s[pos++] - '0');
        if (value > (maximum - digit) / 10) {
            overflow = true;
            value = maximum;
        } else if (!overflow) {
            value = value * 10 + digit;
        }
    }
    if (pos == start) return {0, "invalid"};
    if (negative && !overflow) value = std::uint64_t(0) - value;
    return {value, "parsed"};
}

Json scalar_field(const Json &attrs, const char *name, bool boolean,
                  std::uint64_t fallback = 0, bool wide = false) {
    const auto scalar = read_scalar(attrs, name, boolean);
    const bool parsed = std::string_view(scalar.status) == "parsed";
    const auto value = parsed ? scalar.value : fallback;
    return {{"value", boolean ? Json(value != 0) : wide ? Json(value) : Json(std::uint32_t(value))},
            {"read_status", scalar.status}, {"default_applied", !parsed}};
}

Json floating_field(const Json &attrs, const char *name, double fallback = 0) {
    // Both XML readers use the same UCRT floating scanner. Keep its exact
    // nonfinite representation and failed-assignment distinction, then apply
    // this style reader's per-field fallback (not the material caller's).
    auto read = material_xml_float(attrs, name);
    const auto status = read.at("status").get<std::string>();
    const bool parsed = status == "decoded";
    const bool failed = material_xml_numeric_failed(read);
    Json out = {{"value", parsed ? read.at("value") : failed ? Json(fallback) : Json()},
                {"read_status", parsed ? "parsed" : status}, {"default_applied", failed},
                {"conversion", read.at("conversion")}};
    if (parsed) {
        out["ieee754_hex"] = read.at("conversion").at("destination_bits_hex");
    } else if (failed) {
        std::uint64_t bits;
        std::memcpy(&bits, &fallback, sizeof bits);
        std::string text(16, '0');
        for (std::size_t i = text.size(); i; bits >>= 4)
            text[--i] = "0123456789abcdef"[bits & 15];
        out["ieee754_hex"] = std::move(text);
    }
    return out;
}

Json string_field(const Json &attrs, const char *name) {
    if (!attrs.contains(name))
        return {{"value", ""}, {"read_status", "missing"}, {"default_applied", true}};
    if (!attrs.at(name).is_string())
        return {{"value", nullptr}, {"read_status", "unresolved"}, {"default_applied", false}};
    const auto &text = attrs.at(name).get_ref<const std::string &>();
    return {{"value", text.substr(0, text.find('\0'))}, {"read_status", "parsed"}, {"default_applied", false}};
}

struct Flag {
    const char *name;
    unsigned offset;
    unsigned bit;
    bool fallback;
};
// P3DKJ.dll 0xd9371..0xd980a. Four flags default to true on either
// missing OR invalid XML; numeric strings such as "1" are invalid booleans.
constexpr Flag flags[] = {
    {"DisplayVisibleEdges", 0x48, 6, false}, {"DisplayHiddenEdges", 0x48, 7, false},
    {"DisplayShadows", 0x48, 11, false}, {"BackgroundColor", 0x48, 13, false},
    {"LegacyDrawOrder", 0x48, 12, false}, {"ApplyEdgeStyleToLines", 0x48, 14, false},
    {"IgnoreGeometryMaps", 0x48, 15, false}, {"IgnoreImageMaps", 0x48, 16, false},
    {"HideInPickers", 0x48, 17, false}, {"InvisibleToCamera", 0x48, 18, false},
    {"DisplayGroundPlane", 0x48, 19, false}, {"VisibleEdgeColor", 0x50, 0, false},
    {"VisibleEdgeStyle", 0x50, 7, true}, {"VisibleEdgeWeight", 0x50, 1, true},
    {"HiddenEdgeLineStyle", 0x50, 8, true}, {"HiddenEdgeWeight", 0x50, 9, false},
    {"Transparency", 0x50, 2, false}, {"FillColor", 0x50, 3, false},
    {"LineStyle", 0x50, 4, false}, {"LineWeight", 0x50, 5, false},
    {"Material", 0x50, 6, false}, {"HLineTransparency", 0x50, 10, true},
    {"HLineMaterialColors", 0x50, 11, false}, {"SmoothIgnoreLights", 0x50, 12, false},
    {"UseDisplayHandler", 0x50, 13, false}
};
} // namespace

Json decode_display_style_xml(const Json &tree) {
    Json out = {{"status", "unresolved"}, {"scope", "native_xml_field_writes"},
                {"runtime_resolution", "not_evaluated"}};
    try {
        const auto tag = tree.at("tag").get<std::string>();
        require(tag == "ShowStyle" || tag == "DisplayStyle", "unsupported style XML root");
        const auto &root = tree.at("attributes");
        require(root.is_object(), "style XML attributes must be an object");
        if (!root.contains("Name") || !root.at("Name").is_string()) {
            out["status"] = "rejected";
            out["reason"] = "missing_or_invalid_Name";
            return out;
        }
        const Json *flag_attrs = nullptr, *overrides = nullptr;
        for (const auto &child : tree.at("children")) {
            const auto name = child.at("tag").get<std::string>();
            if (name == "Flags") {
                require(!flag_attrs, "ambiguous Flags nodes");
                flag_attrs = &child.at("attributes");
            } else if (name == "Overrides") {
                require(!overrides, "ambiguous Overrides nodes");
                overrides = &child.at("attributes");
            }
        }
        if (!flag_attrs || !overrides) {
            out["status"] = "rejected";
            out["reason"] = "missing_Flags_or_Overrides";
            return out;
        }
        require(flag_attrs->is_object() && overrides->is_object(), "invalid style XML sections");
        auto mode = scalar_field(*overrides, "DisplayMode", false);
        if (mode.at("read_status") != "parsed") {
            out["status"] = "rejected";
            out["reason"] = "missing_or_invalid_DisplayMode";
            return out;
        }
        Json fields = Json::object();
        std::uint32_t word48 = mode.at("value").get<std::uint32_t>() & 0x3fu, word50 = 0;
        mode["stored_value"] = word48;
        fields["Overrides.DisplayMode"] = std::move(mode);
        auto hidden = scalar_field(*overrides, "HiddenEdgeLineStyle", false);
        const auto hidden_bits = hidden.at("value").get<std::uint32_t>() & 7u;
        hidden["stored_value"] = hidden_bits;
        word48 |= hidden_bits << 8;
        fields["Overrides.HiddenEdgeLineStyle"] = std::move(hidden);
        for (const auto &flag : flags) {
            auto field = scalar_field(*flag_attrs, flag.name, true, flag.fallback);
            if (field.at("value").get<bool>())
                (flag.offset == 0x48 ? word48 : word50) |= std::uint32_t(1) << flag.bit;
            field["object_offset"] = flag.offset;
            field["bit"] = flag.bit;
            fields[std::string("Flags.") + flag.name] = std::move(field);
        }
        for (const auto *name : {"VisibleEdgeColor", "VisibleEdgeWeight", "BackgroundColor",
                                 "FillColor", "LineStyle", "LineWeight", "DisplayHandler"})
            fields[std::string("Overrides.") + name] = scalar_field(*overrides, name, false);
        fields["Overrides.HiddenEdgeWeight"] = scalar_field(*overrides, "HiddenEdgeWeight", false, 65535);
        fields["Overrides.Material"] = scalar_field(*overrides, "Material", false, 0, true);
        fields["EnvironmentTypeDisplayed"] = scalar_field(root, "EnvironmentTypeDisplayed", false);
        fields["ShowGroundFromBelow"] = scalar_field(root, "ShowGroundFromBelow", true);
        // 0xd9276..0xd92bc: getters clear their destination even when absent,
        // so the constructor's ground color/height defaults do not survive.
        for (const auto *name : {"GroundPlaneColor.R", "GroundPlaneColor.G", "GroundPlaneColor.B",
                                 "GroundPlaneHeight", "GroundPlaneTransparency"})
            fields[name] = floating_field(root, name);
        fields["Overrides.Transparency"] = floating_field(*overrides, "Transparency");
        // 0xd99fe: unlike the other doubles, failed threshold reads use 0.3.
        fields["Overrides.HLineTransparencyThreshold"] =
            floating_field(*overrides, "HLineTransparencyThreshold", 0.3);
        fields["Name"] = string_field(root, "Name");
        fields["EnvironmentName"] = string_field(root, "EnvironmentName");
        out["usages"] = decode_display_style_usages(root);
        out["status"] = "partial";
        out["fields"] = std::move(fields);
        out["packed_flags_at_48"] = word48;
        out["packed_flags_at_50"] = word50;
        out["unmodeled_steps"] = Json::array({"display_handler_registry", "table_selection_and_resource_binding"});
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
