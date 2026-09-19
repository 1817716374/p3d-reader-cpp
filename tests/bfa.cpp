#include "blob_internal.hpp"
#include <future>
namespace {
using namespace p3d;
template <class T> void put(Bytes &b, T v) {
    auto p = reinterpret_cast<const std::uint8_t *>(&v);
    b.insert(b.end(), p, p + sizeof(v));
}
void ref(Bytes &b, std::uint64_t id) {
    b.insert(b.end(), {0x7c, 0x23, 0x40});
    put(b, id);
}
Bytes type_body(std::uint64_t id, const Bytes &name, const std::vector<std::uint64_t> &placed) {
    Bytes b;
    ref(b, id);
    put<std::uint64_t>(b, 0);
    put<std::uint32_t>(b, name.size());
    b.insert(b.end(), name.begin(), name.end());
    for (auto x : placed)
        ref(b, x);
    return b;
}
void record(Bytes &b, const std::string &tag, std::uint64_t id, const Bytes &body,
            const std::vector<std::uint64_t> &children = {}) {
    b.insert(b.end(), tag.begin(), tag.end());
    ref(b, id);
    put<std::uint64_t>(b, body.size());
    b.insert(b.end(), body.begin(), body.end());
    put<std::uint32_t>(b, children.size());
    for (auto x : children)
        ref(b, x);
}
Bytes drive_body(unsigned kind, const Bytes &formula, bool type_field = true) {
    Bytes b;
    ref(b, 23);
    put<std::uint64_t>(b, 7);
    put<std::uint32_t>(b, kind);
    ref(b, 0xfedcba9876543210ULL);
    if (kind == 3)
        put<std::int64_t>(b, -17);
    ref(b, 0x5d5b23405b5d237cULL); // Delimiter bytes inside an ID are not separators.
    ref(b, 0x5d5b23405b5d237cULL);
    b.push_back('[');
    ref(b, 31);
    b.push_back(']');
    ref(b, 9);
    put<std::int64_t>(b, -19);
    b.push_back('#');
    put<std::uint32_t>(b, formula.size());
    b.insert(b.end(), formula.begin(), formula.end());
    b.insert(b.end(), {1, 0});
    if (type_field)
        put<std::int32_t>(b, 2);
    return b;
}
Json decode_drive(const Bytes &body) {
    Bytes b;
    record(b, "`%!", 23, body);
    record(b, "@#$", 18, type_body(18, {}, {31}));
    return complex_blob("BfaTree", b);
}
void str(Bytes &b, const Bytes &s) {
    put<std::uint32_t>(b, s.size());
    b.insert(b.end(), s.begin(), s.end());
}
Bytes property_body(bool variable, unsigned value_type, const Bytes &value) {
    Bytes b;
    ref(b, 23);
    put<std::uint64_t>(b, 0);
    b.push_back(variable);
    str(b, Bytes{'K', 0, 'x'});
    str(b, Bytes{'E'});
    put<std::int32_t>(b, 2); // Declaration deliberately differs from value wire type.
    put<std::uint32_t>(b, value_type);
    b.insert(b.end(), value.begin(), value.end());
    b.push_back(1);
    str(b, Bytes{'m', 'm'});
    str(b, Bytes{'G'});
    str(b, Bytes{'D'});
    return b;
}
Json decode_property(const Bytes &body) {
    Bytes b;
    record(b, "!_#", 23, body);
    record(b, "@#$", 18, type_body(18, {}, {31}));
    return complex_blob("BfaTree", b);
}
Bytes component_body(const Bytes &mapping, bool extended = false) {
    Bytes b;
    ref(b, 40);
    put<std::uint64_t>(b, 0);
    str(b, Bytes{'C', 0, 'x'});
    str(b, Bytes{'D'});
    str(b, Bytes{0, 0xff, '+'});
    str(b, {});
    put<std::int32_t>(b, -3);
    if (extended)
        put<std::uint64_t>(b, 0x1122334455667788ULL);
    b.insert(b.end(), mapping.begin(), mapping.end());
    return b;
}
Json decode_component(const Bytes &body, bool duplicate_property = false) {
    Bytes b;
    record(b, "~$^", 40, body, {23});
    const auto property = property_body(true, 4, {});
    record(b, "!_#", 23, property);
    if (duplicate_property)
        record(b, "!_#", 23, property);
    record(b, "@#$", 18, type_body(18, {}, {31}));
    return complex_blob("BfaTree", b);
}
} // namespace
unsigned bfa_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *message) {
        if (!value)
            throw std::runtime_error(message);
        ++checks;
    };
    const std::uint64_t placed_id = 0xfedcba9876543210ULL;
    const auto body = type_body(17, Bytes{'T', 0, 'x'}, {placed_id, 31, placed_id});
    Bytes b;
    record(b, "@#$", 17, body, {23});
    Bytes driven;
    ref(driven, 23);
    put<std::uint64_t>(driven, 7);
    driven.insert(driven.end(), {0x81, 0x92, 0xa3});
    record(b, "`%!", 23, driven, {999});
    record(b, "@#$", 18, type_body(18, {}, {}));
    const auto j = complex_blob("BfaTree", b);
    const auto &type = j["records"][0];
    check(type["node_kind"] == "component_type" && type["id"] == 17,
          "BFA type definition retains its graph identity");
    check(type["placed_instance_ids"] == Json({placed_id, 31, placed_id}),
          "type instance list preserves full IDs, duplicates and native order");
    check(type["children"] == Json({23}) && j["external_child_ids"] == Json({999}),
          "placed instances do not become definition-tree children");
    check(type["placed_instance_reference_scope"] == "component_project" &&
              type["placed_instance_resolution_status"] == "not_performed",
          "placed IDs are not mislabeled scene entity IDs");
    check(type["type_name"]["text"] == std::string("T\0x", 3) &&
              type["type_name"]["native_value"]["text"] == "T",
          "type names retain full field and native terminated value");
    check(type["body_base64"] == base64(body) && !type.contains("unassigned_suffix_hex"),
          "known placed-reference suffix retains complete original body");
    check(j["records"][1]["node_kind"] == "driven_object" &&
              j["records"][1]["payload_status"] == "malformed_or_unsupported" &&
              j["records"][1]["unassigned_suffix_hex"] == "8192a3",
          "malformed driven-node payload stays bounded and retains its source");
    check(j["records"][2]["placed_instance_ids"].empty(),
          "driven payload does not hide a following type definition");
    auto malformed = body;
    malformed.back() = 0xaa;
    malformed.push_back(1);
    Bytes bad;
    record(bad, "@#$", 17, malformed);
    record(bad, "@#$", 18, type_body(18, {}, {31}));
    auto bj = complex_blob("BfaTree", bad);
    check(bj["records"][0].contains("placed_instance_decode_error") &&
              !bj["records"][0].contains("placed_instance_ids") &&
              bj["records"][1]["placed_instance_ids"] == Json({31}),
          "truncated bounded reference list is isolated from next graph record");
    malformed = body;
    malformed[19 + 4 + 3] = 0;
    bad.clear();
    record(bad, "@#$", 17, malformed);
    check(complex_blob("BfaTree", bad)["records"][0].contains("placed_instance_decode_error"),
          "bad reference marker is not accepted as an instance ID");
    bad.clear();
    record(bad, "@#$", 17, type_body(17, Bytes{0xc4, 0xe3}, {}));
    const auto non_ascii = complex_blob("BfaTree", bad)["records"][0]["type_name"];
    check(non_ascii["encoding"] == "not_established" && non_ascii["text"].is_null() &&
              non_ascii["source_bytes"] == rawbytes(Bytes{0xc4, 0xe3}),
          "new type-name view does not infer source codepage from legacy display decoding");
    bad.clear();
    record(bad, "@#$", 17, type_body(17, Bytes{0x81}, {31}));
    const auto unknown_encoding = complex_blob("BfaTree", bad)["records"][0];
    check(unknown_encoding["placed_instance_ids"] == Json({31}) &&
              unknown_encoding["type_name"]["source_bytes"] == rawbytes(Bytes{0x81}) &&
              unknown_encoding.contains("names_decode_error"),
          "legacy name conversion failure does not hide a valid instance relation");
    Bytes component;
    ref(component, 17);
    put<std::uint32_t>(component, 0);
    component.resize(component.size() + 25);
    const auto c = complex_blob("ParaCmptInstance", component);
    check(c["type_definition_id"] == 17 && c["parent_id"] == 17 &&
              c["type_definition_reference"]["kind"] == "imported_bfa_type",
          "component prefix references its imported type rather than a scene parent");
    std::fill(component.begin(), component.begin() + 11, 0);
    check(complex_blob("ParaCmptInstance",
                       component)["type_definition_reference"]["resolution_status"] == "absent",
          "absent definition reference is not resolved to object zero");
    auto f = std::async(std::launch::async, [&] { return complex_blob("BfaTree", b); });
    check(f.get() == j, "BFA decoding is reentrant");
    Bytes formula{'x', '+', '>', '<', '@'};
    put<std::uint64_t>(formula, 0xffffffffffffffffULL);
    formula.insert(formula.end(), {'#', 'a', '+', '2'});
    const auto db = drive_body(3, formula);
    const auto dj = decode_drive(db);
    const auto &drive = dj["records"][0]["driven"];
    check(drive["target"]["object_id"] == placed_id && drive["target"]["property_id"] == -17 &&
              drive["target"]["body_offset"] == 23,
          "explicit driven target separates full object ID and signed property ID");
    check(drive["inputs"].size() == 4 &&
              drive["inputs"][0]["property_reference_id"] == 0x5d5b23405b5d237cULL &&
              drive["inputs"][1]["property_reference_id"] == 0x5d5b23405b5d237cULL &&
              drive["inputs"][2]["storage_kind"] == 2 && drive["inputs"][3]["object_id"] == 9 &&
              drive["inputs"][3]["property_id"] == -19,
          "driving inputs retain source groups, duplicates, and fixed-width ID payloads");
    check(drive["target"]["resolution_status"] == "requires_component_context" &&
              dj["external_child_ids"].empty(),
          "driven property links do not pretend to be resolved scene or tree edges");
    check(drive["formula"]["valid"] == true && drive["bidirectional"] == false &&
              drive["driven_type"] == "location" && drive["driven_type_code"] == 2,
          "formula validity, direction, and drive type are independent fields");
    const auto &expr = drive["formula"];
    check(expr["parts"].size() == 3 && expr["parts"][1]["encoded_offset"] == 2 &&
              expr["parts"][1]["reference_id_bits"] == 0xffffffffffffffffULL &&
              expr["reference_expanded_form"]["text"] == "x+><@18446744073709551615#a+2",
          "packed formula references expand unsigned bits without evaluating the formula");
    check(expr["encoded_bytes"] == rawbytes(formula) &&
              expr["expression_status"] == "requires_native_expression_conversion" &&
              expr["evaluation_status"] == "not_performed" &&
              dj["records"][0]["body_base64"] == base64(db) &&
              drive["consumed_body_bytes"] == db.size(),
          "binary formula and whole body stay lossless with explicit conversion boundary");
    for (unsigned kind : {1u, 2u}) {
        const auto target = decode_drive(drive_body(kind, {}))["records"][0]["driven"]["target"];
        check(target["storage_kind"] == kind && target["property_reference_id"] == placed_id &&
                  !target.contains("property_id"),
              "component property-object references are not mistaken for property IDs");
    }
    auto legacy = drive_body(1, {}, false);
    auto old_drive = decode_drive(legacy)["records"][0]["driven"];
    check(old_drive["driven_type_status"] == "not_stored" &&
              old_drive["driven_type_code"].is_null() && old_drive["formula"]["parts"].empty(),
          "physically absent legacy drive type is not assigned a guessed default");
    legacy[legacy.size() - 2] = 7;
    legacy.back() = 9;
    const auto invalid_flags = decode_drive(legacy)["records"][0]["driven"];
    check(invalid_flags["formula"]["valid"].is_null() &&
              invalid_flags["formula"]["valid_byte"] == 7 &&
              invalid_flags["bidirectional"].is_null() && invalid_flags["bidirectional_byte"] == 9,
          "noncanonical boolean source bytes are not silently coerced");
    put<std::int32_t>(legacy, 99);
    legacy.push_back(0xaa);
    const auto future = decode_drive(legacy)["records"][0]["driven"];
    check(future["driven_type_code"] == 99 && future["driven_type"].is_null() &&
              future["driven_type_status"] == "unknown_value" &&
              future["unassigned_suffix_hex"] == "aa",
          "future drive values and extra bytes remain explicit");
    for (const auto &incomplete : {Bytes{'>', '<', '@'}, Bytes{'>', '<', '@', 1, 2, 3}}) {
        const auto damaged = decode_drive(drive_body(1, incomplete))["records"][0]["driven"];
        check(damaged["formula"]["token_status"] == "malformed" &&
                  damaged["formula"]["valid"] == true && damaged["driven_type"] == "location",
              "truncated bounded formula cannot consume following flags or type");
    }
    auto odd = decode_drive(drive_body(1, Bytes{'>', 'x', '>', '<', 'y', 0, 0x81}));
    check(odd["records"][0]["driven"]["formula"]["parts"].size() == 1 &&
              odd["records"][0]["driven"]["formula"]["reference_expanded_form"]["text"].is_null(),
          "only complete reference markers are tokens and unknown text encoding stays unknown");
    // Exercise every structural truncation, excluding the complete legacy form.
    bool bounded = true;
    for (std::size_t cut = 19; cut < db.size(); ++cut) {
        auto truncated = decode_drive(Bytes(db.begin(), db.begin() + cut));
        bounded = bounded && truncated["records"].size() == 2 &&
                  truncated["records"][1]["placed_instance_ids"] == Json({31}) &&
                  (cut == db.size() - 4 || truncated["records"][0].contains("decode_error"));
    }
    check(bounded, "all driven-payload truncations remain inside their BFA record");
    for (std::size_t at : {std::size_t(19), std::size_t(23), std::size_t(42)}) {
        auto corrupt = db;
        corrupt[at] = 0xff;
        const auto isolated = decode_drive(corrupt);
        check(isolated["records"][0].contains("decode_error") &&
                  isolated["records"][1]["placed_instance_ids"] == Json({31}),
              "unknown target kind or malformed target/input marker cannot hide the next node");
    }
    auto g = std::async(std::launch::async, [&] { return decode_drive(db); });
    check(g.get() == dj, "driven reference and formula decoding is reentrant");
    Bytes numeric;
    put<double>(numeric, 2.5);
    auto prop = property_body(true, 0, numeric);
    const auto base = decode_property(prop)["records"][0];
    const auto &definition = base["property_definition"];
    check(definition["variable"] == true && definition["declared_type"] == "double" &&
              definition["declared_type_code"] == 2 && definition["base_value"]["type_code"] == 0 &&
              definition["base_value"]["kind"] == "double" &&
              definition["base_value"]["value"] == 2.5,
          "BFA declared type and base value wire type are independent enums");
    check(definition["keys"]["chinese"]["text"] == std::string("K\0x", 3) &&
              definition["keys"]["chinese"]["native_value"]["text"] == "K" &&
              definition["keys"]["english"]["text"] == "E" && definition["readonly"] == true &&
              definition["unit"]["text"] == "mm" && definition["group"]["text"] == "G" &&
              definition["description"]["text"] == "D",
          "property keys retain source and terminated values with independent metadata");
    check(definition["combobox"]["status"] == "not_stored" &&
              definition["consumed_body_bytes"] == prop.size() &&
              base["body_base64"] == base64(prop),
          "legacy base body does not invent combo options or change original bytes");
    prop.push_back(1);
    put<std::uint32_t>(prop, 3);
    str(prop, Bytes{'A'});
    str(prop, Bytes{'A'});
    str(prop, Bytes{0x81});
    const auto combo = decode_property(prop)["records"][0]["property_definition"]["combobox"];
    check(combo["enabled"] == true && combo["options"].size() == 3 &&
              combo["options"][0] == combo["options"][1] && combo["options"][2]["text"].is_null(),
          "combobox preserves order, duplicate values, and unknown text encoding");
    Bytes iv;
    put<std::int64_t>(iv, -42);
    check(decode_property(property_body(
              true, 1, iv))["records"][0]["property_definition"]["base_value"]["value"] == -42,
          "base value wire type one retains a signed 64-bit integer");
    check(decode_property(property_body(
              true, 2,
              Bytes{9}))["records"][0]["property_definition"]["base_value"]["value_status"] ==
              "invalid_boolean",
          "base bool retains malformed source instead of treating it as an integer");
    Bytes sv;
    str(sv, Bytes{'v', 0, 'z'});
    check(decode_property(
              property_body(true, 3, sv))["records"][0]["property_definition"]["base_value"]
                                         ["value"]["native_value"]["text"] == "v",
          "base string keeps native NUL termination separate from full stored bytes");
    auto derived = property_body(false, 4, {});
    derived.push_back('{');
    ref(derived, 18);
    put<std::uint32_t>(derived, 0);
    derived.push_back(1); // Map zero is bool, not base zero's double.
    ref(derived, 18);
    put<std::uint32_t>(derived, 1);
    put<double>(derived, 3.5);
    derived.push_back('}');
    ref(derived, 0xffffffffffffffffULL);
    put<std::uint32_t>(derived, 2);
    put<std::int64_t>(derived, -7);
    ref(derived, 30);
    put<std::uint32_t>(derived, 4);
    str(derived, Bytes{'{', '}', '-', '|', '#', '@', 0});
    ref(derived, 31);
    put<std::uint32_t>(derived, 5); // Map five is none.
    derived.push_back('-');
    const auto controls_at = derived.size();
    derived.insert(derived.end(), {1, 0, 1, 0, 1, 0, 0x7a, 1, 1});
    put<std::int32_t>(derived, 2);
    const auto mapped = decode_property(derived);
    const auto &pd = mapped["records"][0]["property_definition"];
    const auto &maps = pd["unassigned_value_maps"];
    check(maps.size() == 2 && maps[0].size() == 2 && maps[1].size() == 3 &&
              maps[0][0]["value"]["kind"] == "bool" && maps[0][0]["value"]["value"] == true &&
              maps[0][1]["value"]["value"] == 3.5 &&
              maps[1][0]["reference_id"] == 0xffffffffffffffffULL &&
              maps[1][0]["value"]["value"] == -7 && maps[1][1]["value"]["kind"] == "binary" &&
              maps[1][2]["value"]["kind"] == "none",
          "derived map values use their own type enum and bounded binary lengths");
    check(maps[0][0]["reference_id"] == maps[0][1]["reference_id"] &&
              pd["base_value"]["kind"] == "none" && mapped["external_child_ids"].empty(),
          "value-map duplicates remain in source order without changing tree edges");
    const auto &controls = pd["controls"];
    check(controls["inner_property"] == true && controls["type_property"] == false &&
              controls["can_delete"] == true && controls["name_editable"] == false &&
              controls["value_editable"] == true && controls["description_editable"] == false &&
              controls["unassigned_byte"] == 0x7a && controls["value_type_editable"] == true &&
              controls["driven_readonly"] == true && controls["source_type"] == "user",
          "property control flags and source enum follow confirmed independent accessors");
    for (const auto width : {2u, 7u, 8u}) {
        auto short_body = Bytes(derived.begin(), derived.begin() + controls_at + width);
        const auto old = decode_property(short_body)["records"][0]["property_definition"];
        check(old["controls"].contains("value_type_editable") == (width == 8) &&
                  !old["controls"].contains("driven_readonly") &&
                  !old["controls"].contains("source_type_code"),
              "shorter native property control layouts do not synthesize absent fields");
    }
    auto broken = derived;
    broken[controls_at - 1] = '!';
    const auto fail = decode_property(broken);
    check(fail["records"][0].contains("decode_error") &&
              fail["records"][0]["body_base64"] == base64(broken) &&
              fail["records"][1]["placed_instance_ids"] == Json({31}),
          "malformed property table marker remains bounded to its source node");
    for (std::size_t cut = 19; cut < derived.size(); ++cut) {
        const auto cut_result = decode_property(Bytes(derived.begin(), derived.begin() + cut));
        check(cut_result["records"].size() == 2 &&
                  cut_result["records"][1]["placed_instance_ids"] == Json({31}),
              "every property body truncation preserves the following graph record");
    }
    auto pjob = std::async(std::launch::async, [&] { return decode_property(derived); });
    check(pjob.get() == mapped, "property definitions are reentrant");
    const auto &semantics = pd["value_map_semantics"];
    check(semantics[0]["kind"] == "placed_instance_values" &&
              semantics[0]["key_kind"] == "bfa_placed_handle" &&
              semantics[1]["kind"] == "component_type_values" &&
              semantics[1]["key_kind"] == "imported_bfa_type",
          "property value maps retain distinct placed-instance and component-type key scopes");
    check(semantics[0]["selected_entry_indices"] == Json({0}) &&
              semantics[1]["selected_entry_indices"] == Json({1, 2, 0}) &&
              semantics[0]["duplicate_key_rule"] == "first_entry_wins",
          "native map lookup retains first duplicate and sorts unsigned 64-bit keys");
    check(semantics[1]["missing_key_value"] == "none" &&
              !semantics[0].contains("missing_key_value"),
          "type-value miss has native none semantics without inventing an instance fallback");
    Bytes extra_values;
    for (const auto pair :
         {std::make_pair(18ULL, 5LL), std::make_pair(18ULL, 6LL), std::make_pair(23ULL, 7LL)}) {
        ref(extra_values, pair.first);
        put<std::uint32_t>(extra_values, 2);
        put<std::int64_t>(extra_values, pair.second);
    }
    auto bound_body = derived;
    bound_body.insert(bound_body.begin() + controls_at - 1, extra_values.begin(),
                      extra_values.end());
    const auto bound = decode_property(bound_body);
    const auto &bindings = bound["type_property_bindings"];
    check(bindings.size() == 5 && bindings[0]["type_definition_id"] == 18 &&
              bindings[0]["property_definition_id"] == 23 &&
              bindings[0]["property_record_index"] == 0 &&
              bindings[0]["target_status"] == "matched_type_record" &&
              bindings[0]["type_record_index"] == 1,
          "type value binds to a unique typed record in the same source BFA graph");
    const auto &source = bindings[0]["value_source"];
    check(source["map_index"] == 1 && source["entry_index"] == 3 &&
              !bindings[0].contains("value") &&
              bound["records"][0]["property_definition"]["unassigned_value_maps"][1][3]["value"]
                   ["value"] == 5 &&
              bound["records"][0]["property_definition"]["unassigned_value_maps"][1][4]["value"]
                   ["value"] == 6,
          "binding selects native first value by index while preserving later source duplicates");
    check(bindings[1]["target_status"] == "unexpected_node_kind" &&
              !bindings[1].contains("type_record_index") &&
              bindings[2]["target_status"] == "not_in_this_tree" &&
              bound["external_child_ids"].empty(),
          "same-number property node and missing type are not invented type or tree edges");
    Bytes duplicate_tree;
    record(duplicate_tree, "!_#", 23, bound_body);
    record(duplicate_tree, "@#$", 18, type_body(18, {}, {}));
    record(duplicate_tree, "@#$", 18, type_body(18, {}, {}));
    const auto ambiguous = complex_blob("BfaTree", duplicate_tree);
    check(ambiguous["type_property_bindings"][0]["target_status"] == "ambiguous_id" &&
              !ambiguous["type_property_bindings"][0].contains("type_record_index"),
          "duplicate graph identity does not silently resolve by traversal order");
    {
        const auto empty_component = decode_component(component_body(Bytes{'+'}));
        const auto &empty_definition = empty_component["records"][0]["component_definition"];
        check(empty_definition["mapping_layout_status"] == "unique_candidate" &&
                  empty_definition["mapping_layout_candidates"][0]["has_context_uint64"] == false &&
                  empty_definition["mapping_layout_candidates"][0]["property_id_maps"][1]
                                  ["terminator_stored"] == false &&
                  empty_component["component_property_bindings"].empty(),
              "legacy empty property ID maps terminate at their bounded component body");
        check(empty_definition["unassigned_binary_fields"][0]["base64"] ==
                      base64(Bytes{0, 0xff, '+'}) &&
                  empty_definition["name_fields"][0]["native_value"]["text"] == "C" &&
                  empty_definition["unassigned_int32"] == -3,
              "component prefix retains bounded binary fields and full text before native NUL "
              "handling");
        Bytes mapping;
        for (const auto pair : {std::make_pair(23ULL, 5LL), std::make_pair(23ULL, -42LL),
                                std::make_pair(placed_id, 88LL), std::make_pair(18ULL, 9LL)}) {
            ref(mapping, pair.first);
            put<std::int64_t>(mapping, pair.second);
        }
        mapping.push_back('+');
        ref(mapping, 23);
        put<std::int64_t>(mapping, -17);
        const auto cb = component_body(mapping);
        const auto component = decode_component(cb);
        const auto &layout =
            component["records"][0]["component_definition"]["mapping_layout_candidates"][0];
        const auto &maps = layout["property_id_maps"];
        check(maps[0]["selected_entry_indices"] == Json({3, 1, 2}) &&
                  maps[0]["entries"].size() == 4 && maps[0]["entries"][0]["property_id"] == 5 &&
                  maps[0]["entries"][1]["property_id"] == -42 &&
                  maps[0]["entries"][1]["property_id_bits"] == std::uint64_t(-42) &&
                  maps[0]["duplicate_key_rule"] == "last_entry_wins",
              "component mapping uses native last assignment and unsigned node-key ordering");
        const auto &cpb = component["component_property_bindings"];
        check(cpb.size() == 4 && cpb[1]["target_status"] == "matched_property_record" &&
                  cpb[1]["component_definition_id"] == 40 &&
                  cpb[1]["component_record_index"] == 0 && cpb[1]["property_record_index"] == 1 &&
                  cpb[1]["property_id"] == -42 && cpb[1]["mapping_source"]["entry_index"] == 1 &&
                  cpb[3]["property_id"] == -17 && cpb[3]["mapping_source"]["map_index"] == 1 &&
                  maps[0]["driven_storage_kind"] == 1 && maps[1]["driven_storage_kind"] == 2,
              "SDK IDs bind through each distinct source map without conflating graph node IDs");
        check(cpb[0]["target_status"] == "unexpected_node_kind" &&
                  cpb[2]["target_status"] == "not_in_this_tree" &&
                  component["records"][0]["children"] == Json({23}) &&
                  component["external_child_ids"].empty(),
              "ID mapping does not invent tree edges or match a type node as an attribute");
        const auto duplicate_component = decode_component(cb, true);
        check(duplicate_component["component_property_bindings"][1]["target_status"] ==
                      "ambiguous_id" &&
                  !duplicate_component["component_property_bindings"][1].contains(
                      "property_record_index"),
              "duplicate property nodes leave mapping target unresolved");
        mapping.push_back('+');
        mapping.insert(mapping.end(), {0x91, 0x82});
        const auto newer = decode_component(component_body(mapping, true));
        const auto &new_definition = newer["records"][0]["component_definition"];
        const auto &new_layout = new_definition["mapping_layout_candidates"][0];
        check(new_definition["mapping_layout_status"] == "unique_candidate" &&
                  new_layout["has_context_uint64"] == true &&
                  new_layout["unassigned_context_uint64"] == 0x1122334455667788ULL &&
                  new_layout["unassigned_suffix_hex"] == "9182" &&
                  newer["component_property_bindings"].size() == 4,
              "extended component prefix and unparsed tail remain separate from native ID maps");
        Bytes ambiguous_mapping{'+'};
        ref(ambiguous_mapping, 0x00002b2b00000007ULL);
        put<std::int64_t>(ambiguous_mapping, 23);
        ambiguous_mapping.push_back('+');
        const auto ambiguous_component = decode_component(component_body(ambiguous_mapping));
        check(ambiguous_component["records"][0]["component_definition"]["mapping_layout_status"] ==
                      "requires_version_context" &&
                  ambiguous_component["records"][0]["component_definition"]
                                     ["mapping_layout_candidates"]
                                         .size() == 2 &&
                  ambiguous_component["component_property_bindings"].empty(),
              "two plausible component layouts retain both candidates without fabricated bindings");
        const auto invalid_component = decode_component(component_body(Bytes{0xff}));
        check(invalid_component["records"][0]["component_definition"]["mapping_layout_status"] ==
                      "malformed_or_unsupported" &&
                  invalid_component["component_property_bindings"].empty() &&
                  invalid_component["records"][2]["placed_instance_ids"] == Json({31}),
              "invalid component maps do not affect the following graph records");
        for (std::size_t cut = 19; cut < cb.size(); ++cut) {
            const auto cut_result = decode_component(Bytes(cb.begin(), cb.begin() + cut));
            check(cut_result["records"][0]["body_base64"] ==
                          base64(Bytes(cb.begin(), cb.begin() + cut)) &&
                      cut_result["records"][2]["placed_instance_ids"] == Json({31}),
                  "truncated component definitions preserve source bytes and subsequent nodes");
        }
        auto cjob = std::async(std::launch::async, [&] { return decode_component(cb); });
        check(cjob.get() == component, "component mapping decode is reentrant");
    }
    {
        auto decode_extension = [&](const Bytes &tail, bool extended = true) {
            Bytes mapping{'+', '+'};
            mapping.insert(mapping.end(), tail.begin(), tail.end());
            return decode_component(component_body(mapping, extended));
        };
        auto extension_of = [](const Json &tree) -> const Json & {
            return tree["records"][0]["component_definition"]["mapping_layout_candidates"][0]
                       ["extension"];
        };
        Bytes names;
        for (const auto key : {7ULL, 7ULL, 0xfedcba9876543210ULL}) {
            put(names, key);
            str(names, Bytes{'N', 0, 'x'});
        }
        const auto legacy = decode_extension(names, false);
        const auto &legacy_extension = extension_of(legacy);
        check(legacy_extension["layout_status"] == "unique_candidate" &&
                  legacy_extension["layout_candidates"][0]["has_driven_point_section"] == false &&
                  !legacy_extension["layout_candidates"][0].contains("driven_points"),
              "older display-name map terminates at payload end without invented driven points");
        const auto &name_map =
            legacy_extension["layout_candidates"][0]["offset_to_display_name_map"];
        check(name_map["entries"].size() == 3 &&
                  name_map["selected_entry_indices"] == Json({1, 2}) &&
                  name_map["entries"][0]["display_name"]["text"] == std::string("N\0x", 3) &&
                  name_map["entries"][1]["display_name"]["native_value"]["text"] == "N",
              "display names retain source bytes, native NUL prefix and last unsigned-key "
              "assignment");
        Bytes points = names;
        points.push_back('+');
        auto add_point = [&](std::uint64_t snap, const std::vector<std::uint64_t> &codes,
                             bool terminator) {
            str(points, Bytes{'P'});
            str(points, Bytes{'D', 0, 'x'});
            put(points, snap);
            for (const auto code : codes) {
                put(points, code);
                str(points, Bytes{'x', '+', '@', 0, 'y'});
            }
            if (terminator)
                points.push_back('@');
        };
        add_point(1u | 4u | (1u << 25), {1, 0x100000001ULL, 2, 4}, true);
        add_point(0xffffffffffffffffULL, {std::uint64_t(-7), 5}, false);
        const auto tree = decode_extension(points);
        const auto &extension = extension_of(tree);
        check(tree["records"][0]["component_definition"]["next_property_id"] == -3 &&
                  tree["records"][0]["component_definition"]["mapping_layout_candidates"][0]
                      ["related_tree_reference"]["tree_id"] == 0x1122334455667788LL &&
                  tree["records"][0]["component_definition"]["mapping_layout_candidates"][0]
                      ["related_tree_reference"]["scope"] == "active_project" &&
                  !legacy["records"][0]["component_definition"]["mapping_layout_candidates"][0]
                       .contains("related_tree_reference"),
              "property ID allocator and optional related tree retain their native signed scopes");
        Bytes absent_tree_body = component_body(Bytes{'+', '+'}, true);
        std::fill(absent_tree_body.end() - 10, absent_tree_body.end() - 2, std::uint8_t(0xff));
        const auto absent_tree = decode_component(absent_tree_body);
        check(absent_tree["records"][0]["component_definition"]["mapping_layout_candidates"][0]
                         ["related_tree_reference"]["tree_id"] == -1 &&
                  absent_tree["records"][0]["component_definition"]["mapping_layout_candidates"][0]
                             ["unassigned_context_uint64"] == std::uint64_t(-1),
              "signed related-tree sentinel preserves the original unsigned compatibility bits");
        check(extension["layout_status"] == "unique_candidate" &&
                  extension["layout_candidates"][0]["has_driven_point_section"] == true &&
                  !tree["records"][0]["component_definition"]["mapping_layout_candidates"][0]
                       .contains("unassigned_suffix_hex"),
              "driven-point extension consumes its complete source layout");
        const auto &decoded_points = extension["layout_candidates"][0]["driven_points"];
        check(decoded_points.size() == 2 && decoded_points[0]["name"]["text"] == "P" &&
                  decoded_points[0]["description"]["native_value"]["text"] == "D" &&
                  decoded_points[1]["name"]["text"] == "P",
              "driven-point array preserves duplicate names and independent source order");
        check(decoded_points[0]["selected_formula_indices"] == Json({1, 2, 3}) &&
                  decoded_points[0]["formulas"].size() == 4 &&
                  decoded_points[0]["formulas"][1]["coordinate_wire_uint64"] == 0x100000001ULL &&
                  decoded_points[0]["formulas"][1]["coordinate_type_code"] == 1 &&
                  decoded_points[0]["formulas"][3]["coordinate_type"] == "all" &&
                  decoded_points[0]["formulas"][1]["formula"]["native_value"]["text"] == "x+@",
              "formula keys use native low signed DWORD and bounded strings preserve marker bytes");
        check(decoded_points[0]["native_is_valid"] == true &&
                  decoded_points[0]["formula_status"] == "not_evaluated" &&
                  decoded_points[0]["snap_mode_flags"] == Json({"Nearest", "MidPoint"}) &&
                  decoded_points[0]["unknown_snap_mode_bits"] == (1u << 25),
              "native validity checks map size and snap sentinel without demanding XYZ or "
              "evaluating formulas");
        check(decoded_points[1]["native_is_valid"] == false &&
                  decoded_points[1]["snap_mode_code"] == -1 &&
                  decoded_points[1]["snap_mode_status"] == "invalid" &&
                  decoded_points[1]["snap_mode_flags"].empty() &&
                  decoded_points[1]["selected_formula_indices"] == Json({0, 1}) &&
                  decoded_points[1]["formulas"][0]["coordinate_type_code"] == -7 &&
                  decoded_points[1]["formulas"][1]["coordinate_type"].is_null() &&
                  decoded_points[1]["terminator_stored"] == false,
              "unknown coordinate values and native end-of-body termination remain represented");
        const auto empty_points = decode_extension(Bytes{'+'});
        check(extension_of(empty_points)["layout_candidates"][0]["driven_points"].empty(),
              "explicit extension separator preserves an empty driven-point array");
        Bytes plus_key;
        put<std::uint64_t>(plus_key, 43);
        str(plus_key, Bytes{'K'});
        const auto plus_name = decode_extension(plus_key);
        check(
            extension_of(plus_name)["layout_status"] == "unique_candidate" &&
                extension_of(plus_name)["layout_candidates"][0]["has_driven_point_section"] ==
                    false &&
                extension_of(plus_name)["layout_candidates"][0]["offset_to_display_name_map"]
                                       ["entries"][0]["offset_key"] == 43,
            "a plus byte within an older offset key is not unconditionally treated as a separator");
        Bytes dual;
        put<std::uint64_t>(dual, 43);
        put<std::uint32_t>(dual, 256);
        dual.resize(17, 0);
        put<std::uint64_t>(dual, 1);
        put<std::uint32_t>(dual, 239);
        dual.resize(268, 'x');
        const auto dual_tree = decode_extension(dual);
        check(extension_of(dual_tree)["layout_status"] == "requires_version_context" &&
                  extension_of(dual_tree)["layout_candidates"].size() == 2 &&
                  dual_tree["records"][0]["component_definition"]["mapping_layout_candidates"][0]
                           ["unassigned_suffix_hex"] == hex(dual),
              "ambiguous display-name and driven-point layouts retain both alternatives and source "
              "bytes");
        Bytes malformed = points;
        malformed.pop_back();
        const auto bad = decode_extension(malformed);
        check(extension_of(bad)["layout_status"] == "malformed_or_unsupported" &&
                  bad["records"][2]["placed_instance_ids"] == Json({31}) &&
                  bad["records"][0]["component_definition"]["mapping_layout_candidates"][0]
                     ["unassigned_suffix_hex"] == hex(malformed),
              "truncated formula retains the complete undecoded extension and following records");
        for (std::size_t cut = names.size() + 1; cut < points.size(); ++cut) {
            const auto truncated = decode_extension(Bytes(points.begin(), points.begin() + cut));
            check(truncated["records"].size() == 3 &&
                      truncated["records"][2]["placed_instance_ids"] == Json({31}),
                  "every driven-point truncation remains inside its bounded definition");
        }
        auto job = std::async(std::launch::async, [&] { return decode_extension(points); });
        check(job.get() == tree, "component extensions decode concurrently without shared state");
    }
    {
        auto with_id = [](Bytes body, std::uint64_t id) {
            Bytes header;
            ref(header, id);
            std::copy(header.begin(), header.end(), body.begin());
            return body;
        };
        auto drive_references = [&](unsigned kind, std::uint64_t target) {
            Bytes body;
            ref(body, 90);
            put<std::uint64_t>(body, 0);
            put<std::uint32_t>(body, kind);
            ref(body, target);
            if (kind == 3)
                put<std::int64_t>(body, -7);
            for (auto id : {41, 41, 0, 18, 999})
                ref(body, id);
            body.push_back('[');
            ref(body, 42);
            body.push_back(']');
            for (auto id : {31, 18}) {
                ref(body, id);
                put<std::int64_t>(body, -17);
            }
            body.push_back('#');
            str(body, {});
            body.insert(body.end(), {1, 0});
            put<std::int32_t>(body, 0);
            return body;
        };
        auto make_graph = [&](unsigned kind, std::uint64_t target, bool duplicate) {
            Bytes mapping;
            ref(mapping, 41);
            put<std::int64_t>(mapping, 101);
            mapping.push_back('+');
            ref(mapping, 42);
            put<std::int64_t>(mapping, 102);
            Bytes graph;
            record(graph, "~$^", 40, component_body(mapping), {90, 41, 42, 31, 18});
            auto ordinary = property_body(false, 4, {});
            ordinary.insert(ordinary.end(), {'{', '}', '-', 0, 0});
            record(graph, "!_#", 41, with_id(ordinary, 41));
            record(graph, "!_#", 42, with_id(property_body(true, 4, {}), 42));
            Bytes primitive;
            ref(primitive, 31);
            put<std::uint64_t>(primitive, 0);
            record(graph, "&@`", 31, primitive);
            record(graph, "@#$", 18, type_body(18, {}, {}));
            record(graph, "`%!", 90, drive_references(kind, target));
            Bytes other_map;
            ref(other_map, 41);
            put<std::int64_t>(other_map, 201);
            other_map.push_back('+');
            record(graph, "~$^", 50, with_id(component_body(other_map), 50));
            if (duplicate)
                record(graph, "!_#", 41, with_id(ordinary, 41));
            if (target == 0)
                record(graph, "!_#", 0, with_id(ordinary, 0));
            return complex_blob("BfaTree", graph);
        };
        const auto graph = make_graph(1, 41, false);
        const auto &bindings = graph["driven_reference_bindings"];
        check(bindings.size() == 9 && bindings[0]["driven_object_id"] == 90 &&
                  bindings[0]["driven_record_index"] == 5 &&
                  bindings[0]["reference_source"]["role"] == "target" &&
                  bindings[0]["target_status"] == "matched_property_record" &&
                  bindings[0]["property_record_index"] == 1,
              "driven target node reference binds to the unique local property record");
        check(bindings[0]["component_mapping_binding_indices"] == Json({0, 2}) &&
                  bindings[0]["environment_selection_status"] == "not_performed" &&
                  bindings[0]["runtime_resolution_status"] == "requires_component_context" &&
                  !bindings[0].contains("property_id") &&
                  graph["component_property_bindings"][0]["property_id"] == 101 &&
                  graph["component_property_bindings"][2]["property_id"] == 201,
              "multiple component environments remain explicit mapping candidates without guessing "
              "an SDK ID");
        check(graph["node_parent_bindings"][1]["status"] == "matched_parent_record" &&
                  graph["node_parent_bindings"][1]["parent_record_index"] == 0 &&
                  graph["node_parent_bindings"][1]["parent_id"] == 40 &&
                  graph["node_parent_bindings"][1]["native_identity_status"] == "preserved" &&
                  bindings[0]["parent_binding_index"] == 1 &&
                  bindings[0]["parent_component_mapping_binding_indices"] == Json({0}) &&
                  bindings[0]["component_mapping_binding_indices"] == Json({0, 2}) &&
                  graph["component_property_bindings"][0]["parent_binding_index"] == 1,
              "serialized parent identifies the owning component map without losing other sources");
        check(bindings[1]["reference_source"]["input_index"] == 0 &&
                  bindings[2]["reference_source"]["input_index"] == 1 &&
                  bindings[1]["property_record_index"] == bindings[2]["property_record_index"] &&
                  graph["records"][5]["driven"]["inputs"].size() == 8,
              "duplicate driven inputs preserve their source positions and share the original "
              "property record");
        check(bindings[3]["target_status"] == "not_in_this_tree" &&
                  bindings[4]["target_status"] == "unexpected_node_kind" &&
                  bindings[5]["target_status"] == "not_in_this_tree" &&
                  bindings[8]["target_status"] == "unexpected_node_kind" &&
                  graph["external_child_ids"].empty() &&
                  graph["records"][0]["children"] == Json({90, 41, 42, 31, 18}),
              "driven references do not invent hierarchy edges or resolve missing and wrong-kind "
              "targets");
        check(bindings[6]["storage_kind"] == 2 && bindings[6]["property_record_index"] == 2 &&
                  bindings[6]["component_mapping_binding_indices"] == Json({1}),
              "the second driven storage family uses only the matching component property map");
        check(bindings[7]["target_status"] == "matched_object_record" &&
                  bindings[7]["object_record_index"] == 3 && bindings[7]["property_id"] == -17 &&
                  bindings[7]["property_resolution_status"] == "requires_object_property_schema" &&
                  !bindings[7].contains("property_record_index"),
              "explicit object-property pair preserves SDK ID without mistaking it for a property "
              "node");
        const auto explicit_target = make_graph(3, 31, false);
        check(explicit_target["driven_reference_bindings"][0]["target_status"] ==
                      "matched_object_record" &&
                  explicit_target["driven_reference_bindings"][0]["property_id"] == -7,
              "explicit driven target and input pairs follow the same scoped object lookup");
        const auto unmatched_family = make_graph(2, 41, false);
        check(unmatched_family["driven_reference_bindings"][0]["target_status"] ==
                      "matched_property_record" &&
                  unmatched_family["driven_reference_bindings"][0]
                                  ["component_mapping_binding_indices"]
                                      .empty(),
              "a property record can exist without a matching map family and is not force-mapped "
              "through the other table");
        const auto ambiguous = make_graph(1, 41, true);
        check(ambiguous["driven_reference_bindings"][0]["target_status"] == "ambiguous_id" &&
                  !ambiguous["driven_reference_bindings"][0].contains("property_record_index") &&
                  ambiguous["driven_reference_bindings"][6]["target_status"] ==
                      "matched_property_record",
              "duplicate property identity blocks only affected driven bindings");
        const auto zero = make_graph(1, 0, false);
        check(
            zero["driven_reference_bindings"][0]["target_status"] == "inactive_zero_target" &&
                !zero["driven_reference_bindings"][0].contains("property_record_index") &&
                zero["driven_reference_bindings"][3]["target_status"] == "matched_property_record",
            "native target ignores zero while input enumeration retains the serialized zero entry");
        const auto resolved = resolve_bfa_driven_references(graph, 90, {40, true});
        check(resolved["status"] == "resolved" &&
                  resolved["target"]["status"] == "handle_constructed" &&
                  resolved["target"]["object_id"] == 40 &&
                  resolved["target"]["property_id"] == 101 &&
                  resolved["target"]["handle_kind_code"] == 10 &&
                  resolved["target"]["property_validation"] == "not_performed",
              "target conversion resolves its parent component and constructs the native property "
              "handle");
        check(resolved["inputs_status"] == "native_early_return" &&
                  resolved["input_handle_source_indices"] == Json({0, 1}) &&
                  resolved["inputs"][0]["property_id"] == 101 &&
                  resolved["inputs"][1]["property_id"] == 101 &&
                  resolved["input_stop_index"] == 2 &&
                  resolved["inputs"][2]["lookup_environment_id"] == 0xffffffffffffffffULL &&
                  resolved["inputs"][3]["status"] == "not_visited_after_native_return" &&
                  resolved["inputs"].size() == 8,
              "native input conversion preserves duplicate prefix handles and returns before later "
              "inputs on missing owner");
        const auto partial = resolve_bfa_driven_references(graph, 90, {40, false});
        check(partial["target"]["status"] == "handle_constructed" &&
                  partial["inputs_status"] == "unresolved" && partial["input_stop_index"] == 0 &&
                  partial["inputs"][1]["status"] == "not_evaluated_after_unresolved",
              "partial registry can resolve a known parent but cannot infer a globally parentless "
              "editing environment");
        auto selected = graph;
        selected["records"][5]["driven"]["inputs"] = Json::array(
            {graph["records"][5]["driven"]["inputs"][0], graph["records"][5]["driven"]["inputs"][5],
             graph["records"][5]["driven"]["inputs"][6]});
        const auto mixed = resolve_bfa_driven_references(selected, 90, {40, true});
        check(mixed["inputs_status"] == "complete" &&
                  mixed["input_handle_source_indices"] == Json({0, 1, 2}) &&
                  mixed["inputs"][1]["property_id"] == 0 &&
                  mixed["inputs"][1]["lookup_trace"].size() == 2 &&
                  mixed["inputs"][1]["lookup_trace"][1]["matched"] == false &&
                  mixed["inputs"][2]["object_id"] == 31 && mixed["inputs"][2]["property_id"] == -17,
              "input miss runs second-map inverse lookup and preserves a zero result; non-root "
              "explicit object pairs pass through");
        const auto family_two =
            resolve_bfa_driven_references(make_graph(2, 41, false), 90, {40, true});
        const auto target_miss =
            resolve_bfa_driven_references(make_graph(2, 42, false), 90, {40, true});
        check(family_two["target"]["property_id"] == 101 &&
                  target_miss["target"]["status"] == "default_handle" &&
                  target_miss["target"]["lookup_trace"].size() == 1,
              "target conversion first uses the ordinary forward map for both stored families and "
              "does not fall back on minus one");
        auto zero_forward = selected;
        auto &zero_maps = zero_forward["records"][0]["component_definition"]
                                      ["mapping_layout_candidates"][0]["property_id_maps"];
        zero_maps[0]["entries"][0]["property_id"] = 0;
        for (const auto pair :
             {std::make_pair(77, 41), std::make_pair(88, 41), std::make_pair(88, 99)})
            zero_maps[1]["entries"].push_back(
                {{"property_definition_id", pair.first}, {"property_id", pair.second}});
        const auto different = resolve_bfa_driven_references(zero_forward, 90, {40, true});
        check(different["target"]["property_id"] == 77 &&
                  different["target"]["lookup_trace"].size() == 2 &&
                  different["target"]["lookup_trace"][1]["direction"] == "property_to_definition" &&
                  different["inputs"][0]["property_id"] == 0 &&
                  different["inputs"][0]["lookup_trace"].size() == 1,
              "target zero invokes inverse lookup after map overrides while input zero is retained "
              "without fallback");
        auto minus_one = selected;
        minus_one["records"][0]["component_definition"]["mapping_layout_candidates"][0]
                 ["property_id_maps"][0]["entries"][0]["property_id"] = -1;
        const auto sentinel = resolve_bfa_driven_references(minus_one, 90, {40, true});
        check(sentinel["target"]["status"] == "default_handle" &&
                  sentinel["target"]["lookup_trace"][0]["matched"] == true &&
                  sentinel["inputs"][0]["property_id"] == 0,
              "a stored minus-one value follows the same native sentinel branch as a missing "
              "forward key");
        auto wrong_owner = selected;
        wrong_owner["records"][0]["children"] = Json({90, 42, 31, 18});
        wrong_owner["records"][3]["children"] = Json({41});
        const auto redirected = resolve_bfa_driven_references(wrong_owner, 90, {40, true});
        check(redirected["target"]["lookup_environment_id"] == 31 &&
                  redirected["target"]["object_id"] == 40 &&
                  redirected["target"]["property_id"] == 41 &&
                  redirected["inputs_status"] == "native_early_return" &&
                  redirected["inputs"][0]["reason"] == "environment_is_not_component",
              "target primitive branch retains the original pair while input requires a component "
              "environment and aborts");
        const auto explicit_pair = resolve_bfa_driven_references(explicit_target, 90, {40, true});
        check(explicit_pair["target"]["object_id"] == 31 &&
                  explicit_pair["target"]["property_id"] == -7 &&
                  explicit_pair["target"]["property_id_bits"] == std::uint64_t(-7),
              "explicit primitive target preserves the signed SDK property bit pattern");
        const auto changed_editing = resolve_bfa_driven_references(selected, 90, {31, true});
        check(changed_editing["target"]["property_id"] == 101 &&
                  changed_editing["inputs"][0]["object_id"] == 31 &&
                  changed_editing["inputs"][0]["property_id"] == 41,
              "input environment predicate tests parentlessness rather than assuming the current "
              "editing ID is a component root");
        const auto missing_editing = resolve_bfa_driven_references(selected, 90, {999, true});
        const auto unknown_editing = resolve_bfa_driven_references(selected, 90, {999, false});
        check(missing_editing["inputs"][0]["status"] == "handle_constructed" &&
                  missing_editing["inputs"][0]["object_id"] == 999 &&
                  unknown_editing["inputs"][0]["status"] == "unresolved",
              "missing pair object passes through only with complete registry evidence; partial "
              "absence is unresolved");
        const auto no_drive = resolve_bfa_driven_references(graph, 123, {40, true});
        const auto unknown_drive = resolve_bfa_driven_references(graph, 123, {40, false});
        check(no_drive["status"] == "drive_unavailable" && no_drive["inputs"].empty() &&
                  unknown_drive["status"] == "unresolved",
              "drive absence distinguishes a complete selected registry from incomplete file "
              "context");
        auto versioned = graph;
        versioned["records"][0]["component_definition"]["mapping_layout_status"] =
            "requires_version_context";
        check(resolve_bfa_driven_references(versioned, 90, {40, true})["target"]["status"] ==
                  "unresolved",
              "ambiguous component layouts cannot construct effective driver property handles");
        auto wide = graph;
        wide["records"][0]["id"] = 0x100000028ULL;
        check(resolve_bfa_driven_references(wide, 90, {40, true})["reason"] ==
                      "native_node_id_conversion_required" &&
                  resolve_bfa_driven_references(ambiguous, 90, {40, true})["target"]["status"] ==
                      "unresolved" &&
                  resolve_bfa_driven_references(Json::object(), 90, {40, true})["status"] ==
                      "invalid_input",
              "native ID conversion, duplicate identities and malformed API inputs remain explicit "
              "failures");
        auto declared_missing = selected;
        declared_missing["records"][0]["children"].push_back(999);
        const auto placeholder = resolve_bfa_driven_references(declared_missing, 90, {999, true});
        check(placeholder["inputs"][0]["status"] == "unresolved" &&
                  placeholder["inputs"][0]["reason"] == "unloaded_child_placeholder",
              "a serialized child without a supplied body cannot be treated as absent because "
              "native loading registers placeholders");
        auto zero_environment = make_graph(1, 0, false);
        zero_environment["records"].erase(zero_environment["records"].end() -
                                          1); // Remove the separate property record with ID zero.
        zero_environment["records"][0]["id"] = 0;
        zero_environment["records"][0]["component_definition"]["mapping_layout_candidates"][0]
                        ["property_id_maps"][0]["entries"]
                            .push_back({{"property_definition_id", 0}, {"property_id", 9}});
        const auto zero_handle = resolve_bfa_driven_references(zero_environment, 90, {0, true});
        check(zero_handle["target"]["pair_object_id"] == 0 &&
                  zero_handle["target"]["pair_property_bits"] == 0 &&
                  zero_handle["target"]["object_id"] == 0 &&
                  zero_handle["target"]["property_id"] == 9 &&
                  zero_handle["target"]["status"] == "handle_constructed",
              "an inactive zero target slot still follows environment-zero lookup rather than "
              "being declared unconditionally invalid");
        auto resolve_job = std::async(std::launch::async, [&] {
            return resolve_bfa_driven_references(graph, 90, {40, true});
        });
        check(resolve_job.get() == resolved && graph == make_graph(1, 41, false),
              "contextual drive conversion is reentrant and never mutates the supplied decoded "
              "graph");
        auto job = std::async(std::launch::async, [&] { return make_graph(1, 41, false); });
        check(job.get() == graph, "driven bindings are deterministic across concurrent decoding");
    }
    {
        Bytes mapping;
        for (const auto pair :
             {std::make_pair(9ULL, 17LL), std::make_pair(4ULL, 17LL), std::make_pair(9ULL, 22LL),
              std::make_pair(7ULL, 17LL), std::make_pair(0xffffffffffffffffULL, -5LL),
              std::make_pair(0ULL, -5LL), std::make_pair(2ULL, 0LL), std::make_pair(3ULL, 0LL)}) {
            ref(mapping, pair.first);
            put<std::int64_t>(mapping, pair.second);
        }
        mapping.push_back('+');
        ref(mapping, 1);
        put<std::int64_t>(mapping, 17);
        const auto tree = decode_component(component_body(mapping));
        const auto &maps = tree["records"][0]["component_definition"]["mapping_layout_candidates"]
                               [0]["property_id_maps"];
        const auto &inverse = maps[0]["inverse_lookup"];
        check(inverse["selected_entry_indices"] == Json({4, 7, 3, 2}) &&
                  inverse["duplicate_value_rule"] == "greatest_unsigned_definition_id_wins" &&
                  inverse["index_order"] == "signed_property_id",
              "inverse lookup uses the greatest effective unsigned node key rather than the last "
              "source row");
        check(maps[0]["entries"].size() == 8 && maps[0]["entries"][0]["property_id"] == 17 &&
                  maps[0]["selected_entry_indices"] == Json({5, 6, 7, 1, 3, 2, 4}) &&
                  inverse["missing_key_value"] == 0 &&
                  maps[1]["inverse_lookup"]["selected_entry_indices"] == Json({0}),
              "inverse lookup excludes overwritten node values, keeps both families independent "
              "and preserves the zero miss sentinel");
    }
    {
        auto node_body = [](std::uint64_t id) {
            Bytes body;
            ref(body, id);
            put<std::uint64_t>(body, 987654321); // Not the parent ID.
            return body;
        };
        auto graph_with_parents = [&](const std::vector<std::uint64_t> &first,
                                      const std::vector<std::uint64_t> &second, std::uint64_t child,
                                      bool duplicate_parent) {
            Bytes data;
            record(data, "&@`", 30, node_body(30), first);
            record(data, "&@`", 31, node_body(31), second);
            record(data, "&@`", child, node_body(child));
            if (duplicate_parent)
                record(data, "&@`", 30, node_body(30));
            return complex_blob("BfaTree", data);
        };
        auto ordinary = graph_with_parents({42, 999}, {}, 42, false);
        const auto &parents = ordinary["node_parent_bindings"];
        check(parents.size() == 3 && parents[2]["status"] == "matched_parent_record" &&
                  parents[2]["parent_id"] == 30 &&
                  parents[2]["parent_sources"] ==
                      Json::array({{{"parent_record_index", 0}, {"child_index", 0}}}) &&
                  ordinary["external_child_ids"] == Json({999}) &&
                  ordinary["records"][2]["unassigned_header_uint64"] == 987654321,
              "parent provenance comes from child lists without consuming the unassigned body "
              "header or missing children");
        check(parents[0]["status"] == "no_parent_in_this_tree" &&
                  parents[0]["parent_sources"].empty(),
              "a local parentless record does not invent an external project environment");
        auto repeated = graph_with_parents({42, 42}, {}, 42, false);
        check(repeated["node_parent_bindings"][2]["status"] == "ambiguous_parent" &&
                  repeated["node_parent_bindings"][2]["parent_sources"].size() == 2 &&
                  !repeated["node_parent_bindings"][2].contains("parent_record_index"),
              "duplicate child occurrences remain visible without guessing native placeholder "
              "replacement");
        auto multiple = graph_with_parents({42}, {42}, 42, false);
        check(multiple["node_parent_bindings"][2]["status"] == "ambiguous_parent",
              "multiple declared parents are not resolved by record order");
        auto duplicate = graph_with_parents({42}, {}, 42, true);
        check(duplicate["node_parent_bindings"][0]["status"] == "ambiguous_node_id" &&
                  duplicate["node_parent_bindings"][2]["status"] == "ambiguous_parent_id",
              "duplicate parent identities prevent an apparently unique edge from selecting an "
              "environment");
        auto self = graph_with_parents({30}, {}, 42, false);
        check(self["node_parent_bindings"][0]["status"] == "self_parent",
              "self-parent is reported without recursive traversal");
        for (const auto id : {0x10000002aULL, 0x80000000ULL}) {
            const auto wide = graph_with_parents({id}, {}, id, false);
            check(wide["node_parent_bindings"][2]["native_identity_status"] ==
                          "requires_32bit_conversion" &&
                      wide["records"][2]["id"] == id,
                  "wide source IDs remain lossless rather than being silently narrowed to native "
                  "wrapper IDs");
        }
        const auto negative =
            graph_with_parents({0xffffffffffffffffULL}, {}, 0xffffffffffffffffULL, false);
        check(negative["node_parent_bindings"][2]["native_identity_status"] == "preserved",
              "sign-extended signed-32 node identities preserve their original 64-bit pattern");
        auto job = std::async(std::launch::async,
                              [&] { return graph_with_parents({42, 999}, {}, 42, false); });
        check(job.get() == ordinary, "parent source indexing is reentrant");
    }
    {
        Bytes bytes;
        for (const auto row : {std::array<std::uint64_t, 3>{31, 8, 9},
                               {31, 8, 10},
                               {99, 8, 9},
                               {0xffffffffffffffffULL, 11, 12}})
            for (auto word : row)
                put(bytes, word);
        const auto decoded = application_blob("DataIdMap", bytes, "BPParaHandleAndBfaDataMap");
        check(decoded["entries"].size() == 4 &&
                  decoded["native_lookup"]["selected_entry_indices"] == Json({0, 2, 3}) &&
                  decoded["native_lookup"]["duplicate_key_rule"] == "first_entry_wins" &&
                  decoded["native_lookup"]["target_key_kind"] == "BPDataKey" &&
                  decoded["native_lookup"]["target_field"] == "BfaTree",
              "component definition data map keeps the first handle key and unsigned ordering "
              "while retaining all rows");
        check(!application_blob("DataIdMap", bytes, "BPParaHandleAndEntityDataMap")
                      .contains("native_lookup") &&
                  !application_blob("DataIdMap", bytes, "").contains("native_lookup"),
              "definition lookup semantics are not imposed on same-named fields of other schemas");
        Json objects =
            Json::array({{{"class_id", 7}, {"object_id", 1}}, {{"class_id", 8}, {"object_id", 9}}});
        auto field = [](const char *cl, const char *name, std::uint64_t cid, std::uint64_t oid,
                        const Json &value) {
            return Json{{"class_name", cl},
                        {"name", name},
                        {"class_id", cid},
                        {"object_id", oid},
                        {"path", std::string("/") + cl + "/" + name + "[0]"},
                        {"value", {{"decoded", value}}}};
        };
        Bytes tree_bytes;
        record(tree_bytes, "@#$", 31, type_body(31, {}, {}));
        const auto tree = complex_blob("BfaTree", tree_bytes);
        Json fields = Json::array({field("BPParaHandleAndBfaDataMap", "DataIdMap", 7, 1, decoded),
                                   field("BPParaBfaTree", "BfaTree", 8, 9, tree)});
        const auto before = fields;
        bind_bfa_definition_sources(fields, objects);
        const auto &links = fields[0]["value"]["decoded"]["definition_source_bindings"];
        check(links.size() == 3 && links[0]["entry_index"] == 0 &&
                  links[0]["target_status"] == "matched_bfa_field" &&
                  links[0]["target_field_index"] == 1 && links[0]["target_object_index"] == 1 &&
                  links[0]["handle_record_indices"] == Json({0}) &&
                  links[0]["handle_node_status"] == "unique_node_in_payload",
              "BPDataKey resolves the source object and BFA field before matching a handle within "
              "that payload");
        check(links[1]["target_field_index"] == links[0]["target_field_index"] &&
                  links[1]["target_status"] == "matched_bfa_field" &&
                  links[1]["handle_node_status"] == "not_in_payload" &&
                  links[1]["handle_record_indices"].empty() && !links[1].contains("records") &&
                  fields[1] == before[1] &&
                  fields[0]["value"]["decoded"]["entries"] == decoded["entries"],
              "several native handles share one source field without cloning its definition or "
              "inventing absent nodes");
        check(links[2]["target_status"] == "object_not_in_document" &&
                  links[2]["project_selection"] == "not_performed",
              "external or missing data key remains separate from local node identity");
        auto duplicate_objects = objects;
        duplicate_objects.push_back(objects[1]);
        auto ambiguous = before;
        bind_bfa_definition_sources(ambiguous, duplicate_objects);
        check(ambiguous[0]["value"]["decoded"]["definition_source_bindings"][0]["target_status"] ==
                      "ambiguous_object_identity" &&
                  !ambiguous[0]["value"]["decoded"]["definition_source_bindings"][0].contains(
                      "target_field_index"),
              "duplicate object identities do not select a BFA field by traversal order");
        auto duplicate_fields = before;
        duplicate_fields.push_back(before[1]);
        bind_bfa_definition_sources(duplicate_fields, objects);
        check(duplicate_fields[0]["value"]["decoded"]["definition_source_bindings"][0]
                              ["target_status"] == "ambiguous_bfa_field",
              "multiple binary fields with the same root property name remain ambiguous");
        auto mixed_properties = objects;
        mixed_properties[1]["root"]["children"] =
            Json::array({{{"name", "BfaTree"}, {"value", false}},
                         {{"name", "BfaTree"}, {"value", {{"binary_base64", ""}}}}});
        auto mixed_names = before;
        bind_bfa_definition_sources(mixed_names, mixed_properties);
        check(
            mixed_names[0]["value"]["decoded"]["definition_source_bindings"][0]["target_status"] ==
                "ambiguous_named_property",
            "a nonbinary duplicate root property prevents silently selecting the only binary "
            "field");
        auto nested = before;
        nested[1]["path"] = "/BPParaBfaTree/Nested[0]/BfaTree[0]";
        bind_bfa_definition_sources(nested, objects);
        check(nested[0]["value"]["decoded"]["definition_source_bindings"][0]["target_status"] ==
                  "bfa_binary_field_not_found",
              "native named-property lookup does not accidentally bind a nested same-named field");
        auto corrupt = before;
        corrupt[1]["value"].erase("decoded");
        bind_bfa_definition_sources(corrupt, objects);
        check(corrupt[0]["value"]["decoded"]["definition_source_bindings"][0]["target_status"] ==
                      "bfa_payload_unavailable" &&
                  corrupt[0]["value"]["decoded"]["definition_source_bindings"][0]
                         ["target_field_index"] == 1,
              "a source field remains identified when its BFA payload cannot be decoded");
        auto duplicate_nodes = before;
        duplicate_nodes[1]["value"]["decoded"]["records"].push_back(tree["records"][0]);
        bind_bfa_definition_sources(duplicate_nodes, objects);
        check(duplicate_nodes[0]["value"]["decoded"]["definition_source_bindings"][0]
                             ["handle_node_status"] == "ambiguous_node_id" &&
                  duplicate_nodes[0]["value"]["decoded"]["definition_source_bindings"][0]
                                 ["handle_record_indices"] == Json({0, 1}),
              "source tree identity can resolve while its handle node identity remains ambiguous");
        auto wrong_class = before;
        wrong_class[0]["class_name"] = "BPParaHandleAndEntityDataMap";
        bind_bfa_definition_sources(wrong_class, objects);
        check(!wrong_class[0]["value"]["decoded"].contains("definition_source_bindings"),
              "same-name entity map does not acquire BFA source associations");
        auto job = std::async(std::launch::async, [&] {
            auto result = before;
            bind_bfa_definition_sources(result, objects);
            return result;
        });
        check(job.get() == fields,
              "source bindings reuse only per-call indices and are deterministic concurrently");
    }
    return checks;
}
