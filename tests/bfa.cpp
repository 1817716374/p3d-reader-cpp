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
    return checks;
}
