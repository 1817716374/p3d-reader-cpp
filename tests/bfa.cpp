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
              j["records"][1]["payload_status"] == "not_decoded" &&
              j["records"][1]["unassigned_suffix_hex"] == "8192a3",
          "native driven-node framing is recognized without guessed payload semantics");
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
    return checks;
}
