#include <p3d/csg.hpp>
#include "internal.hpp"
#include "geometry.hpp"

namespace {
using namespace p3d;
template <class T> void put(Bytes &b, T v) {
    const auto *p = reinterpret_cast<const std::uint8_t *>(&v);
    b.insert(b.end(), p, p + sizeof(v));
}
void write(Bytes &b, std::size_t at, std::int32_t v) {
    std::memcpy(b.data() + at, &v, 4);
}
void block(Bytes &b, const Bytes &v) {
    put(b, std::int32_t(v.size()));
    b.insert(b.end(), v.begin(), v.end());
}
Bytes leaf() {
    // Native version, ignored header-offset word, four empty arrays,
    // operation Unknown, a 16-byte GUID, isOld = true, no children.
    return {0x90, 0xe2, 0x1e, 0, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,  0, 0,
            0,    4,    0,    0, 0,  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 0};
}
Bytes branch(unsigned code, const Bytes &left, const Bytes &right = {}) {
    auto b = leaf();
    write(b, 24, 2);
    b[45] = std::uint8_t(code);
    if (code)
        block(b, left);
    if (code != 0 && code != 1 && code != 2)
        block(b, right);
    return b;
}
Bytes line() {
    Bytes b(104);
    std::memcpy(b.data(), "bg0001fb", 8);
    write(b, 8, 12);
    b[12] = 8;
    b[14] = 12;
    b[16] = 4;
    b[18] = 8;
    write(b, 20, 8);
    b[24] = 1;
    write(b, 28, 20);
    b[32] = 6;
    b[34] = 56;
    b[36] = 8;
    write(b, 48, 16);
    for (unsigned i = 0; i < 6; ++i) {
        const double v = double(i + 1);
        std::memcpy(b.data() + 56 + i * 8, &v, 8);
    }
    return b;
}
Bytes archive(const Bytes &tree, const std::vector<Bytes> &geometries = {},
              const std::vector<Bytes> &caches = {}, bool transform = false) {
    Bytes b;
    put(b, std::int32_t(22));
    for (const auto *items : {&geometries, &caches}) {
        put(b, std::int32_t(items->size()));
        for (const auto &item : *items)
            block(b, item);
    }
    put(b, std::int32_t(transform));
    if (transform)
        for (unsigned i = 0; i < 12; ++i)
            put(b, double(i + 1));
    put(b, std::int32_t(36));
    block(b, tree);
    block(b, {0, 255, 3});
    block(b, {'a', 0, 255, 'z'});
    return b;
}
} // namespace
unsigned csg_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto invalid = [&](const Bytes &b) {
        try {
            decode_csg_bytes(b);
            return false;
        } catch (const std::exception &) {
            return true;
        }
    };
    auto b = archive(leaf(), {line(), line(), {1, 2, 3}}, {line()}, true);
    auto j = decode_csg_bytes(b);
    const auto &root = j.at("tree").at("nodes").at(0);
    check(j.at("geometries").size() == 3 && j.at("node_caches").size() == 1,
          "CSG keeps operand and cache lists separate without deduplication");
    check(j["geometries"][0]["geometry"]["geometry"]["_type"] == "LineSegment" &&
              j["geometries"][1]["source_index"] == 1 &&
              j["geometries"][2]["status"] == "not_decoded",
          "CSG BGFB decoding and local failures");
    check(j["transforms"][0]["matrix_3x4_rows"] ==
              Json({{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}}),
          "CSG 96-byte row matrix survives independently of geometry count");
    check(j["angle_tolerance"] == 36 && root["guid_hex"] == "000102030405060708090a0b0c0d0e0f" &&
              root["operation_name"] == "unknown" && root["is_old_value"] == 1,
          "CSG native node fields");
    check(j["mark_bytes"]["raw_base64"] == base64(Bytes{'a', 0, 255, 'z'}) &&
              j["attach_data"]["raw_base64"] == base64(Bytes{0, 255, 3}) &&
              j["raw_base64"] == base64(b),
          "CSG binary mark and extension preserve NUL and encoding");
    auto n = leaf();
    // Source index lists retain order, duplicates, negative and out-of-range values.
    Bytes lists;
    for (const auto &a :
         {std::vector<int>{2, 0, 2, -1}, std::vector<int>{0, 99}, std::vector<int>{1, 0}}) {
        put(lists, std::int32_t(a.size()));
        for (auto x : a)
            put(lists, std::int32_t(x));
    }
    put(lists, std::int32_t(1));
    lists.insert(lists.end(), 16, 0xab);
    n.erase(n.begin() + 8, n.begin() + 24);
    n.insert(n.begin() + 8, lists.begin(), lists.end());
    j = decode_csg_bytes(archive(n));
    auto q = j["tree"]["nodes"][0];
    check(q["geometry_indices"] == Json({2, 0, 2, -1}) && q["cache_indices"] == Json({0, 99}) &&
              q["matrix_indices"] == Json({1, 0}) &&
              q["geometry_guids_hex"][0] == "abababababababababababababababab",
          "CSG source associations are not sorted, zipped or rewritten");
    j = decode_csg_bytes(archive(n, {line(), line(), line()}, {line()}, true));
    q = j["tree"]["nodes"][0]["references"];
    check(q["geometry_indices"][0]["target_index"] == 2 &&
              q["geometry_indices"][2]["target_index"] == 2 &&
              q["geometry_indices"][3]["status"] == "out_of_range" &&
              q["cache_indices"][1]["target_index"].is_null() &&
              q["matrix_indices"][0]["status"] == "out_of_range" &&
              q["matrix_indices"][1]["target_index"] == 0,
          "CSG native zero-based associations retain repeated targets and invalid source values");
    for (unsigned code : {1u, 2u, 3u, 4u, 255u}) {
        j = decode_csg_bytes(archive(branch(code, leaf(), leaf())));
        const auto &t = j["tree"];
        check(t["status"] == "decoded" && t["nodes"].size() == (code < 3 ? 2u : 3u),
              "CSG native child selector uses ordered both-children fallback");
        check(t["nodes"][0][code == 2 ? "right_index" : "left_index"] == 1 &&
                  (code < 3 || t["nodes"][0]["right_index"] == 2),
              "CSG left/right node identity");
    }
    auto bad_child = leaf();
    bad_child.resize(8);
    j = decode_csg_bytes(archive(branch(3, bad_child, leaf())));
    check(j["tree"]["status"] == "partial" && j["tree"]["nodes"][1]["status"] == "invalid" &&
              j["tree"]["nodes"][2]["status"] == "decoded",
          "CSG bad child cannot consume sibling bytes");
    for (std::int32_t v : {0, 0x1ee28f, 0x1ee291}) {
        n = leaf();
        write(n, 0, v);
        j = decode_csg_bytes(archive(n));
        check(j["tree"]["nodes"][0]["status"] == "unsupported_version" &&
                  j["mark_bytes"]["source_bytes"] == 4,
              "CSG unknown node version preserves archive suffix");
    }
    n = leaf();
    write(n, 8, -1);
    check(decode_csg_bytes(archive(n))["tree"]["nodes"][0]["status"] == "invalid",
          "CSG rejects negative array count");
    n = leaf();
    write(n, 8, 0x7fffffff);
    check(decode_csg_bytes(archive(n))["tree"]["nodes"][0]["status"] == "invalid",
          "CSG rejects excessive array count");
    n = leaf();
    write(n, 4, 0x7fffffff);
    check(decode_csg_bytes(archive(n))["tree"]["status"] == "decoded",
          "CSG exact version ignores header offset word");
    n = leaf();
    n.push_back(77);
    check(decode_csg_bytes(archive(n))["tree"]["nodes"][0]["trailing_bytes"]["source_bytes"] == 1,
          "CSG unassigned node suffix remains located");
    j = decode_csg_bytes(archive(branch(3, {}, leaf())));
    check(j["tree"]["nodes"].size() == 2 && j["tree"]["nodes"][0]["left_index"].is_null(),
          "CSG empty child restores no node");
    j = decode_csg_bytes(archive({}));
    check(j["tree"]["nodes"].empty() && j["tree"]["root_index"].is_null(),
          "CSG absent root keeps independent fields");
    b = archive(branch(3, leaf(), leaf()));
    j = decode_csg_bytes(b, 1);
    check(j["tree"]["nodes"][1]["status"] == "node_limit" &&
              j["tree"]["nodes"][2]["status"] == "node_limit",
          "CSG node budget retains deferred child ranges");
    n = leaf();
    for (unsigned i = 0; i < 600; ++i)
        n = branch(1, n);
    j = decode_csg_bytes(archive(n));
    check(j["tree"]["nodes"].size() == 601 && j["tree"]["status"] == "decoded",
          "CSG deep tree decoding uses no recursion");
    b = archive(leaf());
    for (std::size_t length = 0; length < b.size(); ++length)
        check(invalid(slice(b, 0, length)),
              "CSG all outer truncations rejected before suffix loss");
    for (auto at : {4u, 8u, 12u, 20u, 70u, 77u}) {
        auto bad = b;
        write(bad, at, -1);
        check(invalid(bad), "CSG negative outer counts and lengths rejected");
    }
    auto tail = b;
    tail.push_back(99);
    check(decode_csg_bytes(tail)["trailing_bytes"]["source_bytes"] == 1,
          "CSG future archive suffix retained");
    j = command_fields(58, b);
    check(j["name"] == "csg_tree" && j["tree"]["nodes"].size() == 1 &&
              j["boolean_evaluation_status"] == "not_evaluated",
          "command 58 decodes CSG without inventing Boolean mesh");
    const auto geometry = reconstruct(
        Json::array({{{"op", 58}, {"offset", 9}, {"body", rawbytes(b)}, {"decoded", j}}}),
        Tessellation{});
    check(geometry.vertices.empty() && geometry.faces.empty() && geometry.unknown.size() == 1 &&
              geometry.unknown[0]["csg"]["tree"]["nodes"].size() == 1 &&
              geometry.unknown[0].contains("placement_matrix") &&
              geometry.unknown[0].contains("style"),
          "CSG reconstruction preserves source and placement without promoting operands to mesh");
    return checks;
}
