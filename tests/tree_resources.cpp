#include "tree_entity_refs.hpp"
#include "tree_resources.hpp"
#include <stdexcept>
unsigned tree_resource_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        if (!ok)
            throw std::runtime_error(why);
    };
    auto property = [](const Json &value) {
        return native_tree_property_bytes(Json::array({"", value}));
    };
    auto bytes = [](const Json &q) {
        return unbase64(q.at("payload").at("base64").get<std::string>());
    };
    auto visible = [](const Json &resources) { return native_tree_visibility(resources); };
    std::string text;
    for (std::uint64_t sentinel :
         {UINT64_C(255), UINT64_C(65535), UINT64_C(4294967295), UINT64_MAX}) {
        Json value = {{"null_string_index", sentinel}};
        check(native_tree_resource_text(value, text) && text.empty() &&
                  property(value)["status"] == "empty",
              "native DEX null string indices become empty loaded strings");
    }
    check(!native_tree_resource_text({{"null_string_index", -1}}, text) &&
              !native_tree_resource_text({{"null_string_index", 254}}, text),
          "only recognized unsigned sentinel values normalize as null strings");
    check(!native_tree_resource_text(nullptr, text) && !native_tree_resource_text(12, text),
          "untyped nulls and numeric values are not silently converted to strings");
    auto q = property("BAAAAAAAAAAAAAAA");
    auto expected = Bytes(12, 0);
    expected[0] = 1;
    check(q["status"] == "decoded" && bytes(q) == expected && q["native_byte_count"] == 12,
          "stored low-sextet-first binary sample decodes with its native byte order");
    check(bytes(property("BA==")) == Bytes({1}) &&
              bytes(property("QIDB")) == Bytes({0x10, 0x32, 4}),
          "native encoding differs from standard base64 sextet order");
    check(bytes(property("////")) == Bytes({255, 255, 255}) &&
              bytes(property("AA/=")) == Bytes({0, 240}),
          "full groups and single padding emit exactly three or two bytes");
    check(bytes(property("AA=X")) == Bytes({0}) && bytes(property("AA=\xc2\xa0")) == Bytes({0}),
          "native decoder ignores remaining code units after accepted first padding");
    check(property("AAA")["status"] == "rejected" && property("A===")["status"] == "rejected",
          "encoded input requires groups of four and sufficient digits before padding");
    q = property("AAAA!!!!");
    check(q["status"] == "rejected" && q["native_byte_count"] == 0 && !q.contains("payload"),
          "byte getter does not publish partial decoder output after later failure");
    check(property("AA=\xf0\x9f\x98\x80")["status"] == "rejected",
          "encoded length counts UTF16 units rather than UTF8 bytes or Unicode scalar values");
    check(property(std::string("BA==\0junk", 9))["native_byte_count"] == 1 &&
              property(std::string("\0BA==", 5))["status"] == "empty",
          "loaded property resource is truncated at NUL before its empty check");
    check(property("TreeNode@Visible:1")["status"] == "rejected" &&
              visible(Json::array({"", "TreeNode@Visible:1"}))["value"] == true,
          "one resource may be a visibility marker while failing the independent byte getter");
    check(property(1)["status"] == "not_evaluated" &&
              property(std::string("\xff", 1))["status"] == "not_evaluated",
          "malformed source text is distinguished from native decoder rejection");
    check(visible(Json::array())["value"] == true &&
              visible(Json::array({{{"null_string_index", 255}}}))["value"] == true,
          "existing node visibility defaults true with empty or null-string resources");
    check(visible(Json::array({"prefixTreeNode@Visible:0suffix"}))["value"] == false,
          "public visibility query searches the marker anywhere in the resource");
    check(visible(Json::array({"TreeNode@Visible:0", "TreeNode@Visible:2"}))["value"] == true &&
              visible(Json::array({"TreeNode@Visible:2", "TreeNode@Visible:0"}))["value"] == false,
          "last matching resource overrides prior visibility in stored order");
    check(visible(Json::array({"TreeNode@Visible:0 TreeNode@Visible:1"}))["value"] == false,
          "only first marker in each individual resource is consumed");
    check(visible(Json::array({"TreeNode@Visible:01"}))["value"] == false &&
              visible(Json::array({"TreeNode@Visible:10"}))["value"] == true,
          "visibility conversion consumes exactly one UTF16 code unit rather than a suffix number");
    check(visible(Json::array({"TreeNode@Visible:\xef\xbc\x90"}))["value"] == false &&
              visible(Json::array({"TreeNode@Visible:\xd9\xa2"}))["value"] == true,
          "single-code-unit visibility conversion uses native wide decimal digits");
    for (const auto *suffix : {"", "-1", "+1", " 1", "x", "\xf0\x9f\x98\x80"}) {
        q = visible(Json::array({std::string("TreeNode@Visible:") + suffix, "TreeNode@Visible:1"}));
        check(q["status"] == "rejected" && q["native_exception"] == "std::invalid_argument" &&
                  !q.contains("value"),
              "native stoi failure stops visibility query before any later resource override");
    }
    check(visible(Json::array({"treenode@Visible:0"}))["value"] == true &&
              visible(Json::array({std::string("\0TreeNode@Visible:0", 19)}))["value"] == true,
          "marker is case-sensitive and loaded C-string truncation precedes searching");
    check(visible(nullptr)["status"] == "not_evaluated" &&
              visible(Json::array({12}))["status"] == "not_evaluated",
          "unsupported source resource types do not silently default visibility");
    auto queries = native_tree_node_queries(
        {{"NodeType", 7}, {"ResourceRefers", {{{"null_string_index", 255}}}}}, {});
    check(queries["model_self"]["status"] == "empty" &&
              queries["property_bytes_self"]["status"] == "empty" &&
              queries["visibility_self"]["value"] == true,
          "all node query consumers share native null-string normalization");
    queries = native_tree_node_queries(
        {{"NodeType", 17}, {"ResourceRefers", {std::string("\0tail", 5)}}}, {});
    check(queries["domain_self"]["value"] == -1,
          "domain query empty default is applied after loading truncates the source string");
    Json refs = Json::array();
    q = append_tree_entity_references(Json::array({"", "", "", {{"null_string_index", 65535}}}),
                                      "node", {}, Json::object(), refs);
    check(q["status"] == "decoded" && refs.empty(),
          "null-string entity list is known empty instead of an unsupported resource");
    return checks;
}
