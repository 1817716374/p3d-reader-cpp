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
    auto ui = [](const Json &r) { return native_tree_ui_parameters(r); };
    auto fold = [](const Json &r) { return native_tree_fold_input(r); };
    q = ui(Json::array());
    check(q["image_index"] == -1 && q["visibility"] == true && fold(Json::array())["value"] == true,
          "UI resource defaults are no image, visible, and folded");
    q = ui(Json::array({"prefixTreeNode@ImgIdx: +12suffix"}));
    check(q["image_index"] == 12 && q["visibility"] == true,
          "UI image parses a signed decimal prefix from its entire suffix");
    check(ui(Json::array({"TreeNode@ImgIdx:-2"}))["image_index"] == -2 &&
              ui(Json::array({"TreeNode@ImgIdx:-2147483648"}))["image_index"] == INT32_MIN,
          "stored negative image values are retained without a fabricated validity filter");
    q = ui(Json::array({"TreeNode@Visible:01"}));
    check(q["visibility"] == true &&
              visible(Json::array({"TreeNode@Visible:01"}))["value"] == false,
          "full suffix UI visibility differs from public one-code-unit query");
    check(
        ui(Json::array({"TreeNode@Visible:-1"}))["visibility"] == true &&
            visible(Json::array({"TreeNode@Visible:-1"}))["status"] == "rejected",
        "UI suffix permits a sign while the public single-code-unit query cannot consume it alone");
    check(ui(Json::array({"TreeNode@Visible: -0tail"}))["visibility"] == false,
          "signed zero suffix remains false after accepted whitespace and sign");
    q = ui(Json::array({"TreeNode@Visible:x TreeNode@ImgIdx:7"}));
    check(q["status"] == "rejected" && q["matches"].size() == 2 &&
              q["matches"][0]["marker"] == "TreeNode@ImgIdx:" &&
              q["matches"][0]["parsed_integer"] == 7 && !q.contains("image_index"),
          "each UI resource parses image before visibility regardless of textual marker order");
    q = ui(Json::array({"TreeNode@Visible:1 TreeNode@ImgIdx:x"}));
    check(q["status"] == "rejected" && q["matches"].size() == 1 &&
              q["native_exception"] == "std::invalid_argument",
          "invalid image suffix prevents evaluating visibility later in the same native iteration");
    for (const auto *suffix : {"2147483648", "-2147483649", "9223372036854775808"}) {
        q = ui(Json::array({std::string("TreeNode@ImgIdx:") + suffix}));
        check(
            q["status"] == "rejected" && q["native_exception"] == "std::out_of_range" &&
                !q.contains("image_index"),
            "native signed 32 bit stoi overflow is an exception rather than a saturated parameter");
    }
    check(ui(Json::array({"TreeNode@ImgIdx:"}))["native_exception"] == "std::invalid_argument" &&
              ui(Json::array({"TreeNode@Visible:garbage"}))["status"] == "rejected",
          "empty or nonnumeric suffixes do not silently adopt UI defaults");
    q = ui(Json::array({"TreeNode@ImgIdx:4 TreeNode@ImgIdx:7", "TreeNode@Visible:0",
                        "TreeNode@ImgIdx:9", "TreeNode@Visible:1"}));
    check(q["image_index"] == 9 && q["visibility"] == true &&
              q["matches"][0]["parsed_integer"] == 4,
          "first marker per kind in a resource is read and later resources override independently");
    q = ui(Json::array(
        {"\xf0\x9f\x98\x80TreeNode@ImgIdx:42", "TreeNode@Visible:\xe3\x80\x80-\xd9\xa1\xd9\xa2"}));
    check(q["image_index"] == 42 && q["visibility"] == true &&
              q["matches"][0]["marker_byte_offset"] == 4,
          "UI numeric conversion retains native Unicode rules and explicit UTF8 source byte "
          "positions");
    q = ui(Json::array({std::string("TreeNode@Visible:0\0TreeNode@ImgIdx:3", 34)}));
    check(q["image_index"] == -1 && q["visibility"] == false,
          "node loading truncates NUL before searching either UI marker");
    check(ui(nullptr)["status"] == "not_evaluated" &&
              ui(Json::array({12}))["status"] == "not_evaluated",
          "malformed UI resource input does not fabricate complete parameters");
    check(fold(Json::array({"prefixTreeNode@Fold:0suffix"}))["value"] == false,
          "fold resource stage consumes the marker after an arbitrary prefix");
    check(fold(Json::array({"TreeNode@Fold:0", "TreeNode@Fold:2"}))["value"] == true &&
              fold(Json::array({"TreeNode@Fold:1", "TreeNode@Fold:0"}))["value"] == false,
          "last matching fold resource overrides the default or previous stored value");
    check(fold(Json::array({"TreeNode@Fold:01 TreeNode@Fold:1"}))["value"] == false,
          "fold uses one code unit and only the first marker of each resource");
    check(fold(Json::array({"TreeNode@Fold:\xef\xbc\x90"}))["value"] == false &&
              fold(Json::array({"TreeNode@Fold:\xd9\xa2"}))["value"] == true,
          "fold applies native wide digit conversion to its one-code-unit value");
    for (const auto *suffix : {"", "-1", " 1", "x"}) {
        q = fold(Json::array({std::string("TreeNode@Fold:") + suffix, "TreeNode@Fold:1"}));
        check(q["status"] == "rejected" && !q.contains("value"),
              "fold conversion failure is not replaced by a later valid resource");
    }
    check(ui(Json::array({"TreeNode@Fold:x"}))["status"] == "decoded" &&
              fold(Json::array({"TreeNode@Visible:x"}))["value"] == true,
          "independent native initialization stages do not borrow each other's marker failures");
    queries = native_tree_node_queries(
        {{"NodeType", 11}, {"ResourceRefers", {"TreeNode@ImgIdx:3", "TreeNode@Fold:0"}}}, {});
    check(queries["ui_parameters_input"]["image_index"] == 3 &&
              queries["fold_resource_input"]["value"] == false &&
              queries["visibility_self"]["value"] == true,
          "source graph node queries expose separate UI parameters, fold input and public "
          "visibility");
    return checks;
}
