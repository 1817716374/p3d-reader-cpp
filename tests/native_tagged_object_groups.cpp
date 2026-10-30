#include "internal.hpp"

unsigned native_tagged_object_group_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        ++checks;
        require(ok, message);
    };

    const Json source_group = {
        {"group_tag_word", 255},
        {"entry_count", 4},
        {"entries", Json::array({{{"object_id", 42}, {"entry_type", 13}},
                                 {{"object_id", 42}, {"entry_type", 13}},
                                 {{"object_id", 77}, {"entry_type", 13}},
                                 {{"object_id", 99}, {"entry_type", 13}}})}};
    const Json graphics = Json::array(
        {{{"id", 1161},
          {"offset", 100},
          {"stream", Json::array({"P3D-SM", "model", "P3D-SMG", "graphics"})},
          {"attributes", Json::array({{{"group", 0},
                                       {"key", 22909},
                                       {"decoded", {{"encoding", "key_22909_tagged_object_groups"},
                                                    {"groups", Json::array({source_group})}}}}})}}});
    const Json non_group_graphics = Json::array(
        {{{"id", 777},
          {"attributes", Json::array({{{"group", 0},
                                       {"key", 22909},
                                       {"decoded", {{"encoding", "key_22909_fixed_u32_words"},
                                                    {"words", Json::array({6, 1, 2, 3, 4, 5, 6})}}}}})}}});
    const auto all_graphics = Json::array({graphics[0], non_group_graphics[0]});
    const Json objects = Json::array(
        {{{"class_id", 1120},
          {"object_id", 1161},
          {"root", {{"name", "PBImportElementProxy"}}},
          {"stream", "proxy"},
          {"offset", 1},
          {"length", 20}},
         {{"class_id", 1120},
          {"object_id", 42},
          {"root", {{"name", "PBImportElementProxy"}}},
          {"stream", "proxy"},
          {"offset", 2},
          {"length", 20}},
         {{"class_id", 1091},
          {"object_id", 42},
          {"root", {{"name", "BimBaseTreeNodeData"}}},
          {"stream", "tree"},
          {"offset", 3},
          {"length", 20}},
         {{"class_id", 1120},
          {"object_id", 77},
          {"root", {{"name", "PBImportElementProxy"}}},
          {"stream", "proxy-a"},
          {"offset", 4},
          {"length", 20}},
         {{"class_id", 1120},
          {"object_id", 77},
          {"root", {{"name", "PBImportElementProxy"}}},
          {"stream", "proxy-b"},
          {"offset", 5},
          {"length", 20}}});

    const auto result = build_native_tagged_object_groups(all_graphics, objects);
    check(result.at("status") == "decoded", "22909 record is exposed");
    const auto &record = result.at("records").at(0);
    check(record.at("owner_object_id_candidate").at("candidate_count") == 1,
          "graphics owner is matched only by numeric object id");
    const auto &members = record.at("groups").at(0).at("members");
    check(members.size() == 4, "all source entries and their order are retained");
    check(members[0].at("object_id_candidate").at("status") ==
              "ambiguous_document_object_id" &&
              members[0].at("object_id_candidate").at("candidate_count") == 2,
          "same numeric id in distinct schema classes stays ambiguous");
    check(members[0].at("source_entry") == members[1].at("source_entry"),
          "duplicate native entries are not collapsed");
    check(members[2].at("object_id_candidate").at("status") ==
              "single_document_object_id_candidate" &&
              members[2].at("object_id_candidate").at("candidates")[0].at("records").size() == 2,
          "same-class copies remain one object candidate with both source records");
    check(members[3].at("object_id_candidate").at("status") ==
              "no_document_object_id_candidate",
          "unmatched ids are preserved as unresolved candidates");
    check(record.at("tag_semantics") == "unresolved" &&
              record.at("entry_type_semantics") == "unresolved",
          "reference lookup does not assign undocumented native semantics");
    check(result.at("records").at(1).at("status") ==
              "no_tagged_group_projection_for_payload_encoding",
          "recognized fixed-word form is retained without being mislabeled as unknown");
    return checks;
}
