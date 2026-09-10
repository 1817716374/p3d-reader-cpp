#include "internal.hpp"
unsigned material_resource_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    for (const auto &entry : std::vector<std::pair<std::string, std::string>>{
             {"", ""},
             {"relative/a.jpg", "relative/a.jpg"},
             {u8"逻辑:木纹.pma", u8"逻辑:木纹.pma"},
             {"LAYEREDPROCEDURALNAME", "LAYEREDPROCEDURALNAME"},
             {"a<1>x", "a<1>x"},
             {"a<+0012tail>x", "a<12>x"},
             {"a<0>wood.pma", "a"},
             {"wood.pma<0>image.jpg", "wood.pma"},
             {"a<-0>x", "a"},
             {"a<-2>x", "a<>x"},
             {"a<>x", "a<>x"},
             {"a< \t\n\r\f\v+2 trailing>x", "a<2>x"},
             {"a<2147483647>x", "a<2147483647>x"},
             {"a<-2147483648>x", "a<>x"},
             {"a<0x20>x", "a"},
             {"a<1.5>x", "a<1>x"},
             {"a<1e2>x", "a<1>x"},
             {"a<1>tail<2>end", "a<1>tail<2>end"},
             {"<2>tail", ""},
             {"<>tail", ""},
             {"<0>tail", ""},
             {"http:wood.pma<2>suffix", "http:wood.pma"},
             {"http:wood.pma<-2>suffix", "http:wood.pma"},
             {"https:wood.pma<2>suffix", "https:wood.pma<2>suffix"},
             {"HTTP:wood.pma<2>suffix", "HTTP:wood.pma<2>suffix"},
             {"http:wood.pma<bad>suffix", "http:wood.pma<bad>suffix"},
             {"a< >x", "a< >x"},
             {"a<+>x", "a<+>x"},
             {"a<+-1>x", "a<+-1>x"},
             {"a<--1>x", "a<--1>x"},
             {"a<<1>>x", "a<<1>>x"},
             {"a<bad><1>x", "a<bad><1>x"},
             {"a<1", "a<1"},
             {"a>1", "a>1"},
             {u8"木😀<01>纹理", u8"木😀<1>纹理"}}) {
        auto parsed = material_resource_reference(entry.first);
        check(parsed["status"] == "decoded" && parsed["value"] == entry.second &&
                  parsed["source_value"] == entry.first &&
                  parsed["lookup_status"] == "not_performed",
              "default resource reference normalization preserves source and native marker rules");
    }
    for (const auto &s : {"a<2147483648>x", "a<-2147483649>x", u8"a<　1>x"}) {
        const auto parsed = material_resource_reference(s);
        check(parsed["status"] == "unresolved" && parsed["value"].is_null(),
              "range or locale dependent marker does not invent a resource reference");
    }
    for (const auto &source :
         {Json(2), Json(), Json(std::string("a\0b", 3)), Json(std::string("a\xff", 2))}) {
        const auto parsed = material_resource_reference(source);
        check(parsed["status"] == "invalid" && parsed["value"].is_null(),
              "invalid resource input cannot accidentally select a shader branch");
    }
    auto node = [](const std::string &tag, const Json &attrs, const Json &children = Json::array(),
                   const std::string &text = "") {
        return Json{{"tag", tag}, {"attributes", attrs}, {"children", children}, {"text", text}};
    };
    auto material = [&](const Json &attrs, const Json &children) {
        return node("Material", {{"material_version", "9"}},
                    Json::array({node("Map", attrs, children)}));
    };
    auto children = Json::array({node("M633", {{"M634", "1"}}), node("M541", {{"M542", "3"}}),
                                 node("Data", Json::object(), Json::array(), "x_brightness .5")});
    for (const auto &name : {"wood.pma<0>image.jpg", "http:wood.pma<2>image.jpg"}) {
        const auto source = material({{"Type", "1"}, {"Filename", name}, {"layer", "0"}}, children);
        const auto original = source;
        const auto settings = material_settings(source);
        const auto &map = settings["maps"][0];
        const auto &path = map["reader_path"];
        check(source == original && path["source_filename"]["extension"] == "jpg" &&
                  path["first_resource_reference"]["path_parts"]["extension"] == "pma" &&
                  path["single_provider_type_before_preset"]["reader_value"] == 2 &&
                  path["provider_dispatch"]["preset"]["name"] == "wood" &&
                  path["provider_dispatch"]["provider_type_after_dispatch"] == 4 &&
                  map["procedures"]["reader_applicability"]["status"] == "skipped" &&
                  map["legacy_parameters"]["status"] == "decoded",
              "Map branches use transformed reference while source XML and source gate remain "
              "intact");
    }
    for (const auto selector : {3, 4, 6, 7}) {
        auto settings = material_settings(material(
            {{"Type", "1"}, {"Filename", "image.jpg"}, {"layer", std::to_string(selector)}},
            children));
        const auto &map = settings["maps"][0];
        check(map["procedures"]["reader_applicability"]["status"] ==
                      (selector == 7 ? "skipped" : "read") &&
                  map["replicators"]["reader_applicability"]["status"] ==
                      (selector == 7 ? "read" : "skipped"),
              "ordinary present filename resolves procedure versus replicator invocation");
    }
    auto settings = material_settings(
        material({{"Type", "1"}, {"Filename", "layers.pma<0>image.jpg"}}, children));
    check(settings["maps"][0]["reader_path"]["branch"] == "single_provider" &&
              settings["maps"][0]["reader_path"]["provider_dispatch"]["branch"] ==
                  "pma_user_data" &&
              settings["maps"][0]["legacy_parameters"]["entries"].size() == children.size(),
          "transformed layers.pma does not re-enter source-only layer collection gate");
    settings = material_settings(
        material({{"Type", "1"}, {"Filename", "image<2147483648>.jpg"}, {"layer", "3"}}, children));
    check(settings["maps"][0]["procedures"]["reader_applicability"]["status"] == "unresolved",
          "unresolved resource conversion keeps dependent shader selection unresolved");
    settings = material_settings(material({{"Type", "1"}, {"Filename", ""}}, children));
    check(settings["maps"][0]["reader_path"]["first_resource_reference"]["resource_created"] ==
              true,
          "present empty reference is distinguished from omitted resource creation");
    settings = material_settings(material({{"Type", "1"}}, children));
    check(settings["maps"][0]["reader_path"]["first_resource_reference"]["resource_created"] ==
              false,
          "missing filename preserves empty native resource list");
    return checks;
}
