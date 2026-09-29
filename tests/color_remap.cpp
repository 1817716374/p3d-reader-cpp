#include "internal.hpp"
#include "color_remap_oracle.hpp"
#include <p3d/color_remap.hpp>
#include <codecvt>
#include <locale>

unsigned color_remap_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) { ++checks; require(ok, message); };
    std::string fixture;
    for (const auto *part : color_remap_oracle_parts) fixture += part;
    const auto oracle = Json::parse(fixture);
    std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> utf;
    auto decode_cache = [&](const Json &j) {
        NativeExtendedColorCache c;
        c.dirty = j.at("dirty").get<bool>();
        for (const auto &e : j.at("entries")) {
            NativeExtendedColor entry{e.at("rgb").get<NativeRgb>(), {}};
            if (!e.at("book_name").is_null()) {
                const auto b = e.at("book_name").at("book").get<std::string>();
                const auto n = e.at("book_name").at("name").get<std::string>();
                entry.book_name = NativeColorBookName{utf.from_bytes(b),utf.from_bytes(n)};
            }
            c.entries.push_back(std::move(entry));
        }
        return c;
    };
    auto snapshot = [&](const NativeExtendedColorCache &c) {
        Json entries = Json::array();
        for (const auto &e : c.entries) {
            Json pair;
            if (e.book_name) pair = {{"book",utf.to_bytes(e.book_name->book)},
                                    {"name",utf.to_bytes(e.book_name->name)}};
            entries.push_back({{"rgb",e.rgb},{"book_name",pair}});
        }
        return Json{{"entries",entries},{"dirty",c.dirty}};
    };
    auto run = [&](const Json &scenario, bool background) {
        const auto n = scenario.at("contexts").size();
        std::vector<NativePackedPalette> palettes(n);
        std::vector<NativeExtendedColorCache> caches(n);
        std::vector<NativeSystemColorContext> contexts(n);
        for (std::size_t i=0; i<n; ++i) {
            const auto &input = scenario.at("contexts")[i];
            palettes[i] = oracle.at("palettes").at(input.at("palette").get<std::string>()).get<NativePackedPalette>();
            caches[i] = decode_cache(input);
            contexts[i] = {i+1,&palettes[i],&caches[i]};
        }
        NativeColorRemapCache mapping;
        for (const auto &q : scenario.at("queries")) {
            if (q.contains("mutation") && !q.at("mutation").is_null()) {
                const auto &m = q.at("mutation");
                palettes.at(m[0].get<std::size_t>()) = oracle.at("palettes").at(m[1].get<std::string>()).get<NativePackedPalette>();
            }
            const auto s = q.at("source").get<std::size_t>(), t = q.at("target").get<std::size_t>();
            if (background) {
                auto r = project_native_system_background(q.at("id").get<std::uint32_t>(),
                    q.at("enabled").get<bool>(),true,q.at("initial_rgb").get<NativeRgb>(),contexts[s],contexts[t]);
                check(r.rgb && Json(*r.rgb)==q.at("rgb") && r.changed==(q.at("rgb")!=q.at("initial_rgb")),
                      "background projection matches original daaf0 plus 4b6570 without model refresh");
            } else {
                const auto r = remap_native_system_color(q.at("id").get<std::uint32_t>(), contexts[s],contexts[t],
                    q.at("mapping_enabled").get<bool>() ? &mapping : nullptr,q.at("bypass").get<bool>());
                check(r.color_id && *r.color_id==q.at("color_id").get<std::uint32_t>(),
                      "system color remap matches original DLL including mask, reserved IDs and stale cache");
                const auto &m = q.at("mapping");
                check(m.is_null() ? !mapping.initialized : mapping.initialized &&
                    mapping.source_identity==m.at("source").get<std::uint64_t>()+1 &&
                    mapping.target_identity==m.at("target").get<std::uint64_t>()+1 &&
                    mapping.palettes_differ==m.at("palettes_differ").get<bool>() && Json(mapping.indices)==m.at("indices"),
                    "persistent mapping cache matches native identity, first-255 comparison and lazy entries");
            }
            check(snapshot(caches[t])==q.at("target_state"),
                  "target extended entries, duplicate selection, optional names and dirty state match original DLL");
        }
    };
    for (const auto &s : oracle.at("scenarios")) run(s,false);
    for (const auto &s : oracle.at("backgrounds")) run(s,true);
    NativeColorRemapCache mapping;
    NativeSystemColorContext source{1}, target{2};
    check(!remap_native_system_color(1,source,target,&mapping).color_id && !mapping.initialized,
          "unknown palette cannot create a fabricated mapping cache");
    check(!remap_native_system_color(0x100,source,target,&mapping).color_id && !mapping.initialized,
          "unknown extended source remains unresolved without cache mutation");
    check(remap_native_system_color(0xffffff00,source,target,&mapping).color_id==0xffffff00 && !mapping.initialized,
          "reserved colors do not require resource contents");
    check(remap_native_system_color(1,source,target,nullptr).color_id==1 &&
              remap_native_system_color(1,source,target,&mapping,true).color_id==1,
          "disabled low-byte mapping preserves plain palette IDs");
    NativeExtendedColorCache empty;
    source.extended_colors=&empty;
    check(remap_native_system_color(0x10000001,source,target,&mapping).color_id==0x10000001 && !mapping.initialized,
          "nonzero high bits with masked slot zero preserve the original ID");
    check(!lookup_native_background_color(0x10000001,source).rgb &&
              lookup_native_background_color(0x10000001,source).source==NativeBackgroundColorSource::InvalidExtendedSlot,
          "invalid extended background ID never falls back to its low-byte palette entry");
    check(project_native_system_background(0x100,true,false,{1,2,3},source,target).rgb==NativeRgb{1,2,3},
          "absent view bypasses background resource resolution");
    auto palette = oracle.at("palettes").at("gray").get<NativePackedPalette>();
    source.palette=target.palette=&palette;
    source.identity=0;
    check(!remap_native_system_color(1,source,target,&mapping).color_id && !mapping.initialized,
          "unknown context identity does not imply same file or reset a cache");
    return checks;
}
