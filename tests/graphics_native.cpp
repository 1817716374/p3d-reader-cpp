#include "internal.hpp"

unsigned graphics_native_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto write = [](Bytes &bytes, std::size_t offset, std::uint64_t value, unsigned size) {
        for (unsigned i = 0; i < size; ++i)
            bytes.at(offset + i) = std::uint8_t(value >> (8 * i));
    };
    auto bgfb = [&](unsigned tag) {
        Bytes bytes(40);
        const std::string magic = "bg0001fb";
        std::copy(magic.begin(), magic.end(), bytes.begin());
        write(bytes, 8, 12, 4); // root at 20
        write(bytes, 12, 8, 2);
        write(bytes, 14, 12, 2);
        write(bytes, 16, 4, 2);
        write(bytes, 18, 8, 2);
        write(bytes, 20, 8, 4);
        write(bytes, 24, tag, 1);
        write(bytes, 28, 8, 4); // child deliberately not constructed by this test
        return bytes;
    };
    auto packet = [&](std::int32_t type, Bytes bytes) {
        Bytes result(36);
        write(result, 4, std::uint32_t(type), 4);
        write(result, 28, bytes.size(), 8);
        result.insert(result.end(), bytes.begin(), bytes.end());
        return result;
    };
    auto with_project = [](const Bytes &bytes) {
        return graphics_entry_native_input(bytes).at("with_project");
    };
    auto result = with_project(packet(6, bgfb(13)));
    check(result.at("status") == "rejected" && result.at("bgfb_geometry_type") == 13,
          "valid BGFB root shape with mesh tag cannot satisfy the solid reader");
    result = with_project(packet(3, bgfb(13)));
    check(result.at("status") == "not_evaluated" && result.at("root_dispatch") == "selected",
          "matching mesh tag selects a reader but is not proof of child construction");
    result = with_project(packet(2, bgfb(5)));
    check(result.at("root_dispatch") == "selected", "curve collection selects its own root tag");
    result = with_project(packet(5, bgfb(14)));
    check(result.at("root_dispatch") == "selected", "B-spline surface selects its own root tag");
    for (const auto tag : {6u, 12u, 20u, 21u})
        check(with_project(packet(6, bgfb(tag))).at("root_dispatch") == "selected",
              "solid dispatch includes primitive and guided-solid endpoints");
    check(with_project(packet(6, bgfb(19))).at("status") == "rejected",
          "gap before custom guided-solid tags is not accepted by range broadening");
    check(with_project(packet(1, bgfb(18))).at("root_dispatch") == "selected" &&
              with_project(packet(1, bgfb(5))).at("status") == "rejected",
          "curve primitive and curve collection remain distinct native factories");
    check(with_project(packet(1, bgfb(15))).at("status") == "rejected",
          "native primitive dispatch skips the union tag between surface and curve extensions");
    check(with_project(packet(6, bgfb(255))).at("status") == "rejected",
          "unknown union tag can be rejected without decoding its unknown child schema");
    for (const auto offset : {16u, 18u}) {
        auto source = bgfb(13);
        write(source, offset, 0, 2);
        result = with_project(packet(3, source));
        check(result.at("status") == "rejected" &&
                  result.at("reason") == "missing_bgfb_union_field",
              "native wrapper rejects either missing union field before dispatch");
    }
    for (auto size : {0u, 4u, 8u, 27u, 28u, 35u})
        check(with_project(Bytes(size)).at("status") == "rejected",
              "truncated common header is a confirmed native null Entry");
    for (auto type : {1, 2, 3, 4, 5, 6, 7, 10}) {
        result = with_project(packet(type, {}));
        check(result.at("status") == "geometry_not_read" &&
                  result.at("material_footer") == "not_read" && result.at("reader").is_null(),
              "empty length returns the Entry before typed geometry and material readers");
    }
    for (auto type : {7, 10})
        check(with_project(packet(type, bgfb(13))).at("status") == "not_evaluated",
              "separate byte formats are not accepted because generic BGFB decodes");
    check(with_project(packet(4, bgfb(13))).at("status") == "rejected",
          "B-spline helper first rejects roots not accepted by the basic curve reader");
    result = with_project(packet(4, bgfb(3)));
    check(result.at("root_dispatch") == "selected" &&
              result.at("post_read_operation") == "get_bspline_curve_pointer" &&
              result.at("status") == "not_evaluated",
          "B-spline extraction follows basic curve construction, not arbitrary conversion");
    for (auto type : {0, 8, 9, -1, INT32_MAX})
        check(with_project(packet(type, bgfb(13))).at("status") == "geometry_not_read",
              "native default branch leaves geometry null without rejecting the Entry");
    auto malformed = packet(6, bgfb(6));
    write(malformed, 28, UINT64_MAX, 8);
    check(with_project(malformed).at("status") == "not_evaluated",
          "out-of-buffer geometry length is not guessed to be a native return value");
    auto truncated = bgfb(6);
    truncated.resize(25);
    result = with_project(packet(6, truncated));
    check(result.at("status") == "not_evaluated",
          "root selection alone does not establish an out-of-range child is constructed");
    auto wrong_root = bgfb(6);
    write(wrong_root, 8, UINT32_MAX, 4);
    check(with_project(packet(6, wrong_root)).at("status") == "not_evaluated",
          "root address uses bounded wide arithmetic");
    auto wrong_vtable = bgfb(6);
    write(wrong_vtable, 20, 100, 4);
    check(with_project(packet(6, wrong_vtable)).at("status") == "not_evaluated",
          "negative vtable address is reported instead of underflowing");

    auto missing_project = packet(6, bgfb(6));
    missing_project.erase(missing_project.begin() + 16, missing_project.begin() + 20);
    result = graphics_entry_native_input(missing_project);
    check(result.at("without_project").at("root_dispatch") == "selected" &&
              result.at("without_project").at("geometry_offset") == 32,
          "reader without a project does not advance past the serialized color word");
    result = graphics_entry_native_input(Bytes(35));
    check(result.at("status") == "not_evaluated" &&
              result.at("with_project").at("status") == "rejected" &&
              result.at("without_project").at("status") == "geometry_not_read",
          "a conditional header rejection is not an unconditional native rejection");
    check(graphics_entry_native_input(Bytes(31)).at("status") == "rejected",
          "shorter than both native headers is rejected independently of project context");
    result = graphics_entry_native_input(packet(6, bgfb(13)));
    check(result.at("status") == "not_evaluated" &&
              result.at("with_project").at("status") == "rejected",
          "reader mismatch is conditional on the project-dependent native offset");
    auto oversized = packet(6, bgfb(6));
    write(oversized, 28, 100, 8);
    result = with_project(oversized);
    check(result.at("status") == "geometry_not_read" && result.at("material_footer") == "not_read",
          "positive out-of-buffer length returns the Entry instead of calling its factory");

    const auto curve = parametric_graphics_append_rule(1);
    const auto array = parametric_graphics_append_rule(2);
    const auto spline = parametric_graphics_append_rule(4);
    check(curve.at("inline_material") == "not_passed" &&
              array.at("inline_material") == "copy_entry" && curve.at("widths") == "copy_entry" &&
              array.at("widths") == "copy_entry",
          "two curve append paths copy widths but differ in material transfer");
    check(spline.at("inline_material") == "not_passed" && spline.at("line_style_scale") == "one" &&
              spline.at("widths") == "zero",
          "B-spline append does not inherit source width, material or line scale");
    const auto solid = parametric_graphics_append_rule(6);
    check(solid.at("inline_material") == "copy_entry" && solid.at("layer_id") == UINT32_MAX &&
              solid.at("view_flag") == "destination_graphics" &&
              solid.at("source_container_transform") == "not_read",
          "copied solid material does not imply source layer or container transform inheritance");
    check(solid.at("output_count") == "zero_or_one_depending_on_add_result" &&
              parametric_graphics_append_rule(8).at("action") == "skip" &&
              parametric_graphics_append_rule(9).at("action") == "skip",
          "append planning does not fabricate stable final indices across failed or skipped adds");
    return checks;
}
