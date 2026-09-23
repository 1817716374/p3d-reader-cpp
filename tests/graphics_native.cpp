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

    auto scalar_solid = [&](unsigned tag, unsigned detail_size) {
        auto b = bgfb(tag);
        b.resize(56 + detail_size);
        write(b, 28, 20, 4); // child at 48, vtable at 40, inline detail at 56
        write(b, 40, 6, 2);
        write(b, 42, 8 + detail_size, 2);
        write(b, 44, 8, 2);
        write(b, 48, 8, 4);
        return b;
    };
    const unsigned detail_sizes[] = {120, 120, 104, 136};
    const char *solid_names[] = {"DgnCone", "DgnSphere", "DgnTorusPipe", "DgnBox"};
    for (unsigned tag = 6; tag <= 9; ++tag) {
        auto source = scalar_solid(tag, detail_sizes[tag - 6]);
        result = with_project(packet(6, source));
        check(result.at("status") == "geometry_constructed" &&
                  result.at("geometry_pointer") == "non_null" &&
                  result.at("construction").at("geometry_type") == solid_names[tag - 6] &&
                  result.at("construction").at("detail_bytes") == detail_sizes[tag - 6],
              "scalar solid constructor copies the complete type-specific detail");
        write(source, 56, UINT64_C(0x7ff8000000000000), 8);
        check(with_project(packet(6, source)).at("status") == "geometry_constructed",
              "native scalar allocation does not assert finite or nondegenerate geometry");
        for (unsigned n = 0; n < detail_sizes[tag - 6]; ++n) {
            auto short_source = source;
            short_source.resize(56 + n);
            check(with_project(packet(6, short_source)).at("status") == "not_evaluated",
                  "every truncated detail extent stays unproved rather than a null native result");
        }
        write(source, 44, 0, 2);
        result = with_project(packet(6, source));
        check(result.at("status") == "not_evaluated" &&
                  result.at("reason") == "native_solid_reader_requires_detail_pointer",
              "missing detail is an unsafe native dereference, not an accepted default solid");
    }
    auto polyface = [&]() {
        auto b = bgfb(13);
        b.resize(160);
        write(b, 28, 60, 4); // child at 88
        write(b, 40, 44, 2);
        write(b, 42, 72, 2);
        write(b, 88, 48, 4);
        return b;
    };
    auto scalar_field = [&](Bytes &b, unsigned field, std::uint32_t value) {
        write(b, 44 + 2 * field, 4 + 4 * field, 2);
        write(b, 92 + 4 * field, value, field == 13 ? 1 : 4);
    };
    auto vector_field = [&](Bytes &b, unsigned field, unsigned count, unsigned stored_count) {
        const auto offset = b.size();
        const auto width = field < 4 ? 8u : 4u;
        b.resize(offset + 4 + stored_count * width);
        write(b, 44 + 2 * field, 4 + 4 * field, 2);
        write(b, 92 + 4 * field, offset - (92 + 4 * field), 4);
        write(b, offset, count, 4);
    };
    auto mesh = polyface();
    result = with_project(packet(3, mesh));
    check(result.at("status") == "geometry_constructed" &&
              result.at("construction").at("two_sided") == true &&
              result.at("construction").at("source_two_sided") == false,
          "empty mesh still constructs and layout initialization resets two-sided to true");
    const auto &empty_channels = result.at("construction").at("channels");
    check(empty_channels.at("point").at("active") == true &&
              empty_channels.at("pointIndex").at("active") == false &&
              result.at("construction").at("index_block_size") == 1,
          "omitted mesh channels retain factory activation and block-size defaults");
    scalar_field(mesh, 12, 1);
    scalar_field(mesh, 10, UINT32_MAX);
    scalar_field(mesh, 11, 17);
    scalar_field(mesh, 13, 0);
    result = with_project(packet(3, mesh));
    const auto &layout = result.at("construction");
    check(layout.at("mesh_style") == 1 && layout.at("num_per_row") == 17 &&
              layout.at("num_per_face") == -1 && layout.at("index_block_size") == UINT32_MAX &&
              layout.at("channels").at("pointIndex").at("active") == true,
          "mesh layout preserves signed source fields and unsigned index-block comparison");
    const char *channel_names[] = {"point",      "param",      "normal",     "doubleColor",
                                   "intColor",   "pointIndex", "paramIndex", "normalIndex",
                                   "colorIndex", "colorTable"};
    const unsigned component_counts[] = {3, 2, 3, 3, 1, 1, 1, 1, 1, 1};
    for (unsigned i = 0; i < 10; ++i) {
        auto source = polyface();
        const auto components = component_counts[i];
        vector_field(source, i, 2 * components + components - 1, 2 * components);
        result = with_project(packet(3, source));
        const auto &channel = result.at("construction").at("channels").at(channel_names[i]);
        check(result.at("status") == "geometry_constructed" && channel.at("active") == true &&
                  channel.at("source_present") == true && channel.at("element_count") == 2 &&
                  channel.at("ignored_tail_scalars") == components - 1 &&
                  channel.at("copied_bytes") == 2 * components * (i < 4 ? 8 : 4),
              "native channel reader copies complete tuples and ignores the scalar remainder");
        source.pop_back();
        check(with_project(packet(3, source)).at("status") == "not_evaluated",
              "missing copied channel bytes cannot establish native construction");
        auto zero = polyface();
        vector_field(zero, i, 0, 0);
        check(with_project(packet(3, zero))
                      .at("construction")
                      .at("channels")
                      .at(channel_names[i])
                      .at("active") == true,
              "present empty vector activates its channel independently of count");
        auto huge = polyface();
        vector_field(huge, i, UINT32_MAX, 0);
        check(with_project(packet(3, huge)).at("status") == "not_evaluated",
              "huge channel count is checked without allocating a native-sized array");
    }
    auto ignored = polyface();
    write(ignored, 44 + 2 * 14, 64, 2);
    write(ignored, 88 + 64, UINT32_MAX, 4);
    result = with_project(packet(3, ignored));
    check(result.at("status") == "geometry_constructed" &&
              result.at("construction").at("unread_fields").at(0) == "faceIndex",
          "source-only extension pointer is not dereferenced by the native mesh reader");
    auto bad_pointer = polyface();
    write(bad_pointer, 28, UINT32_MAX, 4);
    check(with_project(packet(3, bad_pointer)).at("status") == "not_evaluated",
          "out-of-range union child remains unproved");
    auto zero_pointer = polyface();
    write(zero_pointer, 28, 0, 4);
    check(with_project(packet(3, zero_pointer)).at("status") == "not_evaluated",
          "zero union relative pointer does not fabricate a table");
    auto projectless = packet(6, scalar_solid(9, 136));
    projectless.erase(projectless.begin() + 16, projectless.begin() + 20);
    check(graphics_entry_native_input(projectless).at("without_project").at("status") ==
              "geometry_constructed",
          "construction proof uses the project-dependent source geometry boundary");
    return checks;
}
