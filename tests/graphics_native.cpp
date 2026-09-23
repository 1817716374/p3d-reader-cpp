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

    auto point_curve = [&](unsigned tag, unsigned count, unsigned stored) {
        auto b = bgfb(tag);
        b.resize(68 + 8 * stored);
        write(b, 28, 20, 4);
        write(b, 40, 6, 2);
        write(b, 42, 8, 2);
        write(b, 44, 4, 2);
        write(b, 48, 8, 4);
        write(b, 52, 12, 4);
        write(b, 64, count, 4);
        return b;
    };
    auto collection = [&](unsigned tag, const std::vector<Bytes> &members) {
        auto b = bgfb(tag);
        b.resize(68 + 4 * members.size());
        write(b, 28, 20, 4);
        write(b, 40, 8, 2);
        write(b, 42, 12, 2);
        write(b, 44, 4, 2);
        write(b, 46, 8, 2);
        write(b, 48, 8, 4);
        write(b, 52, tag == 5 ? 2 : 12, 4);
        write(b, 56, tag == 5 ? 8 : 1, tag == 5 ? 4 : 1);
        write(b, 64, members.size(), 4);
        for (std::size_t i = 0; i < members.size(); ++i) {
            const auto start = b.size();
            b.insert(b.end(), members[i].begin(), members[i].end());
            // CurveVector stores VariantGeometry; RuledSweep stores CurveVector tables.
            write(b, 68 + 4 * i, start + (tag == 5 ? 20 : 48) - (68 + 4 * i), 4);
        }
        return b;
    };
    auto extrusion = [&](const Bytes &base, bool include_base) {
        auto b = bgfb(10);
        b.resize(88);
        write(b, 28, 20, 4);
        // A ten-byte vtable at 36 precedes the child at 48.
        write(b, 36, 10, 2);
        write(b, 38, 40, 2);
        write(b, 40, include_base ? 4 : 0, 2);
        write(b, 42, 8, 2);
        write(b, 44, 32, 2);
        write(b, 48, 12, 4);
        write(b, 52, 88 + 48 - 52, 4);
        write(b, 80, 1, 1);
        if (include_base)
            b.insert(b.end(), base.begin(), base.end());
        return b;
    };
    for (auto tag : {1u, 2u}) {
        auto source = scalar_solid(tag, tag == 1 ? 48 : 88);
        result = with_project(packet(1, source));
        check(result.at("status") == "geometry_constructed" &&
                  result.at("construction").at("geometry_kind") == "curve",
              "line and ellipse constructors accept complete inline records");
        write(source, 56, UINT64_C(0x7ff0000000000000), 8);
        check(with_project(packet(1, source)).at("status") == "geometry_constructed",
              "curve construction does not substitute a geometric validity test");
        source.pop_back();
        check(with_project(packet(1, source)).at("status") == "not_evaluated",
              "truncated curve detail is not a successfully constructed object");
    }
    for (auto tag : {4u, 18u}) {
        for (unsigned count = 0; count < 10; ++count) {
            auto source = point_curve(tag, count, count / 3 * 3);
            result = with_project(packet(1, source));
            check(result.at("status") == "geometry_constructed" &&
                      result.at("construction").at("point_count") == count / 3 &&
                      result.at("construction").at("ignored_tail_scalars") == count % 3,
                  "point curves retain empty and short input objects and discard scalar remainder");
            if (count >= 3) {
                source.pop_back();
                check(with_project(packet(1, source)).at("status") == "not_evaluated",
                      "point curve must contain every copied coordinate");
            }
        }
        auto missing_points = point_curve(tag, 0, 0);
        write(missing_points, 44, 0, 2);
        check(with_project(packet(1, missing_points)).at("status") == "not_evaluated",
              "absent point vector differs from present empty vector");
    }
    const auto line = scalar_solid(1, 48);
    const auto ellipse = scalar_solid(2, 88);
    const auto polyline = point_curve(4, 6, 6);
    const auto empty_group = collection(5, {});
    auto missing_group = empty_group;
    write(missing_group, 46, 0, 2);
    check(with_project(packet(2, empty_group)).at("status") == "geometry_constructed" &&
              with_project(packet(2, missing_group)).at("status") == "not_evaluated",
          "curve collection requires a stored member vector, including when empty");
    auto null_group = bgfb(5);
    write(null_group, 18, 0, 2);
    const auto nested_group = collection(5, {line, ellipse});
    const auto mixed_group = collection(5, {line, bgfb(255), scalar_solid(6, 120), nested_group,
                                            polyline, null_group, empty_group});
    result = with_project(packet(2, mixed_group));
    const auto &group = result.at("construction");
    check(result.at("status") == "geometry_constructed" && group.at("source_member_count") == 7 &&
              group.at("output_member_count") == 4,
          "native group filters null and non-curve variants without flattening nested groups");
    const auto &members = group.at("members");
    check(members.at(1).at("action") == "skip_null" &&
              members.at(2).at("action") == "skip_non_curve" &&
              members.at(3).at("action") == "wrap_nested_curve_vector" &&
              members.at(3).at("output_member_index") == 1 &&
              members.at(4).at("output_member_index") == 2 &&
              members.at(5).at("action") == "skip_null" &&
              members.at(6).at("output_member_index") == 3,
          "source-to-output member indices follow native filtering in source order");
    check(members.at(3).at("output_member_count") == 2,
          "nested collection retains its own member order and boundary");
    for (auto tag : {0u, 15u, 22u, 255u}) {
        auto ignored_member = bgfb(tag);
        write(ignored_member, 28, UINT32_MAX, 4);
        result = with_project(packet(2, collection(5, {ignored_member})));
        check(result.at("status") == "geometry_constructed" &&
                  result.at("construction").at("output_member_count") == 0,
              "generic member default branch never reads an ignored data pointer");
    }
    check(with_project(packet(2, collection(5, {bgfb(3)}))).at("status") == "not_evaluated",
          "unsupported known curve cannot be mistaken for an ignored generic union tag");
    check(with_project(packet(2, collection(5, {bgfb(6)}))).at("status") == "not_evaluated",
          "non-curve members still run their native constructor before being filtered");
    for (const auto &base : {empty_group, mixed_group}) {
        result = with_project(packet(6, extrusion(base, true)));
        check(result.at("status") == "geometry_constructed" &&
                  result.at("construction").at("capped") == true &&
                  result.at("construction").at("base_curve").at("geometry_pointer") == "non_null",
              "extrusion stores its constructed base even for an empty curve collection");
    }
    result = with_project(packet(6, extrusion({}, false)));
    check(result.at("status") == "geometry_constructed" &&
              result.at("construction").at("base_curve").at("geometry_pointer") == "null",
          "native extrusion constructor can retain a missing base without rejecting the solid");
    auto missing_direction = extrusion(empty_group, true);
    write(missing_direction, 42, 0, 2);
    check(with_project(packet(6, missing_direction)).at("status") == "not_evaluated",
          "missing inline extrusion direction is unsafe and not treated like a null base");
    result = with_project(packet(6, collection(12, {empty_group, mixed_group, nested_group})));
    check(result.at("status") == "geometry_constructed" &&
              result.at("construction").at("section_count") == 3 &&
              result.at("construction").at("sections").at(1).at("output_member_count") == 4 &&
              result.at("construction").at("sections").at(2).at("source_section_index") == 2,
          "ruled sweep preserves empty and nonempty section order without validity filtering");
    auto ruled = collection(12, {});
    write(ruled, 64, UINT32_MAX, 4);
    result = with_project(packet(6, ruled));
    check(result.at("status") == "geometry_constructed" &&
              result.at("construction").at("source_count_signed") == -1 &&
              result.at("construction").at("section_count") == 0,
          "ruled section loop uses signed count and skips negative values");
    auto huge_group = empty_group;
    write(huge_group, 64, UINT32_MAX, 4);
    check(with_project(packet(2, huge_group)).at("status") == "not_evaluated",
          "curve member loop instead uses the full unsigned count");
    auto deep = empty_group;
    for (unsigned i = 0; i < 42; ++i)
        deep = collection(5, {deep});
    result = with_project(packet(2, deep));
    check(result.at("status") == "not_evaluated" &&
              result.at("reason") == "native_geometry_construction_depth_limit",
          "nested construction has an explicit bounded work depth");

    auto loft = [&](const Bytes &bottom, const Bytes &top,
                    const std::vector<std::vector<Bytes>> &groups) {
        auto b = bgfb(21);
        b.resize(76 + 4 * groups.size());
        write(b, 28, 20, 4);
        write(b, 36, 12, 2);
        write(b, 38, 24, 2);
        write(b, 40, bottom.empty() ? 0 : 4, 2);
        write(b, 42, top.empty() ? 0 : 8, 2);
        write(b, 44, 12, 2);
        write(b, 46, 16, 2);
        write(b, 48, 12, 4);
        write(b, 60, 12, 4);
        write(b, 64, 1, 1);
        write(b, 72, groups.size(), 4);
        auto append_curve_vector = [&](const Bytes &source, std::size_t pointer) {
            const auto start = b.size();
            b.insert(b.end(), source.begin(), source.end());
            write(b, pointer, start + 48 - pointer, 4);
        };
        if (!bottom.empty())
            append_curve_vector(bottom, 52);
        if (!top.empty())
            append_curve_vector(top, 56);
        for (std::size_t i = 0; i < groups.size(); ++i) {
            const auto group = b.size();
            b.resize(group + 4 + 4 * groups[i].size());
            write(b, 76 + 4 * i, group - (76 + 4 * i), 4);
            write(b, group, groups[i].size(), 4);
            for (std::size_t j = 0; j < groups[i].size(); ++j)
                append_curve_vector(groups[i][j], group + 4 + 4 * j);
        }
        return b;
    };
    auto loft_source =
        loft(empty_group, nested_group, {{mixed_group, empty_group}, {}, {nested_group}});
    result = with_project(packet(6, loft_source));
    const auto &loft_result = result.at("construction");
    check(result.at("status") == "geometry_constructed" &&
              loft_result.at("section0").at("output_member_count") == 0 &&
              loft_result.at("section1").at("output_member_count") == 2 &&
              loft_result.at("capped") == true && loft_result.at("group_count") == 3,
          "native loft stores both section results and the original guide group count");
    const auto &groups = loft_result.at("guide_groups");
    check(groups.at(0).at("guide_count") == 2 && groups.at(1).at("guide_count") == 0 &&
              groups.at(2).at("guides").at(0).at("source_guide_index") == 0 &&
              groups.at(0).at("guides").at(0).at("output_member_count") == 4,
          "guide arrays retain both levels and empty groups without collapsing their order");
    for (unsigned mask = 0; mask < 4; ++mask) {
        result = with_project(packet(
            6, loft(mask & 1 ? empty_group : Bytes{}, mask & 2 ? nested_group : Bytes{}, {})));
        check(result.at("status") == "geometry_constructed" &&
                  result.at("construction").at("section0").at("geometry_pointer") ==
                      (mask & 1 ? "non_null" : "null") &&
                  result.at("construction").at("section1").at("geometry_pointer") ==
                      (mask & 2 ? "non_null" : "null"),
              "loft factory retains null section pointers without claiming usable geometry");
    }
    auto empty_loft = loft({}, {}, {});
    write(empty_loft, 44, 0, 2);
    check(with_project(packet(6, empty_loft)).at("status") == "not_evaluated",
          "absent guide group vector is an unsafe dereference, not an empty guide list");
    empty_loft = loft({}, {}, {});
    write(empty_loft, 72, UINT32_MAX, 4);
    result = with_project(packet(6, empty_loft));
    check(result.at("status") == "not_evaluated" &&
              result.at("reason") == "native_section_loft_negative_group_allocation",
          "negative outer count cannot skip the allocation that precedes native iteration");
    auto negative_inner = loft({}, {}, {{}});
    write(negative_inner, 80, UINT32_MAX, 4);
    result = with_project(packet(6, negative_inner));
    check(result.at("status") == "geometry_constructed" &&
              result.at("construction").at("guide_groups").at(0).at("guide_count") == 0 &&
              result.at("construction").at("guide_groups").at(0).at("source_count_signed") == -1,
          "negative inner count skips its signed loop while retaining the outer group");
    write(loft_source, 46, 0, 2);
    check(with_project(packet(6, loft_source)).at("construction").at("capped") == false,
          "omitted loft cap flag keeps the native reader default");
    auto unsupported_guide = collection(5, {bgfb(3)});
    check(with_project(packet(6, loft({}, {}, {{unsupported_guide}}))).at("status") ==
              "not_evaluated",
          "unknown guide construction cannot be replaced by an empty guide to accept a loft");
    return checks;
}
