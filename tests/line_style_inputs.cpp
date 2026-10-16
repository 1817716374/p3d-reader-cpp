#include "line_style_source.hpp"
#include <p3d/line_style.hpp>
using namespace p3d;
namespace {
template <class T> void append(Bytes &b, T x) {
    const auto *p = reinterpret_cast<const std::uint8_t *>(&x);
    b.insert(b.end(), p, p + sizeof(x));
}
} // namespace
unsigned line_style_inputs_tests() {
    unsigned checks = 0;
    auto check = [&](bool ok, const char *message) {
        require(ok, message);
        ++checks;
    };
    auto set = [](Bytes &b, std::size_t at, auto v) { std::memcpy(b.data() + at, &v, sizeof(v)); };
    auto link = [](Bytes payload) {
        require(payload.size() % 2 == 0, "fixture word alignment");
        return Json{{"app", 0x79f9},
                    {"header", 0x1000u + unsigned((payload.size() + 4) / 2 - 1)},
                    {"offset", 116},
                    {"payload", rawbytes(payload)}};
    };
    auto payload = [](std::uint32_t flags) {
        Bytes b;
        append(b, flags);
        return b;
    };
    auto record = [&](unsigned type, unsigned style, const Json &links, bool dim3 = false,
                      bool extended = true) {
        Bytes b(116, 0);
        set(b, 4, std::uint16_t(type));
        set(b, 6, std::uint16_t(extended ? 0x20 : 0));
        std::size_t size = b.size();
        for (const auto &l : links)
            size += 4 + bytesof(l.at("payload")).size();
        set(b, 8, std::uint32_t((size - 4) / 2));
        set(b, 12, std::uint32_t((b.size() - 4) / 2));
        set(b, 16, std::uint32_t(64));
        set(b, 36, std::uint16_t(dim3 ? 0x1000 : 0));
        set(b, 48, std::uint32_t(style));
        return Json{{"data", rawbytes(b)}, {"links", links}};
    };
    auto scalars = payload(0xc0003bffu);
    for (double v : {1., 2., 3., 4., 5., 6., 7., 8., 9., 10., 1., 2., 3., 4.})
        append(scalars, v);
    append(scalars, std::uint32_t(1));
    append(scalars, std::uint32_t(7));
    const auto original = link(scalars);
    const auto before = original;
    auto d = load_native_line_style_parameter_link(original, false);
    check(d["status"] == "decoded" && d["reader_return_code"] == 0 &&
              d["source"]["orientation_data"] == Json({1., 2., 3., 4.}) &&
              d["source"]["orientation_encoding"] == "matrix_2x2" && original == before,
          "link parameters retain source orientation and explicit two-dimensional interpretation "
          "without mutation");
    check(d["state"]["orientation_matrix"] == Json({{1., 2., 0.}, {3., 4., 0.}, {0., 0., 1.}}) &&
              d["state"]["orientation_vector"] == Json({0., 0., 1.}) &&
              d["source"]["orientation_vector"] == Json({8., 9., 10.}),
          "two-dimensional four-value matrix overwrites the effective normal while retaining its "
          "earlier source");
    check(d["state"]["scalars"]["scale"] == 1. && d["state"]["scalars"]["fraction_phase"] == 7. &&
              d["state"]["multiline_index"] == 1 && d["state"]["multiline_data_type"] == 7 &&
              d["state"]["break_at_corners"] == true && d["state"]["run_through_corners"] == true &&
              d["state"]["true_scale"] == true && d["consumed_payload_bytes"] == scalars.size(),
          "all supported parameter channels and conflicting corner flags retain native values");
    d = load_native_line_style_parameter_link(original, true);
    check(d["source"]["orientation_encoding"] == "quaternion_wxyz" &&
              std::abs(d["state"]["orientation_vector"][0].get<double>() - 22. / 30.) < 1e-15 &&
              std::abs(d["state"]["orientation_vector"][1].get<double>() - 20. / 30.) < 1e-15 &&
              std::abs(d["state"]["orientation_vector"][2].get<double>() - 4. / 30.) < 1e-15,
          "three-dimensional quaternion order, normalization and transpose give the native third "
          "row");
    auto orient = payload(0x200);
    for (double v : {2., 2., 0., 0.})
        append(orient, v);
    d = load_native_line_style_parameter_link(link(orient), true);
    check(d["state"]["orientation_matrix"] == Json({{1., 0., 0.}, {0., 0., 1.}, {0., -1., 0.}}),
          "nonunit quaternion uses native normalization and transposed handedness");
    orient = payload(0x200);
    for (unsigned i = 0; i < 4; ++i)
        append(orient, 0.);
    check(
        load_native_line_style_parameter_link(link(orient), true)["state"]["orientation_matrix"] ==
            Json({{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}}),
        "zero quaternion uses native identity rather than rejecting or dividing by zero");
    auto ignored = link(payload(UINT32_MAX));
    d = load_native_line_style_parameter_link(ignored, true);
    check(d["status"] == "decoded" && d["reader_return_code"] == 0 &&
              d["selection"] == "length_four_words_uses_defaults" && d["state"]["flags"] == 0 &&
              d["state"]["scalars"]["scale"] == 1. && d["consumed_payload_bytes"] == 0,
          "four-word linkage ignores even hostile payload flags and successfully writes defaults");
    auto invalid = payload(0x401);
    append(invalid, 9.);
    d = load_native_line_style_parameter_link(link(invalid), false);
    check(d["status"] == "decoded" && d["reader_return_code"] == 1 &&
              d["source"]["named_values"]["scale"] == 9. && d["state"]["flags"] == 0 &&
              d["state"]["scalars"]["scale"] == 1. && d["rejected_flag_bits"] == 0x400,
          "reserved flags reset the entire loaded state and return failure without discarding "
          "source values");
    auto nonfinite = payload(1);
    append(nonfinite, std::uint64_t(0x7ff8000000001234ull));
    d = load_native_line_style_parameter_link(link(nonfinite), true);
    check(d["reader_return_code"] == 0 &&
              d["state"]["scalars"]["scale"]["floating_point"] == "nan" &&
              d["state"]["scalars"]["scale"]["ieee754_hex"] == "7ff8000000001234" &&
              Reader(bytesof(d["state"]["storage"]), 8).u64() == 0x7ff8000000001234ull,
          "nonfinite scalar bits remain interpretable JSON and unchanged native storage");
    auto negative = payload(1);
    append(negative, std::uint64_t(0x8000000000000000ull));
    check(Reader(bytesof(load_native_line_style_parameter_link(link(negative),
                                                               false)["state"]["storage"]),
                 8)
                  .u64() == 0x8000000000000000ull,
          "negative zero is not normalized during parameter copying");
    auto good = payload(1);
    append(good, 5.);
    auto graphic = record(37, 9, Json::array({link(good)}), true);
    d = read_native_graphic_line_style_parameters(graphic);
    check(
        d["status"] == "decoded" && d["three_dimensional"] == true && d["source_link_index"] == 0 &&
            d["state"]["scalars"]["scale"] == 5.,
        "graphic reader takes dimension from the source header and selects the first user linkage");
    graphic["links"].insert(graphic["links"].begin(), link(invalid));
    d = read_native_graphic_line_style_parameters(graphic);
    check(d["reader_return_code"] == 1 && d["source_link_index"] == 0 &&
              d["state"]["scalars"]["scale"] == 1.,
          "failed first matching graphic linkage does not fall through to a later valid one");
    auto short_payload = payload(1);
    append(short_payload, std::uint32_t(0));
    graphic["links"][0] = link(short_payload);
    d = read_native_graphic_line_style_parameters(graphic);
    check(d["status"] == "not_evaluated" && !d.contains("state") && d["source_link_index"] == 0,
          "truncated first parameter block neither reads the next linkage nor publishes a partial "
          "state");
    auto long_payload = payload(0);
    long_payload.resize(180, 0);
    check(load_native_line_style_parameter_link(link(long_payload), false)["status"] == "decoded" &&
              read_native_graphic_line_style_parameters(
                  record(37, 9, Json::array({link(long_payload)})))["status"] == "not_evaluated",
          "direct bounded linkage reading is separate from the graphic reader's fixed scratch-copy "
          "limit");
    check(read_native_graphic_line_style_parameters(record(
              37, 7, Json::array({link(short_payload)})))["selection"] == "builtin_line_style" &&
              read_native_graphic_line_style_parameters(
                  record(37, 9, Json::array({link(short_payload)}), false, false))["selection"] ==
                  "nonextended_header",
          "native header and built-in-style gates avoid reading unused malformed parameters");
    auto multiline = record(63, 9, Json::array({link(good)}));
    d = read_native_graphic_line_style_parameters(multiline);
    check(d["status"] == "decoded" && d["parameter_reader"] == "multiline_default" &&
              d["reader_return_code"] == 0 && d["state_write"] == "written" &&
              d["state"]["scalars"]["scale"] == 5.,
          "multiline default route selects a converted data-type-zero parameter");
    auto multiline_role = [&](unsigned index, unsigned type, double scale) {
        auto p = payload(0x1801);
        append(p, scale);
        append(p, std::uint32_t(index));
        append(p, std::uint32_t(type));
        return link(p);
    };
    multiline["links"] =
        Json::array({multiline_role(1, 2, 7), multiline_role(99, 0, 8), multiline_role(0, 0, 9)});
    d = read_native_graphic_line_style_parameters(multiline);
    check(d["source_link_index"] == 1 && d["ignored_link_indices"] == Json({0}) &&
              d["state"]["multiline_index"] == 99 && d["state"]["scalars"]["scale"] == 8.,
          "multiline default mode skips nonzero data types, ignores the index, and stops at the "
          "first match");
    multiline["links"] = Json::array({link(invalid), link(good)});
    d = read_native_graphic_line_style_parameters(multiline);
    check(d["reader_return_code"] == 0 && d["source_link_index"] == 0 &&
              d["linkage"]["reader_return_code"] == 1 && d["state"]["flags"] == 0 &&
              d["state"]["scalars"]["scale"] == 1.,
          "multiline selection tests the reset data type and ignores the inner loader failure");
    multiline["links"] = Json::array({multiline_role(0, 4, 7)});
    d = read_native_graphic_line_style_parameters(multiline);
    check(d["status"] == "not_found" && d["reader_return_code"] == 1 &&
              d["state_write"] == "written" && d["ignored_link_indices"] == Json({0}) &&
              bytesof(d["state"]["storage"]) == Bytes(0xa8, 0),
          "multiline known miss writes all zeros rather than the ordinary defaults");
    check(bytesof(read_native_graphic_line_style_parameters(
              record(63, 9, Json::array()))["state"]["storage"]) == Bytes(0xa8, 0),
          "multiline empty linkage span also writes zero state");
    auto gated = record(63, 7, Json::array());
    d = read_native_graphic_line_style_parameters(gated);
    check(d["state_write"] == "not_written" && !d.contains("state"),
          "multiline built-in-style gate precedes destination clearing");
    multiline["links"] = Json::array({link(short_payload), link(good)});
    d = read_native_graphic_line_style_parameters(multiline);
    check(d["status"] == "not_evaluated" && !d.contains("state") && d["source_link_index"] == 0,
          "unsafe multiline input does not fall through or publish zero state as a known result");
    check(read_native_graphic_line_style_parameters(
              record(63, 9, Json::array({link(long_payload)})))["status"] == "decoded",
          "multiline iterator does not inherit the ordinary reader's 176-byte scratch limit");
    d = read_native_graphic_line_style_parameters(record(63, 9, Json::array({link(orient)}), true));
    check(d["three_dimensional"] == true &&
              d["state"]["orientation_matrix"] == Json({{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}}),
          "multiline callback uses the source dimension flag for orientation");
    for (unsigned type : {27u, 38u, 41u, 52u})
        check(read_native_graphic_line_style_parameters(
                  record(type, 9, Json::array({ignored})))["state"]["flags"] == 0x80000000u,
              "native selected graphic types add the run-through flag even after a defaults-only "
              "successful read");
    auto nonuser = link(good);
    nonuser["header"] = 3;
    d = read_native_graphic_line_style_parameters(
        record(37, UINT32_MAX, Json::array({nonuser, link(good)})));
    check(d["source_link_index"] == 1 && d["reader_return_code"] == 0,
          "non-user linkage is skipped and unsigned style sentinels still enter the native "
          "complex-style route");
    auto role = [&](unsigned index, double scale) {
        auto p = payload(0x801);
        append(p, scale);
        append(p, std::uint32_t(index));
        return link(p);
    };
    auto layers = native_layer_line_style_input(Json::array({role(1, 2), role(2, 3), role(1, 4)}));
    check(layers["status"] == "decoded" && layers["selected_entries"]["primary"] == 2 &&
              layers["selected_entries"]["by_layer"] == 1 &&
              layers["entries"][2]["parameters"]["state"]["scalars"]["scale"] == 4.,
          "layer parameter slots load independently and the last entry for each role wins");
    layers = native_layer_line_style_input(Json::array({role(1, 2), link(invalid)}));
    check(layers["selected_entries"]["primary"] == 0 &&
              layers["entries"][1]["selection"] == "ignored_slot_index",
          "rejected parameter flags reset the role to zero and do not erase an earlier layer slot");
    layers = native_layer_line_style_input(Json::array({role(1, 2), link(short_payload)}));
    check(
        layers["status"] == "not_evaluated" && layers["selected_entries"]["primary"].is_null() &&
            layers["entries"].size() == 2,
        "unknown layer input preserves entries but does not expose an unproven final slot prefix");
    auto bad_header = link(good);
    bad_header["header"] = 0x1003;
    check(load_native_line_style_parameter_link(bad_header, false)["status"] == "not_evaluated",
          "declared linkage span cannot silently truncate the supplied source payload");
    auto b = bytesof(record(37, 9, Json::array())["data"]);
    const auto l = link(good);
    append(b, std::uint16_t(l["header"].get<unsigned>()));
    append(b, std::uint16_t(0x79f9));
    const auto p = bytesof(l["payload"]);
    b.insert(b.end(), p.begin(), p.end());
    set(b, 8, std::uint32_t((b.size() - 4) / 2));
    set(b, 12, std::uint32_t(56));
    const auto parsed = parse_native(b)[0];
    check(
        parsed["native_line_style_parameters"] ==
                read_native_graphic_line_style_parameters(parsed) &&
            parsed["native_line_style_parameters"]["reader_return_code"] == 0,
        "native record parsing automatically publishes the same bounded graphic parameter result");
    set(b, 4, std::uint16_t(63));
    const auto parsed_multiline = parse_native(b)[0];
    check(parsed_multiline["native_line_style_parameters"] ==
                  read_native_graphic_line_style_parameters(parsed_multiline) &&
              parsed_multiline["native_line_style_parameters"]["parameter_reader"] ==
                  "multiline_default",
          "native multiline records automatically use the dedicated default-parameter route");
    auto no_link = bytesof(record(37, 9, Json::array())["data"]);
    set(no_link, 8, std::uint32_t(56));
    set(no_link, 12, std::uint32_t(56));
    const auto lazy = parse_native(no_link)[0];
    check(!lazy.contains("native_line_style_parameters") &&
              read_native_graphic_line_style_parameters(lazy)["selection"] ==
                  "no_parameter_linkage",
          "records without parameter links avoid eager result allocation but retain exact "
          "on-demand lookup");
    auto cmd = payload(0x200);
    for (double v : {1., 2., 3., 4.})
        append(cmd, v);
    Bytes wire;
    append(wire, std::uint16_t(0x8000));
    append(wire, std::uint16_t(cmd.size()));
    wire.insert(wire.end(), cmd.begin(), cmd.end());
    const auto modifier = decode_symbology(wire)["line_style_modifiers"];
    check(modifier["orientation_data"] == Json({1., 2., 3., 4.}) &&
              modifier["orientation_encoding"] == "requires_dimension_context" &&
              !modifier.contains("orientation_quaternion"),
          "dimensionless command source does not falsely label a two-dimensional matrix as a "
          "quaternion");
    Bytes layer(108, 0);
    set(layer, 4, std::uint16_t(49));
    set(layer, 16, std::uint32_t(1));
    set(layer, 44, std::uint16_t(7));
    set(layer, 52, std::int32_t(-2));
    set(layer, 56, std::uint32_t(UINT32_MAX));
    set(layer, 60, std::uint32_t(0x23));
    set(layer, 64, std::uint32_t(0x123));
    auto layer_input = decode_native_layer(layer, Json::array({role(1, 2), role(2, 3)}));
    check(layer_input["primary_symbology"]["color_index"] == 0x123 &&
              layer_input["primary_symbology"]["line_style"] == -2 &&
              layer_input["primary_symbology"]["line_weight"] == UINT32_MAX &&
              bytesof(layer_input["unassigned_ranges"][0]["data"]) == slice(layer, 48, 4) &&
              layer_input["line_style_parameter_inputs"]["selected_entries"]["by_layer"] == 1,
          "native layer primary symbology and both parameter slots are decoded independently of "
          "by-layer fields");
    set(layer, 60, std::uint32_t(0x24));
    check(decode_native_layer(layer, Json::array())["primary_symbology"]["color_index"] == 0x24,
          "incompatible primary extended color preserves its original legacy index");
    const auto parameter_links = Json::array({role(1, 2), role(2, 3), role(1, 4)});
    const auto parameter_inputs = native_layer_line_style_input(parameter_links);
    const std::array<std::int32_t, 6> styles{{INT32_MIN, -1, 0, 1, 7, INT32_MAX}};
    for (unsigned version = 1; version <= 7; ++version)
        for (const auto primary : styles)
            for (const auto by_layer : styles) {
                set(layer, 44, std::uint16_t(version));
                set(layer, 52, primary);
                set(layer, 68, by_layer);
                const auto decoded = decode_native_layer(layer, parameter_links);
                const auto &slots = decoded.at("line_style_parameter_slots");
                check(slots["status"] == "decoded" &&
                          slots["line_style_resource_loading"] == "not_evaluated" &&
                          decoded["line_style_parameter_inputs"] == parameter_inputs,
                      "slot cleanup preserves all saved parameter inputs for each supported layer "
                      "version");
                for (auto item :
                     {std::make_pair("primary", primary), std::make_pair("by_layer", by_layer)}) {
                    const auto &slot = slots.at(item.first);
                    const auto index = item.first == std::string("primary") ? 2u : 1u;
                    check(slot["saved_line_style"] == item.second &&
                              slot["source_entry_index"] == index &&
                              slot["source_link_index"] == index &&
                              slot["presence"] == (item.second > 0 ? "empty" : "present") &&
                              slot["state"] ==
                                  (item.second > 0
                                       ? Json()
                                       : parameter_inputs["entries"][index]["parameters"]["state"]),
                          "signed positive style cleanup retains zero and negative slots with all "
                          "168 bytes");
                }
            }
    auto empty_slots =
        native_layer_line_style_slots(native_layer_line_style_input(Json::array()), 0, -1);
    check(
        empty_slots["primary"]["selection"] == "constructor_default" &&
            empty_slots["primary"]["state"].is_null() &&
            empty_slots["by_layer"]["presence"] == "empty",
        "absent layer parameter pointers are empty rather than allocated default parameter blocks");
    const auto unknown_inputs =
        native_layer_line_style_input(Json::array({role(1, 2), link(short_payload)}));
    const auto unknown_slots = native_layer_line_style_slots(unknown_inputs, 1, 7);
    check(unknown_slots["status"] == "not_evaluated" &&
              unknown_slots["primary"]["presence"] == "unknown" &&
              unknown_slots["by_layer"]["presence"] == "unknown",
          "unsafe linkage reads cannot be concealed by a later positive-style cleanup");
    return checks;
}
