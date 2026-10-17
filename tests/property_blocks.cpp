#include "internal.hpp"
#include <p3d/property_blocks.hpp>
#include <thread>

unsigned property_block_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        require(value, why);
        ++checks;
    };
    auto append = [](Bytes &b, auto v) {
        const auto n = b.size();
        b.resize(n + sizeof(v));
        std::memcpy(b.data() + n, &v, sizeof(v));
    };
    auto text = [&](Bytes &b, const std::string &s) {
        append(b, std::uint32_t(4));
        append(b, std::uint32_t(s.size()));
        b.insert(b.end(), s.begin(), s.end());
    };
    auto typed = [&](Bytes &b, unsigned type, auto v) {
        append(b, std::uint32_t(type));
        append(b, v);
    };
    auto header = [&] {
        Bytes b;
        text(b, "UserNamespace");
        text(b, "UserClass");
        return b;
    };
    auto field = [&](Bytes &b, std::string name, std::string type, bool array = false) {
        text(b, name);
        text(b, "DeclaringNamespace");
        typed(b, 0, std::uint8_t(array));
        text(b, type);
    };
    auto bytes = [&](Bytes &b, const Bytes &v) {
        append(b, std::uint32_t(5));
        append(b, std::uint32_t(v.size()));
        b.insert(b.end(), v.begin(), v.end());
    };
    auto a = header();
    field(a, "Enabled", "boolean");
    typed(a, 0, std::uint8_t(1));
    field(a, "Count", "int");
    typed(a, 1, std::int32_t(-12));
    field(a, "Scale", "double");
    typed(a, 2, 1.25);
    field(a, "Handle", "long");
    typed(a, 3, INT64_MIN);
    field(a, "Label", "string");
    text(a, "value");
    field(a, "Payload", "binary");
    bytes(a, Bytes{0, 0xff, 4});
    field(a, "Point", "point3d");
    typed(a, 6, 1.0);
    append(a, 2.0);
    append(a, 3.0);
    field(a, "Planar", "point2d");
    typed(a, 7, -2.0);
    append(a, 4.0);
    field(a, "Updated", "dateTime");
    field(a, "OpaqueClass", "UserStruct");
    field(a, "Duplicate", "int");
    typed(a, 1, std::int32_t(9));
    field(a, "Duplicate", "int");
    typed(a, 1, std::int32_t(10));
    const auto result = decode_property_data_block(a);
    const auto &view = result.at("property_structure"), &fields = view.at("fields"),
               &tokens = result.at("tokens");
    check(view.at("status") == "structured" && fields.size() == 12 &&
              view.at("remaining_token_count") == 0 && bytesof(result.at("source_storage")) == a,
          "generic property declarations and exact source bytes survive without a schema");
    for (std::size_t i = 0; i < 8; ++i) {
        const auto index = fields[i].at("value_token_indices")[0].get<std::size_t>();
        check(tokens[index].at("type_code") == i &&
                  fields[i].at("namespace") == "DeclaringNamespace",
              "primitive type table is independent from business names and preserves declaring "
              "namespace");
    }
    check(tokens[fields[0].at("value_token_indices")[0].get<std::size_t>()].at("boolean_value") ==
                  true &&
              tokens[fields[7].at("value_token_indices")[0].get<std::size_t>()].at("value") ==
                  Json::array({-2.0, 4.0}),
          "boolean and point2d values follow the confirmed native layouts");
    check(fields[8].at("value_status") == "not_serialized" &&
              fields[9].at("value_status") == "not_serialized" &&
              fields[10].at("name") == fields[11].at("name") && fields[10] != fields[11],
          "header-only values are not fabricated and repeated declarations are not merged");
    auto arrays = header();
    for (unsigned type = 0; type <= 7; ++type) {
        const char *names[] = {"boolean", "int",    "double",  "long",
                               "string",  "binary", "point3d", "point2d"};
        field(arrays, "Items", names[type], true);
        typed(arrays, 3, std::int64_t(2));
        for (unsigned n = 0; n < 2; ++n) {
            if (type == 0)
                typed(arrays, 0, std::uint8_t(n));
            if (type == 1)
                typed(arrays, 1, std::int32_t(n));
            if (type == 2)
                typed(arrays, 2, double(n));
            if (type == 3)
                typed(arrays, 3, std::int64_t(n));
            if (type == 4)
                text(arrays, std::to_string(n));
            if (type == 5)
                bytes(arrays, Bytes{std::uint8_t(n)});
            if (type == 6 || type == 7) {
                typed(arrays, type, double(n));
                append(arrays, 2.0);
                if (type == 6)
                    append(arrays, 3.0);
            }
        }
    }
    const auto array_result = decode_property_data_block(arrays);
    check(array_result.at("property_structure").at("status") == "structured",
          "all native primitive array layouts are consumed");
    for (const auto &f : array_result.at("property_structure").at("fields"))
        check(f.at("declared_count") == 2 && f.at("value_token_indices").size() == 2,
              "array counts do not consume the following property header");
    auto transforms = header();
    for (const auto &name : {"Placement", "BaseTransform"}) {
        text(transforms, name);
        text(transforms, "AnySchema");
        typed(transforms, 6, 1.0);
        append(transforms, 2.0);
        append(transforms, 3.0);
        for (unsigned i = 0; i < 3; ++i) {
            if (std::string(name) == "Placement")
                typed(transforms, 2, double(i));
            else {
                typed(transforms, 6, double(i));
                append(transforms, 0.0);
                append(transforms, 1.0);
            }
        }
    }
    const auto transform_result = decode_property_data_block(transforms);
    check(transform_result.at("property_structure").at("status") == "structured" &&
              transform_result.at("property_structure").at("fields")[0].at("token_count") == 6 &&
              transform_result.at("property_structure").at("fields")[1].at("token_count") == 6,
          "native special transform fields have four values and no ordinary array/type header");
    auto children = header();
    field(children, "Children", "UserStruct", true);
    typed(children, 3, std::int64_t(2));
    for (std::int64_t i = 0; i < 2; ++i) {
        typed(children, 3, INT64_MAX - i);
        typed(children, 3, std::int64_t(a.size()));
        bytes(children, a);
    }
    const auto nested = decode_property_data_block(children);
    const auto &child_fields = nested.at("property_structure").at("fields")[0];
    check(nested.at("property_structure").at("status") == "structured" &&
              child_fields.at("elements").size() == 2 &&
              child_fields.at("elements")[0].at("data_block") == result &&
              child_fields.at("elements")[1].at("data_block") == result,
          "struct array elements retain independent occurrences and nested source blocks");
    check(child_fields.at("elements")[0].at("stored_entry_offset") == INT64_MAX &&
              child_fields.at("elements")[0].at("stored_entry_offset_matches_position") == false,
          "the reader ignores a stale saved entry offset rather than seeking or reordering by it");
    auto native_offsets = header();
    field(native_offsets, "Children", "UserStruct", true);
    typed(native_offsets, 3, std::int64_t(1));
    typed(native_offsets, 3, std::int64_t(native_offsets.size()));
    typed(native_offsets, 3, std::int64_t(a.size()));
    bytes(native_offsets, a);
    check(decode_property_data_block(native_offsets)
                  .at("property_structure")
                  .at("fields")[0]
                  .at("elements")[0]
                  .at("stored_entry_offset_matches_position") == true,
          "the native writer stores the entry's byte offset before its two int64 headers");
    auto truncated = header();
    field(truncated, "Broken", "point2d");
    typed(truncated, 6, 1.0);
    append(truncated, 2.0);
    append(truncated, 3.0);
    const auto partial = decode_property_data_block(truncated);
    check(partial.at("property_structure").at("status") == "partial" &&
              partial.at("tokens").size() == 7 &&
              bytesof(partial.at("source_storage")) == truncated,
          "inconsistent field layout preserves the complete typed source and reports partial");
    auto empty = header();
    field(empty, "Empty", "point3d", true);
    typed(empty, 3, std::int64_t(-1));
    field(empty, "Dates", "dateTime", true);
    typed(empty, 3, std::int64_t(2));
    const auto empties = decode_property_data_block(empty);
    check(empties.at("property_structure").at("status") == "structured" &&
              empties.at("property_structure").at("fields")[0].at("value_token_indices").empty() &&
              empties.at("property_structure").at("fields")[1].at("value_status") ==
                  "not_serialized",
          "native nonpositive counts and dateTime arrays consume no value tokens");
    auto dates = header();
    field(dates, "Dates", "dateTime", true);
    typed(dates, 3, INT64_MAX);
    PropertyBlockLimits unused_values;
    unused_values.max_array_elements = 1;
    check(decode_property_data_block(dates, unused_values).at("property_structure").at("status") ==
              "structured",
          "an unconsumed dateTime value list does not allocate or exhaust an element budget");
    auto mismatch_bytes = header();
    field(mismatch_bytes, "Nested", "UserStruct", true);
    typed(mismatch_bytes, 3, std::int64_t(1));
    typed(mismatch_bytes, 3, std::int64_t(0));
    typed(mismatch_bytes, 3, std::int64_t(a.size() + 1));
    bytes(mismatch_bytes, a);
    const auto mismatched = decode_property_data_block(mismatch_bytes);
    check(mismatched.at("property_structure").at("status") == "partial" &&
              bytesof(mismatched.at("source_storage")) == mismatch_bytes,
          "nested length disagreement is explicit and cannot be repaired by guessing a boundary");
    auto infinite = header();
    field(infinite, "Value", "double");
    typed(infinite, 2, std::uint64_t(0x7ff8000000000123ull));
    const auto nonfinite = decode_property_data_block(infinite);
    check(nonfinite.at("tokens").back().at("nonfinite_float_bits")[0].at("bits") ==
              0x7ff8000000000123ull,
          "nonfinite floating payloads retain their original bit patterns");
    for (unsigned which = 0; which < 5; ++which) {
        PropertyBlockLimits limits;
        if (which == 0)
            limits.max_tokens = 2;
        if (which == 1)
            limits.max_fields = 1;
        if (which == 2)
            limits.max_array_elements = 1;
        if (which == 3)
            limits.max_depth = 1;
        if (which == 4)
            limits.max_decoded_bytes = children.size() - 1;
        bool denied = false;
        try {
            denied = decode_property_data_block(children, limits)
                         .at("property_structure")
                         .at("status") == "partial";
        } catch (const std::runtime_error &) {
            denied = true;
        }
        check(denied,
              "token, field, array, nested depth and aggregate source budgets are enforced");
    }
    std::array<Json, 4> parallel;
    std::array<std::thread, 4> threads;
    for (unsigned i = 0; i < 4; ++i)
        threads[i] = std::thread([&, i] { parallel[i] = decode_property_data_block(children); });
    for (auto &t : threads)
        t.join();
    check(std::all_of(parallel.begin(), parallel.end(), [&](const Json &j) { return j == nested; }),
          "parallel property decoders do not share mutable parsing state");
    return checks;
}
