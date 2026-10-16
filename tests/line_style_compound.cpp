#include "internal.hpp"
#include <p3d/line_style.hpp>

unsigned line_style_compound_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        require(value, why);
        ++checks;
    };
    auto put = [](Bytes &b, std::size_t p, auto value) {
        std::memcpy(b.data() + p, &value, sizeof(value));
    };
    auto make = [&](unsigned children, unsigned references, std::size_t bytes = 0) {
        const auto extent = children ? 176 + 56 * children : 232;
        Bytes b(bytes ? bytes : 208 + extent + 8 * std::max(children, references) + 28, 0);
        put(b, 4, std::uint16_t(49));
        put(b, 8, std::uint32_t((b.size() - 4) / 2));
        put(b, 12, std::uint32_t((b.size() - 4) / 2));
        put(b, 16, std::uint32_t(8));
        put(b, 70, std::uint16_t(2));
        put(b, 200, std::uint16_t(references));
        put(b, 202, std::uint16_t(8 * references));
        put(b, 204, std::uint16_t(extent));
        put(b, 380, std::uint32_t(children));
        return b;
    };
    auto read = [](const Bytes &b) {
        return read_native_line_style_resource({{"data", rawbytes(b)}});
    };
    // Three children, up to five references, all zero/nonzero reference masks
    // and both native/ordinary/unknown type values. The independent table also
    // covers the two resource types that converge to the same local kind.
    const std::array<std::pair<std::uint32_t, std::uint32_t>, 13> kinds{
        {{0, 6},
         {1, 1},
         {2, 2},
         {3, 3},
         {4, 4},
         {6, 6},
         {2001, 2},
         {2002, 3},
         {2003, 4},
         {2004, 1},
         {2005, 1},
         {0x80000000u, 0x80000000u},
         {0xffffffffu, 0xffffffffu}}};
    const std::array<std::uint64_t, 5> ids{
        {0xfedcba9800000011ull, 0x100000000ull, UINT64_MAX, 0x1234567800000001ull, 5}};
    for (std::size_t k = 0; k < kinds.size(); ++k)
        for (unsigned mask = 0; mask < 32; ++mask)
            for (unsigned used = 0; used <= 5; ++used) {
                auto b = make(3, used);
                constexpr std::size_t refs = 552;
                for (unsigned i = 0; i < 3; ++i) {
                    put(b, 384 + 56 * i, kinds[(k + i) % kinds.size()].first);
                    put(b, 388 + 56 * i, std::uint32_t(101 + i));
                    put(b, 408 + 56 * i, -0.5 + i * 1.25);
                }
                for (unsigned i = 0; i < 5; ++i)
                    put(b, refs + i * 8, mask & (1u << i) ? ids[i] : 0ull);
                const auto source_before = b;
                const auto result = read(b), &c = result.at("compound");
                const auto &copy = c.at("copy_reader_fields"),
                           &obj = c.at("initial_object_children");
                bool invalid_copy = false;
                unsigned first_bad = 0;
                for (unsigned i = 3; i < used; ++i)
                    if (mask & (1u << i)) {
                        invalid_copy = true;
                        first_bad = i;
                        break;
                    }
                bool valid =
                    c.at("source_status") == "decoded" && obj.at("status") == "resolved" &&
                    c.at("children").size() == 3 && c.at("reference_source_offset") == refs &&
                    c.at("source_payload").at("data").at("bytes") == b.size() - 208 &&
                    bytesof(c.at("source_payload").at("data")) == slice(b, 208, b.size() - 208) &&
                    result.at("status") == (invalid_copy ? "partial" : "decoded") &&
                    b == source_before;
                for (unsigned i = 0; i < 3; ++i) {
                    const bool present = mask & (1u << i);
                    const auto [saved, mapped] = kinds[(k + i) % kinds.size()];
                    const auto &entry = obj.at("entries")[i], &child = c.at("children")[i];
                    valid = valid && child.at("resource_type") == saved &&
                            child.at("local_resource_id") == 101 + i &&
                            child.at("offset").at("value") == -0.5 + i * 1.25 &&
                            entry.at("kind") == (present ? mapped : 6u) &&
                            entry.at("resource_id") == (present ? ids[i] : std::uint64_t(101 + i));
                    if (!invalid_copy) {
                        const auto &field = copy.at("final_entries")[i];
                        const bool fixed = present && i < used;
                        valid =
                            valid && field.at("type_value") == (fixed ? mapped : saved) &&
                            field.at("local_resource_id") ==
                                (fixed ? std::uint32_t(ids[i]) : 101 + i) &&
                            field.at("selection") == (fixed ? "reference_fixup" : "saved_fields");
                    }
                }
                if (invalid_copy)
                    valid = valid && copy.at("status") == "unresolved" &&
                            copy.at("final_entries").is_null() &&
                            copy.at("fixups").size() == first_bad + 1 &&
                            copy.at("fixups").back().at("action") == "rejected_out_of_bounds_fixup";
                else
                    valid = valid && copy.at("status") == "resolved" &&
                            copy.at("fixups").size() == used;
                check(valid,
                      "compound source/order, independent counts, full IDs and copy-field oracle");
            }
    auto b = make(0, 0);
    auto j = read(b).at("compound");
    check(j.at("status") == "decoded" && j.at("initial_object_children").at("entries").empty() &&
              j.at("copy_reader_fields").at("native_copy_bytes") == 232 &&
              j.at("copy_reader_fields").at("unused_slot_storage").at("bytes") == 56,
          "zero logical children still have one physical copied slot");
    b = make(0, 1);
    put(b, 384, std::uint32_t(2005));
    put(b, 388, std::uint32_t(7));
    put(b, 440, std::uint64_t(0xffffffff0000002aull));
    j = read(b).at("compound");
    const auto extra = bytesof(j.at("copy_reader_fields").at("unused_slot_storage"));
    check(j.at("status") == "decoded" && j.at("children").empty() &&
              Reader(extra).at<std::uint32_t>(0) == 1 && Reader(extra).at<std::uint32_t>(4) == 42 &&
              j.at("copy_reader_fields").at("fixups")[0].at("logical_child") == false,
          "a reference can fix an unused physical slot without creating a logical child");
    b = make(0, 0, 384);
    j = read(b).at("compound");
    check(j.at("source_status") == "decoded" && j.at("status") == "partial" &&
              j.at("initial_object_children").at("status") == "resolved" &&
              j.at("copy_reader_fields").at("status") == "unresolved",
          "zero-child object reader and larger native copy require different extents");
    b = make(1, 0, 440);
    j = read(b).at("compound");
    check(j.at("copy_reader_fields").at("status") == "resolved" &&
              j.at("initial_object_children").at("status") == "unresolved" &&
              j.at("status") == "partial",
          "object path reads one reference even when saved reference count is zero");
    b = make(1, 2, 448);
    put(b, 440, std::uint64_t(0x100000002ull));
    j = read(b).at("compound");
    check(j.at("initial_object_children").at("status") == "resolved" &&
              j.at("copy_reader_fields").at("status") == "unresolved" &&
              j.at("copy_reader_fields").at("fixups").size() == 1,
          "truncated second saved reference preserves the completed first fixup");
    b = make(3, 1, 560);
    put(b, 552, std::uint64_t(9));
    j = read(b).at("compound");
    check(j.at("copy_reader_fields").at("status") == "resolved" &&
              j.at("initial_object_children").at("status") == "unresolved" &&
              j.at("initial_object_children").at("entries").size() == 1,
          "object reader retains its known prefix without accepting a missing reference suffix");
    b = make(1, 1);
    put(b, 204, std::uint16_t(176));
    put(b, 384, std::uint32_t(2002));
    put(b, 388, std::uint32_t(5));
    j = read(b).at("compound");
    check(j.at("status") == "decoded" && j.at("reference_source_offset") == 384 &&
              j.at("copy_reader_fields").at("final_entries")[0].at("local_resource_id") == 2002 &&
              j.at("initial_object_children").at("entries")[0].at("resource_id") ==
                  (5ull << 32 | 2002),
          "overlapping reference source remains immutable while the copy buffer is fixed");
    b = make(0, 0);
    put(b, 204, std::uint16_t(65535));
    check(read(b).at("compound").at("status") == "decoded",
          "unused out-of-range reference position is not dereferenced");
    b = make(1, 1);
    put(b, 202, std::uint16_t(0));
    put(b, 384, std::uint32_t(2002));
    put(b, 440, std::uint64_t(0x100000005ull));
    j = read(b).at("compound");
    check(j.at("status") == "decoded" &&
              j.at("copy_reader_fields").at("final_entries")[0].at("local_resource_id") == 5 &&
              j.at("initial_object_children").at("entries")[0].at("resource_id") == 0x100000005ull,
          "saved capacity word does not clamp either native reader's reference use");
    b = make(1, 1);
    put(b, 408, std::numeric_limits<double>::quiet_NaN());
    j = read(b).at("compound");
    check(j.at("children")[0].at("offset").at("value").is_null() &&
              j.at("children")[0].at("offset").at("finite") == false && !j.dump().empty(),
          "nonfinite offset keeps source bits and dumpable metadata");
    put(b, 380, UINT32_MAX);
    check(read(b).at("compound").at("status") == "unresolved", "oversize child count is bounded");
    b = make(2, 2);
    auto records = parse_native(b);
    check(records.size() == 1 && records[0].at("line_style_component") == read(b),
          "automatic raw record decoding exposes the compound views");
    return checks;
}
