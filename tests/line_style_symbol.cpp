#include "internal.hpp"
#include <p3d/line_style.hpp>
#include <future>

unsigned line_style_symbol_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool value, const char *why) {
        require(value, why);
        ++checks;
    };
    auto put = [](Bytes &b, std::size_t p, auto value) {
        require(p <= b.size() && sizeof(value) <= b.size() - p, "test fixture bounds");
        std::memcpy(b.data() + p, &value, sizeof(value));
    };
    auto make = [&](const Bytes &geometry = Bytes(), std::uint64_t id = 0) {
        Bytes b(544 + geometry.size(), 0);
        put(b, 4, std::uint16_t(49));
        put(b, 8, std::uint32_t((b.size() - 4) / 2));
        put(b, 12, std::uint32_t((b.size() - 4) / 2));
        put(b, 16, std::uint32_t(8));
        put(b, 20, std::uint64_t(0x1234567800000009ull));
        put(b, 70, std::uint16_t(1));
        put(b, 204, std::uint16_t(328 + geometry.size()));
        put(b, 424, 2.5);
        put(b, 532, std::uint32_t(geometry.size()));
        std::copy(geometry.begin(), geometry.end(), b.begin() + 536);
        put(b, 536 + geometry.size(), id);
        return b;
    };
    auto read = [](const Bytes &b) {
        return read_native_line_style_resource({{"data", rawbytes(b)}});
    };
    auto params = [](const Bytes &b, const NativeLineStyleSymbolContext &context = {}) {
        return prepare_native_line_style_symbol_parameters({{"data", rawbytes(b)}}, context);
    };
    auto bits = [](double value) {
        std::uint64_t result;
        std::memcpy(&result, &value, 8);
        return result;
    };
    auto from_bits = [](std::uint64_t value) {
        double result;
        std::memcpy(&result, &value, 8);
        return result;
    };
    auto encoded_bits = [](const Json &value) {
        return Reader(bytesof(value.at("source"))).at<std::uint64_t>(0);
    };
    auto b = make();
    // Every flag survives independently. A named ODA getter confirms only bit
    // zero as dimensionality; the native initializer separately consumes bit one.
    for (unsigned bit = 0; bit < 32; ++bit) {
        const auto flags = std::uint32_t(1) << bit;
        put(b, 528, flags);
        const auto j = read(b).at("point_symbol");
        check(j.at("status") == "decoded" && j.at("symbol_flags") == flags &&
                  j.at("is_3d") == (bit == 0) && j.at("native_option_bit_1") == (bit == 1) &&
                  j.at("unassigned_symbol_flags") == (bit < 2 ? 0u : flags),
              "point symbol flags retain unknown bits without inventing dimensionality");
    }
    const std::array<std::uint64_t, 13> saved_bits{
        {0, 0x8000000000000000ull, 0x3ff0000000000000ull, 0x4004000000000000ull,
         0xbff0000000000000ull, 0x7ff0000000000000ull, 0xfff0000000000000ull, 0x7ff8000000001234ull,
         0x7ff0000000001234ull, 1, 0x8000000000000001ull, 0x7fefffffffffffffull,
         0xffefffffffffffffull}};
    const std::array<double, 9> units{
        {1, 2, 100000, -4, 0., -0., std::numeric_limits<double>::infinity(),
         -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}};
    // Closed-form reference table: positive values bypass every context;
    // nonpositive or unordered values require either an actual factor or
    // confirmed owner absence. Reciprocal values are independently computed.
    for (std::size_t s = 0; s < saved_bits.size(); ++s) {
        const auto saved = from_bits(saved_bits[s]);
        const bool positive = s == 2 || s == 3 || s == 5 || s == 9 || s == 11;
        put(b, 424, saved);
        auto result = params(b);
        check(result.at("status") == (positive ? "computed" : "unresolved") &&
                  encoded_bits(result.at("saved_scale")) == saved_bits[s] &&
                  (positive || result.at("native_scale").is_null()),
              "scale source bits survive and unknown owner never means absent owner");
        NativeLineStyleSymbolContext absent;
        absent.owner_confirmed_absent = true;
        result = params(b, absent);
        check(result.at("status") == "computed" &&
                  encoded_bits(result.at("native_scale")) == (positive ? saved_bits[s] : bits(1.)),
              "confirmed absent owner uses one only when the saved scale does not pass");
        for (const auto factor : units) {
            NativeLineStyleSymbolContext context;
            context.owner_unit_factor = factor;
            result = params(b, context);
            const auto expected = positive ? saved : 1. / factor;
            const auto &scale = result.at("native_scale");
            const auto actual = from_bits(encoded_bits(scale));
            check(
                result.at("status") == "computed" &&
                    result.at("selection") ==
                        (positive ? "positive_saved_scale" : "reciprocal_owner_unit_factor") &&
                    scale.at("finite") == std::isfinite(expected) &&
                    (std::isnan(expected) ? std::isnan(actual) : bits(actual) == bits(expected)) &&
                    (std::isfinite(expected) ? scale.at("value") == expected
                                             : scale.at("value").is_null()),
                "owner reciprocal preserves IEEE zero infinity sign and NaN semantics");
            context.owner_confirmed_absent = true;
            check(params(b, context).at("status") == (positive ? "computed" : "unresolved"),
                  "conflicting owner context is rejected unless the source bypasses it");
        }
    }
    b = make();
    const std::array<double, 10> fields{{-5, -6, -7, 8, 9, 10, 111, 222, 333, 2.5}};
    for (std::size_t i = 0; i < fields.size(); ++i)
        put(b, 352 + i * 8, fields[i]);
    for (std::size_t i = 336; i < 352; ++i)
        b[i] = std::uint8_t(i);
    for (std::size_t i = 432; i < 528; ++i)
        b[i] = std::uint8_t(i);
    auto j = read(b).at("point_symbol");
    bool correct = true;
    for (unsigned i = 0; i < 3; ++i) {
        correct = correct && j.at("range_low").at("components")[i].at("value") == fields[i] &&
                  j.at("range_high").at("components")[i].at("value") == fields[3 + i] &&
                  j.at("offset").at("components")[i].at("value") == fields[6 + i];
    }
    check(correct && bytesof(j.at("source_payload").at("data")) == slice(b, 208, b.size() - 208) &&
              bytesof(j.at("unassigned_ranges")[0].at("data")) == slice(b, 336, 16) &&
              bytesof(j.at("unassigned_ranges")[1].at("data")) == slice(b, 432, 96) &&
              !j.at("initial_object_parameters").contains("offset"),
          "bounds offset reserved values and initializer scope remain distinct");
    const std::array<std::uint64_t, 4> ids{{0, 0x100000000ull, 0xfedcba9876543210ull, UINT64_MAX}};
    for (const auto count : {0u, 1u, 65535u})
        for (const auto id : ids) {
            b = make({}, id);
            put(b, 200, std::uint16_t(count));
            put(b, 202, std::uint16_t(0));
            const auto reference = read(b).at("point_symbol").at("geometry_reference");
            check(reference.at("status") == "decoded" && reference.at("source_id") == id &&
                      reference.at("selection") ==
                          (id ? "request_owner_geometry" : "no_geometry_request") &&
                      reference.at("target_resolution") == "not_evaluated",
                  "geometry reference uses full first ID independently of the saved count and "
                  "capacity");
        }
    b = make();
    put(b, 204, std::uint16_t(144));
    put(b, 352, ids[2]);
    check(read(b).at("point_symbol").at("geometry_reference").at("source_id") == ids[2],
          "reference position follows its saved offset even when overlapping fixed fields");
    for (const auto size : {532u, 534u, 536u, 542u}) {
        b = make();
        b.resize(size);
        j = read(b).at("point_symbol");
        check(j.at("source_status") == "decoded" && j.at("status") == "partial" &&
                  j.at("geometry_reference").at("status") == "unresolved" &&
                  j.at("embedded_geometry").at("status") ==
                      (size >= 536 ? "decoded" : "unresolved"),
              "fixed fields embedded length and first reference have independent bounds");
    }
    b = make();
    b.resize(531);
    check(read(b).at("point_symbol").at("source_status") == "unresolved" &&
              params(b).at("status") == "unresolved",
          "truncated scalar input is not accepted");
    // Build native records directly from the documented file layout, then
    // omit their four-byte framing exactly as the native buffer writer does.
    auto native = [&](unsigned type, std::size_t size, std::uint64_t id) {
        Bytes raw(size, 0);
        put(raw, 4, std::uint16_t(type));
        put(raw, 8, std::uint32_t((size - 4) / 2));
        put(raw, 12, std::uint32_t((size - 4) / 2));
        put(raw, 20, id);
        return raw;
    };
    auto line = native(37, 156, ids[2]);
    const std::array<double, 6> coords{{-1.5, 2, 3, 4, 5.5, -6}};
    for (std::size_t i = 0; i < coords.size(); ++i)
        put(line, 108 + i * 8, coords[i]);
    auto point_list = native(40, 164, ids[1]);
    put(point_list, 108, std::uint32_t(2));
    for (std::size_t i = 0; i < coords.size(); ++i)
        put(point_list, 116 + i * 8, coords[i] * 2);
    Bytes buffer(line.begin() + 4, line.end());
    buffer.insert(buffer.end(), point_list.begin() + 4, point_list.end());
    b = make(buffer, ids[3]);
    const auto before = b;
    const auto view = read(b);
    const auto &embedded = view.at("point_symbol").at("embedded_geometry");
    const auto &records = embedded.at("records");
    auto geometry = reconstruct_native_line_style_symbol_buffer({{"data", rawbytes(b)}});
    check(embedded.at("status") == "decoded" && records.size() == 2 &&
              records[0].at("id") == ids[2] && records[1].at("id") == ids[1] &&
              records[0].at("parent_record_source_offset") == 536 &&
              records[1].at("parent_record_source_offset") == 688 &&
              records[1].at("embedded_byte_offset") == 152 &&
              records[1].at("synthetic_stream_prefix") == true &&
              bytesof(records[0].at("data")) == line &&
              bytesof(records[1].at("data")) == point_list &&
              bytesof(embedded.at("source")) == buffer &&
              bytesof(embedded.at("unassigned_suffix").at("data")) == slice(b, 848, 8) &&
              geometry.unknown.empty() && geometry.lines.size() == 2 && b == before,
          "unframed native records retain exact source mapping and extract distinct primitives");
    check(geometry.lines[0] == std::vector<Point3>({{-1.5, 2, 3}, {4, 5.5, -6}}) &&
              geometry.lines[1] == std::vector<Point3>({{-3, 4, 6}, {8, 11, -12}}) &&
              geometry.primitive_ranges.size() == 2 &&
              geometry.primitive_ranges[1].at("start") == 1 &&
              geometry.primitive_ranges[1].at("symbol_record_index") == 1 &&
              view.at("point_symbol").at("geometry_reference").at("source_id") == ids[3],
          "primitive extraction does not apply saved scale offset or unrelated geometry reference");
    auto equal_lines = slice(line, 4, line.size() - 4);
    equal_lines.insert(equal_lines.end(), line.begin() + 4, line.end());
    geometry = reconstruct_native_line_style_symbol_buffer({{"data", rawbytes(make(equal_lines))}});
    check(geometry.lines.size() == 2 && geometry.primitive_ranges.size() == 2 &&
              geometry.lines[0] == geometry.lines[1],
          "equal independent native records are not deduplicated");
    // A valid prefix survives a malformed trailing record, while completeness
    // remains false and the complete original buffer is retained.
    for (const auto words : {0u, 15u, 16u, 65535u, 65536u}) {
        Bytes broken(32, 0);
        put(broken, 4, std::uint32_t(words));
        if (words == 16)
            put(broken, 8, std::uint32_t(15)); // invalid base boundary
        auto mixed = slice(line, 4, line.size() - 4);
        mixed.insert(mixed.end(), broken.begin(), broken.end());
        b = make(mixed);
        j = read(b).at("point_symbol").at("embedded_geometry");
        geometry = reconstruct_native_line_style_symbol_buffer({{"data", rawbytes(b)}});
        check(j.at("status") == "unresolved" && j.at("records").size() == 1 &&
                  bytesof(j.at("source")) == mixed && geometry.lines.size() == 1 &&
                  !geometry.unknown.empty(),
              "malformed native suffix preserves known primitives without claiming complete "
              "geometry");
    }
    for (const auto length : {1u, 30u, 31u, 32u, 153u, UINT32_MAX}) {
        b = make(slice(line, 4, line.size() - 4));
        put(b, 532, std::uint32_t(length));
        check(read(b).at("point_symbol").at("embedded_geometry").at("status") != "decoded",
              "embedded length is bounded and never silently ignores a partial header");
    }
    for (const auto type : {19u, 20u, 21u, 97u, 65535u}) {
        auto raw = native(type, 108, ids[2]);
        b = make(slice(raw, 4, raw.size() - 4));
        geometry = reconstruct_native_line_style_symbol_buffer({{"data", rawbytes(b)}});
        check(!geometry.unknown.empty() && geometry.lines.empty() &&
                  geometry.unknown[0].at("symbol_record_index") == 0,
              "unsupported composites and command records remain explicitly incomplete");
    }
    // Recursively nested components are bounded. A failed nested call must
    // release its guard, and independent threads must have independent guards.
    auto nested = make();
    for (unsigned i = 0; i < 33; ++i)
        nested = make(slice(nested, 4, nested.size() - 4));
    check(read(nested).at("point_symbol").at("embedded_geometry").at("status") == "partial" &&
              read(make()).at("status") == "decoded",
          "nested resource limit is explicit and the recursion guard resets after failure");
    std::array<std::future<bool>, 4> futures;
    for (auto &task : futures)
        task = std::async(std::launch::async, [&] {
            return read(nested).at("point_symbol").at("embedded_geometry").at("status") ==
                       "partial" &&
                   read(make()).at("status") == "decoded";
        });
    for (auto &task : futures)
        check(task.get(), "symbol decoder recursion state is thread local");
    const auto automatic = parse_native(make(slice(line, 4, line.size() - 4)));
    check(automatic.size() == 1 && automatic[0]
                                           .at("line_style_component")
                                           .at("point_symbol")
                                           .at("embedded_geometry")
                                           .at("records")
                                           .size() == 1,
          "automatic resource parsing exposes the same embedded source records");
    check(prepare_native_line_style_symbol_parameters(Json::object()).at("status") ==
                  "unresolved" &&
              !reconstruct_native_line_style_symbol_buffer(Json::object()).unknown.empty(),
          "public symbol APIs report missing input without throwing");
    return checks;
}
