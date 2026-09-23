#include "internal.hpp"
#include "proxy_cache_fields.hpp"
#include <lz4/lz4.h>
#include <p3d/proxy_cache.hpp>

unsigned model_edge_cache_tests() {
    using namespace p3d;
    unsigned checks = 0;
    auto check = [&](bool ok, const char *why) {
        ++checks;
        require(ok, why);
    };
    auto put = [](Bytes &b, std::uint64_t n, unsigned width) {
        for (unsigned i = 0; i < width; ++i)
            b.push_back(std::uint8_t(n >> (8 * i)));
    };
    auto append = [](Bytes &a, const Bytes &b) { a.insert(a.end(), b.begin(), b.end()); };
    auto block = [&](const Bytes &b) {
        Bytes out;
        put(out, b.size(), 4);
        append(out, b);
        return out;
    };
    auto string = [&](const Bytes &b) {
        Bytes out;
        put(out, b.size() / 2, 4);
        append(out, b);
        return out;
    };
    auto header = [&](std::uint32_t version = 51) {
        Bytes out;
        put(out, version, 4);
        put(out, 1, 4);
        out.resize(156);
        if (version == 51)
            for (unsigned i = 0; i < 4; ++i)
                append(out, block(Bytes(i, std::uint8_t(i))));
        return out;
    };
    auto envelope = [&](const Bytes &b, unsigned codec = 1) {
        Bytes out;
        put(out, codec, 2);
        put(out, 0x1234, 2); // Codec dispatch uses the low 16 bits only.
        put(out, b.size(), 4);
        if (codec != 3) {
            append(out, b);
        } else {
            Bytes compressed(LZ4_compressBound(static_cast<int>(b.size())));
            const auto size = LZ4_compress_default(reinterpret_cast<const char *>(b.data()),
                                                   reinterpret_cast<char *>(compressed.data()),
                                                   static_cast<int>(b.size()),
                                                   static_cast<int>(compressed.size()));
            require(size > 0, "fixture compression");
            compressed.resize(size);
            append(out, compressed);
        }
        return out;
    };
    auto attribute = [](unsigned index, const Bytes &b) {
        return Json{{"group", 21}, {"key", 22762}, {"index", index}, {"payload", rawbytes(b)}};
    };
    auto model = [&](unsigned children, bool with_sections = false, const Bytes &metadata = Bytes{},
                     const Bytes &parameters = Bytes(328, 9)) {
        Bytes h;
        put(h, 51, 4);
        put(h, 0x87654321, 4);
        put(h, UINT64_MAX, 8);
        h.resize(h.size() + 96, 7);
        append(h, string(Bytes{'A', 0, 0, 0}));
        append(h, string({}));
        append(h, parameters);
        h.resize(h.size() + 32, 9);
        put(h, children, 4);
        append(h, block(metadata));
        auto b = block(h);
        Bytes table;
        put(table, 3, 4);
        for (unsigned key : {UINT32_MAX, 0u, UINT32_MAX}) {
            put(table, key, 4);
            append(table, string(Bytes{std::uint8_t(table.size()), 0, 0, 0}));
        }
        append(b, block(table));
        if (with_sections) {
            for (const auto &[first, second] : std::vector<std::pair<std::int32_t, std::uint32_t>>{
                     {0, 99}, {-1, UINT32_MAX}, {0, 88}, {1, UINT32_MAX}, {1, 0}}) {
                Bytes section;
                put(section, std::uint32_t(first), 4);
                put(section, second, 4);
                // One actual nested component, containing a point-list command.
                Bytes commands{2, 0};
                put(commands, 3, 2);
                put(commands, 32, 4);
                put(commands, 1, 4);
                for (double value : {10., 20., 30.}) {
                    std::uint64_t bits;
                    std::memcpy(&bits, &value, 8);
                    put(commands, bits, 8);
                }
                Bytes graphic(14);
                append(graphic, commands);
                Bytes group;
                put(group, 1, 4);
                append(group, block(graphic));
                Bytes component(16);
                append(component, block({}));
                append(component, block(group));
                append(component, block({}));
                Bytes registry;
                put(registry, 1, 4);
                put(registry, 42, 8);
                append(registry, block(component));
                append(section, block(registry));
                append(b, block(section));
            }
        }
        return block(b);
    };
    const auto root = model(2, true), branch = model(1), leaf = model(0);
    Json input = Json::array({attribute(65535, Bytes{1, 2}), attribute(3, envelope(leaf)),
                              attribute(0, header()), attribute(1, envelope(root, 3)),
                              attribute(4, envelope(leaf, 2)), attribute(2, envelope(branch)),
                              attribute(65534, Bytes(80, 8)), attribute(99, Bytes{1})});
    const auto decoded = decode_native_model_edge_cache(input);
    check(decoded.at("status") == "decoded", "full cache structure decoded");
    check(decoded.at("semantics_status") == "partial", "cache semantics not overstated");
    check(decoded.at("header").at("cache_state_hash").at("source_offset") == 8 &&
              decoded.at("header").at("cache_state_hash").at("mode") == 0,
          "main header hash is decoded at its native source location");
    check(decoded.at("runtime_attachment_status") == "not_evaluated",
          "no invented runtime targets");
    check(decoded.at("models").size() == 4, "all preorder model attributes read");
    check(decoded.at("next_model_attribute_index") == 5, "counter follows recursion");
    const auto &models = decoded.at("models");
    const auto &state_hash = models[0].at("reference_state_hash");
    check(state_hash.at("digest_status") == "declared_size_exceeds_storage" &&
              state_hash.at("declared_digest_bytes") == 0x09090909u,
          "invalid saved model hash does not erase the decodable geometry structure");
    check(slice(root, state_hash.at("model_source_offset").get<std::size_t>(), 32) ==
              bytesof(models[0].at("source_block_32")),
          "reference hash source offset resolves to preserved model bytes");
    check(models[0].at("children") == Json({1, 3}) && models[1].at("children") == Json({2}),
          "child counts reconstruct hierarchy, not numeric ID sorting");
    check(models[0].at("parent_model_index").is_null() && models[2].at("parent_model_index") == 1,
          "parent model index retained");
    check(models[0].at("source_link_id") == UINT64_MAX &&
              models[1].at("source_link_id") == UINT64_MAX,
          "repeated full width link IDs do not merge models");
    check(models[0].at("source_strings")[0].at("text") == "A", "model UTF16 source string");
    check(models[0].at("string_table").at("selected_entries")[1].at("source_entry_index") == 2,
          "string table last duplicate wins");
    const auto &selected = models[0].at("selected_sections");
    check(selected.size() == 4 && selected[0].at("first_key") == -1,
          "signed first section key and zero-key equivalence");
    check(selected[1].at("second_key") == 99 && selected[1].at("key_source_section_index") == 0 &&
              selected[1].at("value_source_section_index") == 2,
          "zero first key keeps first key storage but last value");
    check(selected[2].at("second_key") == 0 && selected[3].at("second_key") == UINT32_MAX,
          "unsigned second section key");
    const auto &registry = models[0].at("sections")[0].at("registry");
    check(registry.at("entries")[0]
                  .at("component")
                  .at("graphics")[0]
                  .at("geometry")
                  .at("commands")[0]
                  .at("decoded")
                  .at("points")[0] == Json({10., 20., 30.}),
          "outer attribute connects to proxy geometry");
    check(decoded.at("unconsumed_cache_source_ordinals") == Json({7}) &&
              decoded.at("association_cache").at("status") == "not_evaluated" &&
              bytesof(decoded.at("association_cache").at("source_storage")) == Bytes({1, 2}),
          "bad optional associations retain their source without discarding models");
    check(decoded.at("display_parameter_lookup").at("status") == "source_available",
          "80-byte separate display parameters accepted");
    check(models[0].at("envelope").at("source_flags") == 0x1234 &&
              models[0].at("envelope").at("codec") == "lz4",
          "native codec header");

    const auto single = [&](const Bytes &body) {
        return Json::array({attribute(0, header()), attribute(1, envelope(body))});
    };
    for (std::size_t n = 0; n < leaf.size(); ++n)
        check(decode_native_model_edge_cache(single(Bytes(leaf.begin(), leaf.begin() + n)))
                      .at("status") == "not_evaluated",
              "every model truncation rejected");
    const auto h = header();
    for (std::size_t n = 0; n < h.size(); ++n)
        check(decode_native_model_edge_cache(
                  Json::array({attribute(0, Bytes(h.begin(), h.begin() + n)),
                               attribute(1, envelope(leaf))}))
                      .at("status") == "not_evaluated",
              "every main header truncation rejected");
    auto broken = input;
    broken.erase(1); // Missing index 3; index 4 must not be substituted.
    const auto missing = decode_native_model_edge_cache(broken);
    check(missing.at("status") == "not_evaluated" && !missing.contains("models") &&
              missing.at("reason") == "missing_edge_cache_model_attribute_3",
          "missing child aborts without presenting a partial tree as complete");
    ModelEdgeCacheLimits limits;
    limits.max_models = 3;
    check(decode_native_model_edge_cache(input, limits).at("reason") ==
              "edge_cache_model_count_limit",
          "global model budget");
    limits = {};
    limits.proxies.max_depth = 2;
    check(decode_native_model_edge_cache(input, limits).at("reason") ==
              "edge_cache_model_depth_limit",
          "model recursion depth budget");
    limits = {};
    limits.max_total_model_bytes = root.size() + branch.size();
    check(decode_native_model_edge_cache(input, limits).at("reason") ==
              "edge_cache_decompressed_byte_limit",
          "global decompression budget");
    limits = {};
    limits.proxies.max_entries = 18; // Root consumes 3 strings + 5*(section, entity, graphic).
    check(decode_native_model_edge_cache(input, limits).at("reason") == "proxy_cache_entry_limit",
          "proxy budget is shared across models");
    for (unsigned version : {15u, 50u, 52u}) {
        const auto result =
            decode_native_model_edge_cache(Json::array({attribute(0, header(version))}));
        check(result.at("status") == "decoded" && result.at("models").empty() &&
                  result.at("model_read_status") == "skipped_version_mismatch",
              "other accepted versions return header without reading model attributes");
    }
    check(decode_native_model_edge_cache(Json::array({attribute(0, header(14))})).at("status") ==
              "ignored",
          "old version ignored");
    check(decode_native_model_edge_cache(Json::array()).at("status") == "absent", "absent cache");
    auto duplicate = single(leaf);
    duplicate.push_back(attribute(1, envelope(leaf, 2)));
    auto result = decode_native_model_edge_cache(duplicate);
    check(result.at("models")[0].at("source_ordinal") == 2, "small collection reverse lookup");
    for (unsigned i = 0; i < 3; ++i)
        duplicate.push_back({{"group", 0}, {"key", i}, {"index", 0}, {"payload", rawbytes({})}});
    result = decode_native_model_edge_cache(duplicate);
    check(result.at("models")[0].at("source_ordinal") == 1,
          "complete collection changes native duplicate lookup; do not prefilter");
    auto stored = envelope(leaf);
    stored.push_back(77);
    result =
        decode_native_model_edge_cache(Json::array({attribute(0, header()), attribute(1, stored)}));
    check(bytesof(result.at("models")[0].at("envelope").at("ignored_suffix")) == Bytes{77},
          "stored envelope native length consumption");
    auto trailing = leaf;
    trailing.push_back(88);
    result = decode_native_model_edge_cache(single(trailing));
    check(bytesof(result.at("models")[0].at("trailing_storage")) == Bytes{88},
          "model suffix preserved");
    auto wrong_codec = single(leaf);
    wrong_codec[1] = attribute(1, envelope(leaf, 4));
    check(decode_native_model_edge_cache(wrong_codec).at("reason") ==
              "unsupported_edge_cache_codec",
          "no guessed codec fallback");
    auto compressed = envelope(leaf, 3);
    compressed.pop_back();
    check(decode_native_model_edge_cache(
              Json::array({attribute(0, header()), attribute(1, compressed)}))
                  .at("reason") == "edge_cache_lz4_size_or_framing_mismatch",
          "truncated LZ4 rejected");
    auto bad_string = leaf;
    // Outer length + header length + version/word/link/96-byte block = 120.
    bad_string[120] = 1;
    bad_string[124] = 'A';
    bad_string[125] = 0;
    check(decode_native_model_edge_cache(single(bad_string)).at("reason") ==
              "unterminated_edge_cache_string",
          "unterminated bounded source string rejected");
    auto display = single(leaf);
    display.push_back(attribute(65534, Bytes(79)));
    check(decode_native_model_edge_cache(display).at("display_parameter_lookup").at("status") ==
              "host_fallback_required",
          "wrong-length display field needs actual host fallback");
    Bytes metadata(496, 0), parameters(328, 0);
    auto set = [](Bytes &data, std::size_t offset, auto value) {
        require(offset + sizeof(value) <= data.size(), "cache test fixture extent");
        std::memcpy(data.data() + offset, &value, sizeof(value));
    };
    set(metadata, 0, std::uint16_t(47));
    set(metadata, 4, std::uint32_t(248));
    set(metadata, 8, std::uint32_t(248));
    set(metadata, 12, std::uint32_t(32));
    set(metadata, 32, std::uint16_t(8));
    set(metadata, 64, std::uint32_t(17));
    set(metadata, 68, std::uint32_t(0xc00)); // Drawing compatibility + current-model-last.
    set(metadata, 72, std::uint32_t(17));
    set(metadata, 76, std::uint32_t(9));
    set(metadata, 80, 1000.);
    set(metadata, 88, 1.);
    set(metadata, 96, 100.);
    set(metadata, 104, 1.);
    set(metadata, 224, 1.);
    set(metadata, 232, 1000.);
    set(metadata, 240, 1.);
    set(metadata, 400, 1.);
    set(metadata, 408, 2.);
    set(metadata, 416, 3.);
    set(metadata, 488, UINT64_MAX);
    // One preserved generic linkage, located after the prefix-free model base.
    put(metadata, 0x1003, 2);
    put(metadata, 0x1234, 2);
    put(metadata, 0x12345678, 4);
    set(metadata, 4, std::uint32_t(252));
    set(parameters, 0, std::uint32_t(0x12348000));
    set(parameters, 4, std::uint32_t(0x80400000));
    for (unsigned i = 0; i < 3; ++i) {
        set(parameters, 104 + i * 8, double((i + 1) * 10));
        set(parameters, 128 + i * 8, double(i + 1));
        set(parameters, 168 + (3 * i + i) * 8, double(i + 3));
    }
    set(parameters, 160, 2.);
    const auto source_metadata = metadata, source_parameters = parameters;
    const auto extended =
        decode_native_model_edge_cache(single(model(0, false, metadata, parameters)));
    check(extended.at("status") == "decoded", "metadata and reference fields decode within cache");
    const auto &cache_model = extended.at("models")[0];
    const auto &meta = cache_model.at("model_metadata");
    check(meta.at("status") == "decoded",
          "optional object is native type47/subtype32 model header");
    check(meta.at("semantics_status") == "partial", "remaining header semantics stay explicit");
    const auto &record = meta.at("record");
    check(record.at("length") == metadata.size() && record.at("base_length") == 496 &&
              bytesof(record.at("data")) == Bytes(metadata.begin(), metadata.begin() + 496),
          "synthetic framing never escapes as source bytes or lengths");
    check(record.at("links")[0].at("offset") == 496 && record.at("links")[0].at("app") == 0x1234,
          "linkages retain native offsets and source bytes");
    const auto &units = record.at("model_unit_state");
    check(units.at("factors").at("meters_per_data_unit").at("value") == .001 &&
              units.at("factors").at("material_projection_unit_factor").at("value") == 1.,
          "cache model units reuse confirmed rational conversion");
    check(units.at("data_units_per_storage_unit").at("source_offset") == 224 &&
              units.at("storage_unit").at("source_offsets").at("packed_flags") == 64 &&
              units.at("factors").at("data_units_per_meter").at("source_offsets") ==
                  Json({224, 232, 240}) &&
              !units.at("source_offsets_include_stream_prefix").get<bool>(),
          "scalar, named and list source offsets all refer to prefix-free metadata");
    const auto &coordinates = record.at("model_coordinate_state");
    check(coordinates.at("model_kind").at("value") == 2 &&
              coordinates.at("reference_origin").at("value") == Json({1., 2., 0.}) &&
              coordinates.at("reference_origin").at("source_value") == Json({1., 2., 3.}),
          "cache model kind compatibility and disabled Z match native load");
    check(coordinates.at("flags_source_offset") == 68 &&
              record.at("model_view_state").at("flags_source_offset") == 68 &&
              record.at("model_view_state").at("current_model_last") == true &&
              record.at("model_layer_group_reference").at("source_offset") == 488 &&
              record.at("model_layer_group_reference").at("table_id") == UINT64_MAX,
          "view flag and layer-group reference retain native meaning and full ID width");
    const auto &reference = cache_model.at("reference_parameters");
    check(reference.at("status") == "decoded" &&
              reference.at("target_application_status") == "not_evaluated",
          "decoded saved parameters do not assert a target was constructed");
    check(reference.at("origin_inputs").at("primary_flags") == 0x12348000u &&
              reference.at("origin_inputs").at("secondary_flags") == 0x80400000u,
          "reference flags are directly restored without ordinary-loader filtering");
    check(reference.at("affine_inputs").at("reference_point").at("source_offset") == 128 &&
              reference.at("affine_inputs").at("translation_point").at("source_offset") == 104,
          "reference points map through verified runtime slots");
    check(reference.at("transform").at("matrix") ==
                  Json({{3., 0., 0.}, {0., 4., 0.}, {0., 0., 5.}}) &&
              reference.at("transform").at("scale") == 2. &&
              reference.at("transform").at("normalization_applied") == false,
          "nonunit cached matrix is copied without type13 column normalization");
    ReferenceAffineContext context;
    context.force_z_scale = true;
    context.provider_id = 0;
    context.origin.model_attached = false;
    const auto affine = reference_affine_transform(reference, context);
    check(affine.at("status") == "computed" &&
              affine.at("matrix") ==
                  Json({{6., 0., 0., 4.}, {0., 8., 0., 4.}, {0., 0., 10., 0.}, {0., 0., 0., 1.}}),
          "explicitly selected cached input feeds existing affine query without renormalization");
    check(metadata == source_metadata && parameters == source_parameters &&
              bytesof(cache_model.at("optional_object_storage")) == metadata &&
              bytesof(cache_model.at("reference_parameters_storage")) == parameters,
          "source and retained bytes remain unchanged");
    const auto metadata_offset = meta.at("model_source_offset").get<std::size_t>();
    const auto parameters_offset = reference.at("model_source_offset").get<std::size_t>();
    const auto model_data = model(0, false, metadata, parameters);
    check(slice(model_data, metadata_offset, metadata.size()) == metadata &&
              slice(model_data, parameters_offset, parameters.size()) == parameters,
          "local semantic offsets can be resolved into decompressed model input");
    check(cached_model_metadata({}).at("status") == "absent",
          "empty optional metadata stays absent");
    for (std::size_t n = 1; n < metadata.size(); ++n)
        check(cached_model_metadata(Bytes(metadata.begin(), metadata.begin() + n)).at("status") ==
                  "not_evaluated",
              "all truncated optional records rejected independently");
    auto unsupported = metadata;
    set(unsupported, 12, std::uint32_t(8));
    const auto partial =
        decode_native_model_edge_cache(single(model(0, false, unsupported, parameters)));
    check(partial.at("status") == "decoded" &&
              partial.at("models")[0].at("model_metadata").at("reason") ==
                  "unsupported_cached_model_header_type",
          "unknown optional record does not erase other cache fields or claim complete metadata");
    set(parameters, 160, 0.);
    auto zero = cached_reference_parameters(parameters);
    check(zero.at("transform").at("scale") == 0. &&
              zero.at("transform").at("zero_scale_default_applied") == false,
          "zero cache scale is not replaced by one");
    set(parameters, 168, std::uint64_t(0x7ff8000000001234));
    auto nonfinite = cached_reference_parameters(parameters);
    check(nonfinite.at("status") == "decoded" &&
              nonfinite.at("transform").at("status") == "not_evaluated" &&
              nonfinite.at("transform").at("matrix")[0][0].at("ieee754_hex") == "7ff8000000001234",
          "nonfinite cached transform preserves bits without computed affine claims");
    for (std::size_t n = 0; n < 328; ++n)
        check(cached_reference_parameters(Bytes(parameters.begin(), parameters.begin() + n))
                      .at("status") == "not_evaluated",
              "truncated packed reference field block rejected");
    // The association attribute is independent of the model attribute tree.
    Bytes directory;
    put(directory, 3, 4);
    for (const auto &path : std::vector<std::vector<std::uint64_t>>{{}, {11, UINT64_MAX}, {11}}) {
        put(directory, path.size(), 2);
        for (const auto id : path)
            put(directory, id, 8);
        append(directory, Bytes(32));
    }
    Bytes groups;
    put(groups, 3, 4);
    const auto group = [&](std::uint32_t reference, const std::vector<std::uint64_t> &states,
                           const std::vector<std::uint32_t> &links) {
        put(groups, reference, 4);
        put(groups, states.size(), 2);
        for (const auto bits : states) {
            put(groups, UINT64_MAX, 8);
            put(groups, bits, 8);
        }
        put(groups, links.size(), 4);
        for (const auto link : links)
            put(groups, link, 4);
    };
    group(1, {0x3ff0000000000000ULL, 0x7ff8000000001234ULL}, {1, 1, 0, UINT32_MAX});
    group(0, {0x7ff0000000000000ULL}, {2});
    group(99, {0x8000000000000000ULL}, {});
    Bytes association_bytes(32);
    append(association_bytes, block(directory));
    const auto association_section = association_bytes.size();
    append(association_bytes, block(groups));
    const auto associations = [&](const Bytes &b, unsigned codec = 1,
                                  ModelEdgeCacheLimits bound = {}) {
        return decode_native_edge_cache_associations(
            Json::array({attribute(65535, envelope(b, codec))}), bound);
    };
    const auto assoc = associations(association_bytes);
    check(assoc.at("status") == "decoded" && assoc.at("semantics_status") == "partial" &&
              assoc.at("runtime_attachment_status") == "not_evaluated",
          "standalone associations decoded without an invented runtime attachment");
    const auto &refs = assoc.at("reference_directory");
    check(refs[0].at("reference_link_path").empty() &&
              refs[1].at("reference_link_path") == Json({std::uint64_t(11), UINT64_MAX}) &&
              refs[2].at("reference_link_path") == Json({11}),
          "root and descendant paths retain order and model-local IDs");
    check(refs[0].at("reference_state_hash").at("source_offset") == 42 &&
              assoc.at("reference_directory_end_offset") == association_section,
          "directory offsets address the decompressed payload");
    const auto &records = assoc.at("associations");
    check(records.size() == 3 && records[0].at("entities").size() == 2 &&
              records[0].at("entities")[0].at("entity_id") == UINT64_MAX &&
              records[0].at("entities")[1].at("entity_id") == UINT64_MAX,
          "duplicate full-width entity IDs are not merged");
    check(records[0].at("entities")[0].at("source_state_value") == 1. &&
              records[0].at("entities")[1].at("source_state_value").is_null() &&
              records[1].at("entities")[0].at("source_state_value").is_null(),
          "nonfinite state values use null with original bytes preserved");
    const auto state_offset =
        records[0].at("entities")[1].at("source_offset").get<std::size_t>() + 8;
    check(bytesof(records[0].at("entities")[1].at("source_state_storage")) ==
                  slice(association_bytes, state_offset, 8) &&
              std::signbit(records[2].at("entities")[0].at("source_state_value").get<double>()),
          "NaN payload and negative zero remain lossless");
    check(records[0].at("links").size() == 4 &&
              records[0].at("links")[0].at("association_index") == 1 &&
              records[0].at("links")[1].at("association_index") == 1 &&
              records[0].at("links")[2].at("self_reference") == true &&
              records[0].at("links")[3].at("source_entry_present") == false &&
              records[1].at("links")[0].at("source_entry_present") == true &&
              records[2].at("reference_directory_entry_present") == false,
          "forward, repeated, self and missing links stay source associations");
    for (unsigned codec : {1u, 2u, 3u}) {
        const auto value = associations(association_bytes, codec);
        check(value.at("status") == "decoded" && value.at("associations") == records &&
                  value.at("reference_directory") == refs,
              "all native codecs produce identical association content");
    }
    for (std::size_t n = 0; n < association_bytes.size(); ++n)
        check(associations(Bytes(association_bytes.begin(), association_bytes.begin() + n))
                      .at("status") == "not_evaluated",
              "every truncated association payload rejected");
    auto padded_groups = groups;
    append(padded_groups, Bytes{4, 5});
    Bytes padded(32);
    append(padded, block(directory));
    append(padded, block(padded_groups));
    append(padded, Bytes{6, 7, 8});
    const auto padding = associations(padded);
    check(padding.at("status") == "decoded" && padding.at("associations") == records &&
              padding.at("consumed_bytes") == association_bytes.size() &&
              bytesof(padding.at("ignored_section_storage")) == Bytes({4, 5}) &&
              bytesof(padding.at("trailing_storage")) == Bytes({6, 7, 8}),
          "unread association section and outside tail have separate native extents");
    auto padded_directory = directory;
    padded_directory.push_back(0);
    Bytes invalid_directory(32);
    append(invalid_directory, block(padded_directory));
    append(invalid_directory, block(groups));
    check(associations(invalid_directory).at("reason") ==
              "edge_cache_reference_directory_length_mismatch",
          "reference directory requires exact count and length agreement");
    for (const auto offset : {std::size_t(32), association_section}) {
        auto huge = association_bytes;
        set(huge, offset, UINT32_MAX);
        check(associations(huge).at("status") == "not_evaluated",
              "oversized section cannot escape decoded payload");
        set(huge, offset, std::uint32_t(4));
        check(associations(huge).at("status") == "not_evaluated",
              "count cannot consume bytes from the next section");
    }
    limits = {};
    limits.proxies.max_entries = 17;
    check(associations(association_bytes, 1, limits).at("reason") == "proxy_cache_entry_limit",
          "one shared budget covers directory, path, entities and links");
    limits.proxies.max_entries = 18;
    check(associations(association_bytes, 1, limits).at("status") == "decoded",
          "exact entry budget accepted");
    limits.proxies.max_depth = 1;
    check(associations(association_bytes, 1, limits).at("reason") ==
              "edge_cache_reference_path_depth_limit",
          "reference path depth is bounded");
    limits = {};
    limits.max_total_model_bytes = association_bytes.size() - 1;
    check(associations(association_bytes, 3, limits).at("reason") ==
              "edge_cache_decompressed_byte_limit",
          "association decompression obeys total byte limit");
    auto combined = single(leaf);
    combined.push_back(attribute(65535, envelope(association_bytes, 3)));
    const auto full = decode_native_model_edge_cache(combined);
    check(full.at("status") == "decoded" &&
              full.at("association_cache").at("associations") == records &&
              full.at("total_decoded_model_bytes") == leaf.size() &&
              full.at("total_decoded_cache_bytes") == leaf.size() + association_bytes.size() &&
              full.at("unconsumed_cache_source_ordinals").empty(),
          "model and association stages share sources and distinguish byte totals");
    limits = {};
    limits.max_total_model_bytes = leaf.size() + association_bytes.size() - 1;
    const auto limited = decode_native_model_edge_cache(combined, limits);
    check(limited.at("status") == "decoded" && limited.at("models").size() == 1 &&
              limited.at("association_cache").at("reason") == "edge_cache_decompressed_byte_limit",
          "association byte budget includes previously decoded models without erasing them");
    limits = {};
    limits.proxies.max_entries = 20; // The model already consumed 3 string entries.
    check(decode_native_model_edge_cache(combined, limits).at("association_cache").at("reason") ==
              "proxy_cache_entry_limit",
          "association entry budget includes previously decoded model entries");
    check(decode_native_edge_cache_associations(Json::array()).at("status") == "absent" &&
              decode_native_edge_cache_associations(Json::object()).at("status") == "not_evaluated",
          "absent attribute and invalid collection are distinct");
    for (unsigned count : {5u, 19u, 20u, 21u, 30u, 65u}) {
        Json attrs = Json::array();
        attrs.push_back(attribute(65535, envelope(association_bytes)));
        for (unsigned i = 0; i < count; ++i)
            attrs.push_back(
                {{"group", i % 25}, {"key", i}, {"index", 0}, {"payload", rawbytes({})}});
        attrs.push_back(attribute(65535, envelope(padded)));
        const auto selected = decode_native_edge_cache_associations(attrs);
        std::size_t ordinal = 0;
        const auto lookup = native_attribute_lookup(attrs);
        for (const auto &key : lookup.at("keys"))
            if (key.at("group") == 21 && key.at("key") == 22762)
                ordinal = key.at("selected_source_ordinal").get<std::size_t>();
        check(selected.at("status") == "decoded" && selected.at("source_ordinal") == ordinal &&
                  selected.at("source_storage") == attrs[ordinal].at("payload"),
              "association selection uses the complete collection across native sort thresholds");
    }
    return checks;
}
