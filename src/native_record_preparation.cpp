#include "native_record_preparation.hpp"
#include <p3d/native_input.hpp>

namespace p3d {
namespace {
template <class T> void put(Bytes &b, std::size_t offset, T value) {
    require(offset <= b.size() && sizeof(T) <= b.size() - offset,
            "prepared_native_field_outside_record");
    std::memcpy(b.data() + offset, &value, sizeof(T));
}
Json matrix3(const Json &matrix) {
    Json out = Json::array();
    for (unsigned r = 0; r < 3; ++r)
        out.push_back(Json::array({matrix.at(r).at(0), matrix.at(r).at(1), matrix.at(r).at(2)}));
    return out;
}
Json matrix_input(const Bytes &source) {
    Reader(source, 164).need(72);
    // The native input branch consumes only nine matrix doubles. Reuse the
    // established matrix repair with a synthetic zero translation, which is
    // independent of its arithmetic. Never read or replace source translation.
    Bytes isolated(260, 0);
    std::copy_n(source.begin() + 164, 72, isolated.begin() + 164);
    auto result = native_block_transform(isolated);
    result["scope"] = "native_type_62_nine_value_matrix_input";
    result["source_matrix"] = matrix3(result.at("source_matrix"));
    result["matrix"] = matrix3(result.at("matrix"));
    result.erase("translation_source_offset");
    return result;
}
} // namespace

Json prepare_native_record_input(const Bytes &source, std::uint64_t assigned_id,
                                 std::uint32_t descendants, bool child, bool system) {
    Json out = {{"scope", "initial_record_layout_header_and_id_input"},
                {"status", "not_evaluated"},
                {"source_record", rawbytes(source)},
                {"prepared_record", nullptr},
                {"decoded_record", nullptr},
                {"decoded_status", "not_evaluated"},
                {"runtime_callbacks", "not_evaluated"},
                {"descriptor_conversion", "not_evaluated"}};
    try {
        Reader r(source);
        r.need(36);
        const auto words = r.at<std::uint32_t>(8), base_words = r.at<std::uint32_t>(12);
        require(source.size() == 4 + std::uint64_t(words) * 2,
                "complete_native_input_record_required");
        Json record = {{"element_type", r.at<std::uint16_t>(4)},
                       {"element_flags", r.at<std::uint16_t>(6)},
                       {"data", rawbytes(slice(source, 0, 36))}};
        const auto filter = native_record_input_filter(record, source);
        out["input_filter"] = filter;
        if (filter.at("status") != "accepted") {
            out["status"] = filter.at("status");
            return out;
        }
        require(base_words >= 16 && base_words <= words, "native_input_base_length_invalid");
        const auto base_size = 4 + std::size_t(base_words) * 2;
        const auto base = slice(source, 0, base_size);
        record["data"] = rawbytes(base);
        const auto topology = native_record_input_topology(record, base);
        out["source_topology"] = topology;
        auto prepared = source;
        auto conversion = topology.value("conversion", Json());
        const auto type = r.at<std::uint16_t>(4);
        if (type == 24 && conversion.at("applied") == true) {
            prepared.erase(prepared.begin() + 36, prepared.begin() + 108);
        } else if (type == 13) {
            prepared = native_reference_layout(source).data;
        } else if (type == 62) {
            auto repaired = matrix_input(base);
            for (unsigned row = 0; row < 3; ++row)
                for (unsigned column = 0; column < 3; ++column)
                    put(prepared, 164 + (row * 3 + column) * 8,
                        repaired.at("matrix").at(row).at(column).get<double>());
            conversion["matrix_input"] = std::move(repaired);
            conversion["payload_reconstruction"] = "matrix_repaired";
        }
        const auto header = native_list_record_header(
            record, conversion, child, topology.at("compound").get<bool>(), descendants, system);
        out["header_preparation"] = header;
        require(header.at("status") == "resolved", "native_input_header_not_prepared");
        const auto output_words = header.at("output_record_word_count").get<std::uint32_t>();
        require(prepared.size() == 4 + std::uint64_t(output_words) * 2,
                "prepared_native_layout_length_mismatch");
        put(prepared, 6, header.at("output_element_flags").get<std::uint16_t>());
        put(prepared, 8, output_words);
        put(prepared, 12, header.at("output_base_word_count").get<std::uint32_t>());
        const auto &count = header.at("descendant_count_update");
        if (count.is_object() && count.at("written") == true)
            put(prepared, 4 + count.at("header_offset").get<std::size_t>(),
                count.at("output_value").get<std::uint32_t>());
        put(prepared, 20, assigned_id);
        out["conversion"] = std::move(conversion);
        out["assigned_id"] = assigned_id;
        out["is_child"] = child;
        out["system_container"] = system;
        out["accepted_descendant_count"] = descendants;
        out["native_descriptor_copy_bytes"] = std::min(std::uint32_t(output_words * 2u), 0x1fffeu);
        out["prepared_record"] = rawbytes(prepared);
        out["status"] = "prepared";
        try {
            const auto decoded = parse_native(prepared);
            require(decoded.size() == 1, "prepared_native_record_count_mismatch");
            out["decoded_record"] = decoded[0];
            out["decoded_status"] = "decoded";
        } catch (const std::exception &e) {
            // Input deliberately ignores validator return codes. Retain its
            // resulting bytes even when the strict record reader rejects them.
            out["decoded_status"] = "not_decoded";
            out["decode_reason"] = e.what();
        }
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
