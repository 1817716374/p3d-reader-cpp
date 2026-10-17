#include "internal.hpp"
#include "property_binary_values.hpp"
#include <p3d/property_blocks.hpp>

namespace p3d {
namespace {
struct BlockBudget {
    const PropertyBlockLimits &limits;
    std::size_t tokens = 0, fields = 0, elements = 0, bytes = 0;
};
Json binary_value_views(const Json &block, BlockBudget &budget) {
    const auto &structure = block.at("property_structure"), &tokens = block.at("tokens");
    Json out = {{"scope", "current_property_block_binary_fields"},
                {"status", "partial"},
                {"declarations_complete", structure.at("status") == "structured"},
                {"active_geometry_selection", "not_performed"},
                {"entries", Json::array()}};
    bool complete = structure.at("status") == "structured";
    for (const auto &field : structure.at("fields")) {
        if (field.value("type_name", "") != "binary")
            continue;
        for (std::size_t vi = 0; vi < field.at("value_token_indices").size(); ++vi) {
            const auto ti = field.at("value_token_indices")[vi].get<std::size_t>();
            const auto &token = tokens.at(ti);
            const auto count = token.at("bytes").get<std::size_t>();
            Json view = {{"field_index", field.at("field_index")},
                         {"value_token_index", ti},
                         {"name", field.at("name")},
                         {"namespace", field.at("namespace")},
                         {"source_offset", token.at("offset").get<std::size_t>() + 8},
                         {"source_byte_count", count},
                         {"status", "not_interpreted"}};
            if (field.at("is_array") == true)
                view["array_index"] = vi;
            const auto contract = detail::property_binary_contract(
                field.at("namespace").get<std::string>(), field.at("name").get<std::string>());
            if (contract)
                view["encoding"] = contract->encoding;
            if (!count)
                view["status"] = "empty";
            else if (!contract) {
                view["reason"] = "native_property_binary_contract_not_known";
                complete = false;
            } else {
                try {
                    require(count <= budget.limits.max_decoded_bytes - budget.bytes,
                            "property_binary_value_byte_budget");
                    budget.bytes += count;
                    const auto bytes = unbase64(token.at("binary_base64").get<std::string>());
                    require(bytes.size() == count, "property_binary_value_size_mismatch");
                    view["decoded"] = detail::decode_property_binary_value(
                        contract->codec, bytes, block.at("class_name").get<std::string>());
                    view["status"] = "parsed";
                } catch (const std::bad_alloc &) {
                    throw;
                } catch (const std::exception &e) {
                    view["status"] = "not_parsed";
                    view["reason"] = e.what();
                    complete = false;
                }
            }
            out["entries"].push_back(std::move(view));
        }
    }
    out["status"] = complete ? "parsed" : "partial";
    return out;
}
Json read_block(const Bytes &bytes, BlockBudget &budget, std::size_t depth) {
    require(depth < budget.limits.max_depth, "property_block_depth_budget");
    require(bytes.size() <= budget.limits.max_decoded_bytes - budget.bytes,
            "property_block_byte_budget");
    budget.bytes += bytes.size();
    Reader r(bytes);
    Json tokens = Json::array();
    while (r.left()) {
        require(budget.tokens < budget.limits.max_tokens, "property_block_token_budget");
        ++budget.tokens;
        const auto offset = r.p;
        const auto type = r.u32();
        Json token = {{"offset", offset}, {"type_code", type}};
        if (type == 0) {
            const auto raw = r.u8();
            // Keep the original numeric token view as well as its confirmed type.
            token.update({{"kind", "unassigned_byte"},
                          {"value", raw},
                          {"native_kind", "boolean"},
                          {"boolean_value", raw != 0}});
        } else if (type == 1 || type == 2 || type == 3 || type == 6 || type == 7) {
            const char *format = type == 1   ? "i"
                                 : type == 2 ? "d"
                                 : type == 3 ? "q"
                                 : type == 6 ? "3d"
                                             : "2d";
            const char *kind = type == 1   ? "int32"
                               : type == 2 ? "double"
                               : type == 3 ? "int64"
                               : type == 6 ? "point3d"
                                           : "point2d";
            token["value"] = r.number(format);
            token["kind"] = kind;
            if (type == 2 || type == 6 || type == 7) {
                Json nonfinite = Json::array();
                const auto count = type == 2 ? 1u : type == 6 ? 3u : 2u;
                for (unsigned i = 0; i < count; ++i) {
                    const auto bits = Reader(bytes).at<std::uint64_t>(offset + 4 + 8 * i);
                    if ((bits & 0x7ff0000000000000ull) == 0x7ff0000000000000ull)
                        nonfinite.push_back({{"component", i}, {"bits", bits}});
                }
                if (!nonfinite.empty())
                    token["nonfinite_float_bits"] = std::move(nonfinite);
            }
        } else if (type == 4 || type == 5) {
            const auto data = r.take(r.u32());
            if (type == 5)
                token.update(
                    {{"kind", "binary"}, {"binary_base64", base64(data)}, {"bytes", data.size()}});
            else {
                std::string text, encoding = "utf8";
                try {
                    text = utf8(data);
                } catch (...) {
                    text = gb18030(data);
                    encoding = "gb18030";
                }
                token.update({{"kind", "string"}, {"value", text}, {"encoding", encoding}});
            }
        } else
            throw std::runtime_error("property token type");
        tokens.push_back(std::move(token));
    }
    require(tokens.size() >= 2 && tokens[0]["kind"] == "string" && tokens[1]["kind"] == "string",
            "property block namespace/class");
    Json out = {{"namespace", tokens[0]["value"]},
                {"class_name", tokens[1]["value"]},
                {"tokens", std::move(tokens)},
                {"source_storage", rawbytes(bytes)},
                {"note", "Ordered typed values; native field boundaries are in property_structure. "
                         "Schema defaults and runtime values are not evaluated."}};
    const auto &ts = out.at("tokens");
    Json structure = {{"status", "partial"},
                      {"scope", "native_property_block_wire_layout"},
                      {"schema_instantiation", "not_performed"},
                      {"header_token_indices", Json::array({0, 1})},
                      {"fields", Json::array()}};
    std::size_t cursor = 2;
    auto take = [&](unsigned type) {
        require(cursor < ts.size(), "property_field_token_truncated");
        require(ts[cursor].at("type_code") == type, "property_field_token_type_mismatch");
        return cursor++;
    };
    auto string = [&] { return ts[take(4)].at("value").get<std::string>(); };
    auto value_type = [](const std::string &type) -> std::optional<unsigned> {
        if (type == "boolean")
            return 0;
        if (type == "int")
            return 1;
        if (type == "double")
            return 2;
        if (type == "long")
            return 3;
        if (type == "string")
            return 4;
        if (type == "binary")
            return 5;
        if (type == "point3d")
            return 6;
        if (type == "point2d")
            return 7;
        return {};
    };
    bool complete = true;
    try {
        while (cursor < ts.size()) {
            require(budget.fields < budget.limits.max_fields, "property_block_field_budget");
            ++budget.fields;
            const auto start = cursor;
            Json field = {{"field_index", structure.at("fields").size()},
                          {"first_token_index", start},
                          {"source_offset", ts[start].at("offset")},
                          {"status", "partial"},
                          {"value_token_indices", Json::array()}};
            try {
                field["name"] = string();
                field["namespace"] = string();
                const auto name = field.at("name").get<std::string>();
                if (name == "Placement" || name == "BaseTransform") {
                    field["layout"] = name == "Placement" ? "placement" : "base_transform";
                    const std::array<unsigned, 4> placement{{6, 2, 2, 2}}, transform{{6, 6, 6, 6}};
                    for (const auto type : name == "Placement" ? placement : transform)
                        field["value_token_indices"].push_back(take(type));
                    field["value_status"] = "encoded";
                } else {
                    const auto flag = take(0);
                    const auto is_array = ts[flag].at("value").get<unsigned>() != 0;
                    field["array_flag_token_index"] = flag;
                    field["is_array"] = is_array;
                    field["type_name"] = string();
                    field["layout"] = is_array ? "array" : "scalar";
                    const auto type = field.at("type_name").get<std::string>();
                    const auto code = value_type(type);
                    if (!is_array) {
                        if (code) {
                            field["value_token_indices"].push_back(take(*code));
                            field["value_status"] = "encoded";
                        } else {
                            // The native scalar dateTime and nonprimitive branches
                            // do not read a value token. Do not invent a default.
                            field["value_status"] = "not_serialized";
                        }
                    } else {
                        const auto count_token = take(3);
                        const auto count = ts[count_token].at("value").get<std::int64_t>();
                        field["count_token_index"] = count_token;
                        field["declared_count"] = count;
                        field["elements"] = Json::array();
                        if (type != "dateTime") {
                            require(count <= 0 ||
                                        std::uint64_t(count) <=
                                            budget.limits.max_array_elements - budget.elements,
                                    "property_block_array_budget");
                            if (count > 0)
                                budget.elements += std::size_t(count);
                        }
                        field["value_status"] = type == "dateTime" ? "not_serialized" : "encoded";
                        if (type != "dateTime")
                            for (std::int64_t i = 0; i < count; ++i) {
                                if (code)
                                    field["value_token_indices"].push_back(take(*code));
                                else {
                                    Json child = {{"array_index", i}};
                                    const auto index_token = take(3), length_token = take(3);
                                    child["source_entry_token_index"] = index_token;
                                    child["stored_entry_offset"] = ts[index_token].at("value");
                                    child["stored_entry_offset_matches_position"] =
                                        ts[index_token].at("value") == ts[index_token].at("offset");
                                    child["byte_count_token_index"] = length_token;
                                    const auto length =
                                        ts[length_token].at("value").get<std::int64_t>();
                                    const auto payload = take(5);
                                    child["payload_token_index"] = payload;
                                    require(length >= 0 && ts[payload].at("bytes") == length,
                                            "property_struct_array_byte_count_mismatch");
                                    try {
                                        child["data_block"] = read_block(
                                            unbase64(
                                                ts[payload].at("binary_base64").get<std::string>()),
                                            budget, depth + 1);
                                        if (child.at("data_block")
                                                .at("property_structure")
                                                .at("status") != "structured")
                                            complete = false;
                                    } catch (const std::bad_alloc &) {
                                        throw;
                                    } catch (const std::exception &e) {
                                        child["reason"] = e.what();
                                        complete = false;
                                    }
                                    field["elements"].push_back(std::move(child));
                                }
                            }
                    }
                }
                field["status"] = "read";
            } catch (const std::bad_alloc &) {
                throw;
            } catch (const std::exception &e) {
                field["reason"] = e.what();
                complete = false;
            }
            field["token_count"] = cursor - start;
            const auto end =
                cursor < ts.size() ? ts[cursor].at("offset").get<std::size_t>() : bytes.size();
            field["source_byte_count"] = end - ts[start].at("offset").get<std::size_t>();
            const auto ok = field.at("status") == "read";
            structure["fields"].push_back(std::move(field));
            if (!ok)
                break;
        }
    } catch (const std::bad_alloc &) {
        throw;
    } catch (const std::exception &e) {
        structure["reason"] = e.what();
        complete = false;
    }
    structure["consumed_token_count"] = cursor;
    structure["remaining_token_count"] = ts.size() - cursor;
    structure["status"] = complete ? "structured" : "partial";
    out["property_structure"] = std::move(structure);
    out["binary_value_views"] = binary_value_views(out, budget);
    return out;
}
} // namespace
Json decode_property_data_block(const Bytes &bytes, const PropertyBlockLimits &limits) {
    require(limits.max_tokens && limits.max_fields && limits.max_array_elements &&
                limits.max_depth && limits.max_decoded_bytes,
            "positive_property_block_limits_required");
    BlockBudget budget{limits};
    return read_block(bytes, budget, 0);
}
} // namespace p3d
