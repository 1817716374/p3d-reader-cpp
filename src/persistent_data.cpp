#include <p3d/persistent_data.hpp>
#include "component_properties.hpp"

namespace p3d {
namespace {
Json identifier_key(Bytes bytes) {
    bytes.erase(std::find(bytes.begin(), bytes.end(), 0), bytes.end());
    Json out = {{"bytes_base64", base64(bytes)}};
    if (bytes.empty()) {
        out["status"] = "ignored_empty";
        return out;
    }
    if (bytes.front() != 7) {
        out["status"] = "indexed";
        out["selector_kind"] = "name";
        out["index_key"] = "name:" + base64(bytes);
        return out;
    }
    if (bytes.size() < 2) {
        out["status"] = "not_evaluated";
        out["reason"] = "native_short_identifier";
        return out;
    }
    // The native reader removes two bytes, without validating the second.
    std::size_t p = 2;
    auto space = [](std::uint8_t c) { return c == 32 || (c >= 9 && c <= 13); };
    while (p < bytes.size() && space(bytes[p]))
        ++p;
    if (p < bytes.size() && bytes[p] >= 128) {
        out["status"] = "not_evaluated";
        out["reason"] = "numeric_identifier_locale";
        return out;
    }
    bool negative = p < bytes.size() && bytes[p] == '-';
    if (p < bytes.size() && (bytes[p] == '+' || bytes[p] == '-'))
        ++p;
    const auto first = p;
    std::uint64_t value = 0;
    bool overflow = false;
    while (p < bytes.size() && bytes[p] >= '0' && bytes[p] <= '9') {
        const auto digit = std::uint64_t(bytes[p++] - '0');
        if (value > (UINT64_MAX - digit) / 10)
            overflow = true;
        else if (!overflow)
            value = value * 10 + digit;
    }
    if (p == first || overflow) {
        out["status"] = "native_failure";
        out["reason"] =
            overflow ? "numeric_identifier_out_of_range" : "numeric_identifier_invalid_argument";
        return out;
    }
    if (negative)
        value = std::uint64_t(0) - value;
    out["status"] = "indexed";
    out["selector_kind"] = "id";
    out["id"] = value;
    out["index_key"] = "id:" + std::to_string(value);
    out["numeric_end_offset"] = p;
    return out;
}

const Json *field(const Json &root, const char *name) {
    const Json *found = nullptr;
    if (!root.contains("children"))
        return nullptr;
    require(root.at("children").is_array(), "DataUnit children must be an array");
    for (const auto &child : root.at("children")) {
        if (child.value("name", Json()) != name)
            continue;
        require(!found, std::string("duplicate DataUnit field: ") + name);
        found = &child.at("value");
    }
    return found;
}
} // namespace

Json persistent_data_index(const Json &objects, const PersistentDataIndexOptions &options) {
    require(objects.is_array(), "persistent data objects must be an array");
    Json out = {{"kind", "persistent_data_index"},
                {"scope", "selected_project_saved_values"},
                {"complete_project", options.complete_project},
                {"native_enumeration_order", options.native_enumeration_order},
                {"status", "ready"},
                {"entries", Json::array()},
                {"key_index", Json::object()}};
    bool uncertain = false, failure = false;
    for (std::size_t i = 0; i < objects.size(); ++i) {
        const auto &object = objects[i];
        const auto &root = object.at("root");
        if (root.value("name", Json()) != "DataUnit")
            continue;
        const auto schema = root.value("attributes", Json::object()).value("xmlns", "");
        if (schema != "PBM_CoreModel" && schema.rfind("PBM_CoreModel.", 0) != 0)
            continue;
        Json row = {{"object_index", i}, {"schema", schema}};
        for (const auto *name : {"class_id", "object_id", "stream", "offset"})
            if (object.contains(name))
                row[name] = object.at(name);
        try {
            const auto *identifier = field(root, "Identifier");
            if (!identifier) {
                row["status"] = "ignored_missing_identifier";
            } else if (!identifier->is_string()) {
                row["status"] = "not_evaluated";
                row["reason"] = "identifier_is_not_a_string";
            } else {
                row["identifier"] = *identifier;
                auto text = identifier->get<std::string>();
                text.resize(text.find('\0') == std::string::npos ? text.size() : text.find('\0'));
                std::optional<Bytes> encoded;
                if (options.encode_name)
                    encoded = options.encode_name(text);
                else if (std::all_of(text.begin(), text.end(),
                                     [](unsigned char c) { return c < 128; }))
                    encoded = Bytes(text.begin(), text.end());
                if (!encoded) {
                    row["status"] = "not_evaluated";
                    row["reason"] = "identifier_code_page_required";
                } else {
                    row["key"] = identifier_key(std::move(*encoded));
                    row["status"] = row.at("key").at("status");
                    if (row.at("status") == "indexed") {
                        if (const auto *value = field(root, "DataUnit"))
                            row["value_field"] = *value;
                        if (const auto *md5 = field(root, "MD5Code"))
                            row["md5_code"] = *md5;
                        const auto key = row.at("key").at("index_key").get<std::string>();
                        auto &indices = out["key_index"][key];
                        if (indices.is_null())
                            indices = Json::array();
                        indices.push_back(out.at("entries").size());
                    }
                }
            }
        } catch (const std::exception &e) {
            row["status"] = "not_evaluated";
            row["reason"] = e.what();
        }
        uncertain |= row.at("status") == "not_evaluated";
        failure |= row.at("status") == "native_failure";
        out["entries"].push_back(std::move(row));
    }
    out["status"] = failure ? "native_failure" : uncertain ? "partial" : "ready";
    return out;
}

Json resolve_persistent_data_reference(const Json &reference, const Json &index) {
    Json out = {{"scope", "selected_project_saved_values"}, {"status", "not_evaluated"}};
    try {
        require(index.at("kind") == "persistent_data_index", "expected persistent data index");
        require(reference.at("kind") == "persistent_data_reference" &&
                    reference.at("status") == "decoded",
                "reference is not completely decoded");
        std::string key;
        if (reference.at("selector_kind") == "id") {
            const auto &id = reference.at("id");
            require(id.is_number_unsigned() ||
                        (id.is_number_integer() && id.get<std::int64_t>() >= 0),
                    "persistent data id is not uint64");
            key = "id:" + std::to_string(id.get<std::uint64_t>());
        } else {
            require(reference.at("selector_kind") == "name", "unknown persistent selector");
            // Unlike Identifier conversion, the reference name keeps embedded NULs.
            key = "name:" +
                  base64(unbase64(reference.at("selector").at("bytes_base64").get<std::string>()));
        }
        out["candidates"] = index.at("key_index").value(key, Json::array());
        if (index.at("status") == "native_failure") {
            out["status"] = "native_failure";
            out["reason"] = "dataset_identifier_conversion_failed";
            return out;
        }
        require(index.at("complete_project") == true, "project_dataset_is_incomplete");
        require(index.at("status") == "ready", "dataset_has_unresolved_identifiers");
        const auto &candidates = out.at("candidates");
        if (candidates.empty()) {
            out["status"] = "missing";
            return out;
        }
        if (candidates.size() > 1 && index.at("native_enumeration_order") != true) {
            out["status"] = "ambiguous";
            out["reason"] = "native_enumeration_order_required";
            return out;
        }
        const auto selected = candidates.back().get<std::size_t>();
        const auto &row = index.at("entries").at(selected);
        out["selected_entry_index"] = selected;
        out["target"] = {{"object_index", row.at("object_index")}};
        for (const auto *name : {"class_id", "object_id", "stream", "offset", "identifier"})
            if (row.contains(name))
                out["target"][name] = row.at(name);
        if (row.contains("md5_code"))
            out["md5_code"] = row.at("md5_code");
        if (!row.contains("value_field")) {
            out["status"] = "missing_value";
            return out;
        }
        const auto &value = row.at("value_field");
        out["value_field"] = value;
        out["status"] = "resolved";
        try {
            require(value.is_object() && value.contains("binary_base64"), "DataUnit is not binary");
            // Decode retained bytes, not potentially stale caller-provided decoded JSON.
            out["value"] = decode_data_unit(unbase64(value.at("binary_base64").get<std::string>()));
            out["value_status"] = out.at("value").at("status");
        } catch (const std::exception &e) {
            out["value_status"] = "invalid";
            out["value_error"] = e.what();
        }
    } catch (const std::exception &e) {
        out["reason"] = e.what();
    }
    return out;
}
} // namespace p3d
