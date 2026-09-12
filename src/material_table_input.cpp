#include "internal.hpp"

namespace p3d {
Json native_system_material_table(const Json &list, const Json &records, const Json &ids) {
    Json out = {{"reader_profile", "bimbase_2025_system_material_table"},
                {"scope", "first_system_input_with_empty_id_registry"},
                {"status", "unresolved"},
                {"table", nullptr}};
    if (list.value("system_bootstrap_required", Json()) != true) {
        out["reason"] = "system_container_required";
        return out;
    }
    if (list.value("status", Json()) != "resolved" || ids.value("status", Json()) != "resolved") {
        out["reason"] = "complete_system_input_and_id_assignments_required";
        return out;
    }
    try {
        const auto &roots = list.at("roots");
        const auto &assigned_roots = ids.at("roots");
        require(roots.size() == assigned_roots.size(), "system root assignment mismatch");
        for (std::size_t ri = 0; ri < roots.size(); ++ri) {
            const auto &root = roots[ri];
            const auto ni = root.at("native_record_index").get<std::size_t>();
            const auto &source = records.at(ni);
            if (source.at("element_type") != 10)
                continue;
            if (Reader(bytesof(source.at("data")), 16).u32() != 18)
                continue;
            const auto &assigned = assigned_roots[ri];
            const auto &items = assigned.at("records");
            const auto &headers = root.at("headers");
            require(assigned.at("native_record_index") == ni &&
                        assigned.at("block_number") == root.at("block_number") && !items.empty() &&
                        items.size() == headers.size(),
                    "material table assignment mismatch");
            auto identity = [&](std::size_t i) {
                const auto &item = items.at(i);
                const auto &header = headers.at(i);
                require(item.at("native_record_index") == header.at("native_record_index") &&
                            item.at("parent_record_index") == header.at("parent_record_index"),
                        "material table member assignment mismatch");
                const auto index = item.at("native_record_index").get<std::size_t>();
                return Json{{"native_record_index", index},
                            {"input_occurrence_index", item.at("input_occurrence_index")},
                            {"source_id", item.at("source_id")},
                            {"assigned_id", item.at("assigned_id")}};
            };
            require(items.front().at("native_record_index") == ni &&
                        items.front().at("parent_record_index").is_null(),
                    "material table root identity mismatch");
            auto table = identity(0);
            table["root_input_index"] = ri;
            table["block_number"] = root.at("block_number");
            table["members"] = Json::array();
            for (std::size_t i = 1; i < items.size(); ++i)
                if (items[i].at("parent_record_index") == ni)
                    table["members"].push_back(identity(i));
            // Fresh list objects start with runtime flags zero. The file-input
            // path does not copy source element flags into the runtime mask
            // 0x20008 tested by the table iterator. Scan roots in insertion
            // order, never child headers, ID order, or physical stream order.
            out["table"] = std::move(table);
            out["status"] = "selected";
            return out;
        }
        out["status"] = "absent";
    } catch (const std::exception &e) {
        out["reason"] = "inconsistent_system_input";
        out["error"] = e.what();
    }
    return out;
}
} // namespace p3d
