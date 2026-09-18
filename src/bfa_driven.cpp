#include "internal.hpp"

namespace p3d {
namespace {
std::int64_t signed_bits(std::uint64_t n) {
    return n <= std::uint64_t(INT64_MAX) ? std::int64_t(n) : -1 - std::int64_t(~n);
}
std::uint64_t wrapper_id(std::uint64_t n) {
    const auto low = std::uint32_t(n);
    return std::uint64_t(low < 0x80000000u ? std::int64_t(low) : std::int64_t(low) - 0x100000000LL);
}
struct BfaLookup {
    bool known = false;
    std::optional<std::size_t> index;
    std::string reason;
};
class BfaDriveResolver {
    const Json &records;
    const BfaDrivenContext &context;
    std::map<std::uint64_t, std::vector<std::size_t>> nodes, parents;
    using PropertyMap = std::map<std::uint64_t, std::pair<std::uint64_t, std::size_t>>;
    // Cache per call/registry: repeated inputs reuse immutable source indices.
    mutable std::map<std::size_t, std::array<PropertyMap, 2>> property_maps;

  public:
    bool needs_id_conversion = false;
    BfaDriveResolver(const Json &r, const BfaDrivenContext &c) : records(r), context(c) {
        require(records.is_array(), "BFA records must be an array");
        for (std::size_t i = 0; i < records.size(); ++i) {
            const auto id = records[i].at("id").get<std::uint64_t>();
            needs_id_conversion |= wrapper_id(id) != id;
            nodes[id].push_back(i);
            for (const auto &child : records[i].at("children")) {
                const auto child_id = child.get<std::uint64_t>();
                needs_id_conversion |= wrapper_id(child_id) != child_id;
                parents[child_id].push_back(i);
            }
        }
    }
    BfaLookup find(std::uint64_t id) const {
        const auto found = nodes.find(id);
        if (found == nodes.end()) {
            // Native loading registers child placeholders before their bodies.
            // A declared child is not proven absent merely because its record
            // is missing from the supplied decoded data.
            if (parents.count(id))
                return {false, {}, "unloaded_child_placeholder"};
            return {context.project_registry_complete, {}, "node_not_in_registry"};
        }
        if (found->second.size() != 1)
            return {false, {}, "ambiguous_node_id"};
        return {true, found->second.front(), {}};
    }
    // A known empty result is the native null parent; its accessor returns -1.
    BfaLookup parent(std::uint64_t id) const {
        const auto node = find(id);
        if (!node.known || !node.index)
            return node;
        const auto found = parents.find(id);
        if (found == parents.end())
            return {context.project_registry_complete, {}, "parent_not_in_registry"};
        if (found->second.size() != 1)
            return {false, {}, "ambiguous_parent"};
        const auto pi = found->second.front();
        if (pi == *node.index)
            return {false, {}, "self_parent"};
        const auto p = find(records[pi].at("id").get<std::uint64_t>());
        return p;
    }
    std::pair<std::uint64_t, std::uint64_t> pair(const Json &reference, bool target) const {
        const auto kind = reference.at("storage_kind").get<unsigned>();
        require(kind >= 1 && kind <= 3, "BFA driven storage kind");
        if (kind == 3) {
            const auto object = reference.at("object_id").get<std::uint64_t>();
            if (target && !object)
                return {0, 0};
            return {object, std::uint64_t(reference.at("property_id").get<std::int64_t>())};
        }
        const auto property = reference.at("property_reference_id").get<std::uint64_t>();
        if (target && !property)
            return {0, 0};
        return {context.currently_editing_id, property};
    }
    static void unresolved(Json &out, const std::string &reason) {
        out["status"] = "unresolved";
        out["reason"] = reason;
    }
    static void handle(Json &out, std::uint64_t object, std::uint64_t property) {
        out["status"] = "handle_constructed";
        out["object_id"] = object;
        out["property_id_bits"] = property;
        out["property_id"] = signed_bits(property);
        out["handle_kind_code"] = 10;
        out["property_validation"] = "not_performed";
    }
    // Map lookup is independent of the original driven storage family: the
    // native target/input converters have already flattened it to a pair.
    bool component(Json &out, std::size_t index, std::uint64_t reference, bool target,
                   std::uint64_t environment) const {
        const auto &node = records[index];
        if (!node.contains("component_definition")) {
            unresolved(out, "component_definition_unavailable");
            return false;
        }
        const auto &definition = node.at("component_definition");
        if (definition.at("mapping_layout_status") != "unique_candidate") {
            unresolved(out, "component_mapping_requires_version_context");
            return false;
        }
        const auto &maps = definition.at("mapping_layout_candidates").at(0).at("property_id_maps");
        require(maps.size() == 2, "BFA component mapping count");
        auto cached = property_maps.find(index);
        if (cached == property_maps.end()) {
            std::array<PropertyMap, 2> effective;
            for (std::size_t m = 0; m < 2; ++m) {
                const auto &entries = maps[m].at("entries");
                for (std::size_t e = 0; e < entries.size(); ++e)
                    effective[m][entries[e].at("property_definition_id").get<std::uint64_t>()] = {
                        std::uint64_t(entries[e].at("property_id").get<std::int64_t>()), e};
            }
            PropertyMap inverse;
            for (const auto &entry : effective[1])
                inverse[entry.second.first] = {entry.first, entry.second.second};
            effective[1] = std::move(inverse);
            cached = property_maps.emplace(index, std::move(effective)).first;
        }
        const auto &effective = cached->second;
        auto value = UINT64_MAX;
        const auto forward = effective[0].find(reference);
        Json trace = {{"map_index", 0},
                      {"direction", "definition_to_property"},
                      {"key_bits", reference},
                      {"matched", forward != effective[0].end()}};
        if (forward != effective[0].end()) {
            value = forward->second.first;
            trace["entry_index"] = forward->second.second;
        }
        trace["result_bits"] = value;
        out["lookup_trace"] = Json::array({trace});
        out["component_record_index"] = index;
        if (target && value == UINT64_MAX) {
            out["status"] = "default_handle";
            out["reason"] = "first_map_returned_minus_one";
            return true;
        }
        if ((target && value == 0) || (!target && value == UINT64_MAX)) {
            value = 0;
            trace = {{"map_index", 1},
                     {"direction", "property_to_definition"},
                     {"key_bits", reference},
                     {"matched", false}};
            const auto inverse = effective[1].find(reference);
            if (inverse != effective[1].end()) {
                value = inverse->second.first;
                trace["matched"] = true;
                trace["entry_index"] = inverse->second.second;
            }
            trace["result_bits"] = value;
            out["lookup_trace"].push_back(std::move(trace));
        }
        handle(out, environment, value);
        return true;
    }
    Json convert(const Json &reference, bool target) const {
        const auto p = pair(reference, target);
        Json out = {{"pair_object_id", p.first}, {"pair_property_bits", p.second}};
        auto environment = p.first;
        if (target) {
            if ((p.first == 0 || p.first == context.currently_editing_id) && p.second != 0) {
                const auto owner = parent(p.second);
                if (!owner.known) {
                    unresolved(out, owner.reason);
                    return out;
                }
                environment =
                    owner.index ? records[*owner.index].at("id").get<std::uint64_t>() : UINT64_MAX;
                out["environment_source"] = "referenced_node_parent";
            } else
                out["environment_source"] = "stored_pair";
        } else {
            const auto object = find(p.first);
            if (!object.known) {
                unresolved(out, object.reason);
                return out;
            }
            bool parentless = false;
            if (object.index) {
                const auto owner = parent(p.first);
                if (!owner.known) {
                    unresolved(out, owner.reason);
                    return out;
                }
                parentless = !owner.index;
            }
            out["pair_object_is_registered_parentless_node"] = parentless;
            if (!parentless) {
                // Native input conversion constructs this pair directly, even
                // for a missing object in a caller-declared complete registry.
                handle(out, p.first, p.second);
                return out;
            }
            const auto owner = parent(p.second);
            if (!owner.known) {
                unresolved(out, owner.reason);
                return out;
            }
            environment =
                owner.index ? records[*owner.index].at("id").get<std::uint64_t>() : UINT64_MAX;
            out["environment_source"] = "referenced_node_parent";
        }
        out["lookup_environment_id"] = environment;
        const auto node = find(environment);
        if (!node.known) {
            unresolved(out, node.reason);
            return out;
        }
        if (!node.index) {
            out["status"] = target ? "default_handle" : "native_early_return";
            out["reason"] = "environment_object_missing";
            return out;
        }
        const auto kind = records[*node.index].at("node_kind").get<std::string>();
        if (kind == "component_definition")
            component(out, *node.index, p.second, target, environment);
        else if (target && kind == "primitive")
            // The target's native primitive branch uses the original pair,
            // even when an environment lookup was redirected to a parent.
            handle(out, p.first, p.second);
        else if (kind == "component_type" || kind == "property_definition" ||
                 kind == "driven_object" || kind == "primitive") {
            out["status"] = target ? "default_handle" : "native_early_return";
            out["reason"] = "environment_is_not_component";
        } else
            unresolved(out, "unsupported_environment_node_kind");
        return out;
    }
};
} // namespace

Json resolve_bfa_driven_references(const Json &bfa_tree, std::uint64_t driven_id,
                                   const BfaDrivenContext &context) {
    Json out = {{"scope", "supplied_component_registry"},
                {"project_registry_complete", context.project_registry_complete},
                {"currently_editing_id", context.currently_editing_id},
                {"driven_id", driven_id},
                {"formula_evaluation", "not_performed"}};
    try {
        const auto &records = bfa_tree.at("records");
        BfaDriveResolver resolver(records, context);
        if (resolver.needs_id_conversion) {
            BfaDriveResolver::unresolved(out, "native_node_id_conversion_required");
            return out;
        }
        const auto drive = resolver.find(driven_id);
        if (!drive.known) {
            BfaDriveResolver::unresolved(out, drive.reason);
            return out;
        }
        if (!drive.index || records[*drive.index].at("node_kind") != "driven_object") {
            out["status"] = "drive_unavailable";
            out["target"] = {{"status", "default_handle"}};
            out["inputs"] = Json::array();
            out["input_handle_source_indices"] = Json::array();
            return out;
        }
        if (!records[*drive.index].contains("driven")) {
            BfaDriveResolver::unresolved(out, "driven_payload_unavailable");
            return out;
        }
        const auto &driven = records[*drive.index].at("driven");
        out["driven_record_index"] = *drive.index;
        out["target"] = resolver.convert(driven.at("target"), true);
        out["inputs"] = Json::array();
        out["input_handle_source_indices"] = Json::array();
        std::string state = "complete";
        const auto &inputs = driven.at("inputs");
        for (std::size_t i = 0; i < inputs.size(); ++i) {
            Json item;
            if (state != "complete")
                item = {{"status", state == "native_early_return"
                                       ? "not_visited_after_native_return"
                                       : "not_evaluated_after_unresolved"}};
            else {
                item = resolver.convert(inputs[i], false);
                const auto status = item.at("status").get<std::string>();
                if (status == "handle_constructed")
                    out["input_handle_source_indices"].push_back(i);
                else {
                    state = status;
                    out["input_stop_index"] = i;
                }
            }
            item["input_index"] = i;
            out["inputs"].push_back(std::move(item));
        }
        out["inputs_status"] = state;
        out["status"] = out["target"]["status"] == "unresolved" || state == "unresolved"
                            ? "unresolved"
                            : "resolved";
    } catch (const std::exception &e) {
        out["status"] = "invalid_input";
        out["reason"] = e.what();
        out.erase("target");
        out.erase("inputs");
        out.erase("input_handle_source_indices");
    }
    return out;
}
} // namespace p3d
