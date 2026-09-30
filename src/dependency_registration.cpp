#include "internal.hpp"
#include <p3d/dependency_registration.hpp>
#include <unordered_map>

namespace p3d {
namespace {
Json direct_dependency_entries(const Bytes &payload,std::size_t remaining,bool honor_disabled=true) {
    require(payload.size()>=8,"truncated_dependency_header");
    Reader header(payload);
    const auto owner=header.u16(),relation=header.u16(),flags=header.u16(),count=header.u16();
    const auto format=(flags>>10)&15u;
    if(format>8 || (honor_disabled && (flags&1)) || count==0)return Json::array();
    require(format<=1,"dependency_format_requires_context");
    require(!(format==0 && owner==10000 && relation==4),"dependency_owner_path_requires_context");
    require(count<=remaining,"work_limit_exceeded");
    const std::size_t needed=8+std::size_t(count)*(format==0?8:16);
    require(payload.size()>=needed,"truncated_dependency_entries");
    // Source decoding may remain partial for other flag bits, format 1's
    // second word, or a suffix; the direct-ID field is nevertheless known.
    auto decoded=native_dependency_link(slice(payload,0,needed));
    require(!decoded.contains("error") && decoded.at("entries").size()==count,"truncated_dependency_entries");
    return std::move(decoded.at("entries"));
}
}
NativeDependencyLoadResult project_native_dependency_load(const NativeDependencyLoadInput &input) {
    NativeDependencyLoadResult out;
    auto fail=[&](const std::string &reason) {
        out.reason=reason;out.dependents.clear();out.pending_entities.clear();out.batches.clear();
        return out;
    };
    if(!input.input_complete)return fail("incomplete_input");
    if(!input.system_registry_known_empty)return fail("system_registry_requires_context");
    if(!input.file_fallback_disabled)return fail("file_fallback_requires_context");
    if(!input.monitored_entity_set_known_empty)return fail("monitored_entities_require_context");
    if(input.entities.size()>input.max_work_items)return fail("work_limit_exceeded");
    std::size_t remaining=input.max_work_items;
    auto tick=[&](){require(remaining!=0,"work_limit_exceeded");--remaining;};
    try {
        std::unordered_map<std::uint64_t,std::size_t> registry;
        std::vector<bool> registered(input.entities.size()),pending(input.entities.size());
        out.dependents.resize(input.entities.size());
        for(std::size_t batch_index=0;batch_index<input.batches.size();++batch_index) {
            out.failed_batch=batch_index;tick();
            const auto &batch=input.batches[batch_index];
            NativeDependencyLoadBatchResult result;
            for(auto index:batch) {
                out.failed_entity=index;tick();
                require(index<input.entities.size(),"entity_index_out_of_range");
                require(!registered[index],"entity_registered_more_than_once");
                const auto id=input.entities[index].assigned_id;
                if(id!=0)require(registry.emplace(id,index).second,"assigned_id_collision");
                registered[index]=true;
            }
            for(auto index:batch) {
                out.failed_entity=index;
                for(const auto &payload:input.entities[index].dependency_payloads) {
                    tick();
                    for(const auto &entry:direct_dependency_entries(payload,remaining)) {
                        tick();const auto id=entry.at("element_id").get<std::uint64_t>();
                        const auto found=registry.find(id);
                        if(found==registry.end()) {
                            if(id!=0 && !pending[index]) {
                                pending[index]=true;result.newly_pending_entities.push_back(index);
                            }
                            continue;
                        }
                        const auto target=found->second;
                        const auto flags=input.entities[target].runtime_flags_10;
                        require(flags.has_value(),"target_runtime_flags_require_context");
                        // Mode 1 and a fresh service skip rejected targets. They
                        // are distinct from a missing nonzero ID, which queues.
                        if(*flags&0x20008u)continue;
                        out.dependents[target].push_back(index);
                        result.added_edges.push_back({target,index});
                    }
                }
            }
            out.batches.push_back(std::move(result));
        }
        require(std::all_of(registered.begin(),registered.end(),[](bool v){return v;}),"entity_not_registered");
        for(auto &list:out.dependents)std::reverse(list.begin(),list.end());
        for(std::size_t i=0;i<pending.size();++i)if(pending[i])out.pending_entities.push_back(i);
        out.failed_batch.reset();out.failed_entity.reset();out.resolved=true;
        return out;
    } catch(const std::exception &e) {return fail(e.what());}
}

namespace {
NativeDependencyRetryResult retry_impl(const NativeDependencyRetryInput &input,std::size_t &remaining) {
    NativeDependencyRetryResult out;
    auto fail=[&](const std::string &reason) {
        out.reason=reason;out.dependents.clear();out.pending_entities.clear();out.monitored_entities.clear();
        out.added_edges.clear();out.scheduled_pairs.clear();return out;
    };
    if(!input.input_complete)return fail("incomplete_input");
    if(!input.system_registry_known_empty)return fail("system_registry_requires_context");
    if(!input.file_fallback_disabled)return fail("file_fallback_requires_context");
    if(!input.monitored_entity_set_known_empty)return fail("monitored_entities_require_context");
    if(!input.model_notified.has_value())return fail("model_notification_requires_context");
    if(input.entities.size()>input.max_work_items)return fail("work_limit_exceeded");
    if(input.dependents.size()!=input.entities.size())return fail("reverse_list_count_mismatch");
    auto tick=[&](){require(remaining!=0,"work_limit_exceeded");--remaining;};
    try {
        std::unordered_map<std::uint64_t,std::size_t> registry;
        std::vector<bool> visited(input.entities.size()),pending(input.entities.size());
        out.dependents.resize(input.entities.size());
        for(std::size_t i=0;i<input.entities.size();++i) {
            out.failed_entity=i;tick();
            const auto id=input.entities[i].assigned_id;
            if(id)require(registry.emplace(id,i).second,"assigned_id_collision");
            // Keep reversed lists while appending new heads, avoiding quadratic
            // insertion for repeated dependencies. Reverse once at completion.
            for(auto it=input.dependents[i].rbegin();it!=input.dependents[i].rend();++it) {
                tick();require(*it<input.entities.size(),"dependent_index_out_of_range");
                out.dependents[i].push_back(*it);
            }
        }
        std::set<std::pair<std::size_t,std::size_t>> scheduled;
        for(auto index:input.pending_iteration_order) {
            out.failed_entity=index;tick();
            require(index<input.entities.size(),"entity_index_out_of_range");
            require(!visited[index],"pending_entity_repeated");visited[index]=true;
            const auto &entity=input.entities[index];
            require(entity.runtime_flags_10.has_value(),"pending_runtime_flags_require_context");
            if(*entity.runtime_flags_10&0x20008u)continue;
            require(!(*entity.runtime_flags_10&0x100000u),"attribute_dependencies_require_context");
            for(const auto &payload:entity.dependency_payloads) {
                tick();
                for(const auto &entry:direct_dependency_entries(payload,remaining)) {
                    tick();const auto id=entry.at("element_id").get<std::uint64_t>();
                    const auto found=registry.find(id);
                    if(found==registry.end()) {if(id)pending[index]=true;continue;}
                    const auto target=found->second;
                    const auto flags=input.entities[target].runtime_flags_10;
                    require(flags.has_value(),"target_runtime_flags_require_context");
                    if(*flags&0x20008u) {if(id)pending[index]=true;continue;}
                    if(!*input.model_notified)continue;
                    out.dependents[target].push_back(index);
                    out.added_edges.push_back({target,index});scheduled.emplace(target,index);
                }
            }
        }
        for(auto &list:out.dependents)std::reverse(list.begin(),list.end());
        for(std::size_t i=0;i<pending.size();++i)if(pending[i])out.pending_entities.push_back(i);
        out.monitored_entities=out.pending_entities;
        for(const auto &pair:scheduled)out.scheduled_pairs.push_back({pair.first,pair.second});
        out.failed_entity.reset();out.resolved=true;return out;
    } catch(const std::exception &e) {return fail(e.what());}
}
NativeDependencyNormalizationResult normalization_impl(const NativeDependencyNormalizationInput &input,std::size_t &remaining) {
    NativeDependencyNormalizationResult out;
    auto fail=[&](const std::string &reason) {
        out.reason=reason;out.dependents.clear();out.retained_positions.clear();return out;
    };
    if(!input.input_complete)return fail("incomplete_input");
    if(!input.standard_entities_known)return fail("target_entity_interface_requires_context");
    if(!input.removal_work_known_empty)return fail("dependency_removal_requires_context");
    if(input.dependents.size()>input.max_work_items)return fail("work_limit_exceeded");
    auto tick=[&](){require(remaining!=0,"work_limit_exceeded");--remaining;};
    try {
        const auto count=input.dependents.size();
        std::vector<std::set<std::size_t>> scheduled(count);
        for(const auto &edge:input.scheduled_pairs) {
            tick();require(edge.target_entity<count && edge.dependent_entity<count,"scheduled_entity_index_out_of_range");
            scheduled[edge.target_entity].insert(edge.dependent_entity);
        }
        out.dependents.resize(count);out.retained_positions.resize(count);
        for(std::size_t target=0;target<count;++target) {
            tick();std::set<std::size_t> seen;
            for(std::size_t position=0;position<input.dependents[target].size();++position) {
                tick();const auto dependent=input.dependents[target][position];
                require(dependent<count,"dependent_index_out_of_range");
                if(scheduled[target].count(dependent) && !seen.insert(dependent).second)continue;
                out.dependents[target].push_back(dependent);out.retained_positions[target].push_back(position);
            }
        }
        out.resolved=true;return out;
    } catch(const std::exception &e) {return fail(e.what());}
}
} // namespace
NativeDependencyRetryResult project_native_dependency_retry(const NativeDependencyRetryInput &input) {
    auto remaining=input.max_work_items;
    return retry_impl(input,remaining);
}
NativeDependencyNormalizationResult project_native_dependency_normalization(const NativeDependencyNormalizationInput &input) {
    auto remaining=input.max_work_items;
    return normalization_impl(input,remaining);
}
NativeDependencyCycleResult project_native_dependency_cycle(const NativeDependencyCycleInput &input) {
    NativeDependencyCycleResult out;
    auto fail=[&](const std::string &reason) {
        out.reason=reason;out.dependents.clear();out.pending_entities.clear();
        out.monitored_entities.clear();out.scheduled_pairs.clear();return out;
    };
    if(!input.other_work_queues_known_empty)return fail("dependency_work_queues_require_context");
    if(!input.standard_entities_known)return fail("target_entity_interface_requires_context");
    if(!input.caller_holds_model)return fail("model_release_requires_context");
    if(!input.link_update_handlers_known_absent)return fail("link_update_handlers_require_context");
    auto remaining=input.max_work_items;
    const auto retry=retry_impl(input,remaining);
    if(!retry.resolved) {out.failed_entity=retry.failed_entity;return fail(retry.reason);}
    auto tick=[&](){require(remaining!=0,"work_limit_exceeded");--remaining;};
    try {
        // 1f5130 visits every linkage of remaining entities. Unlike the retry
        // phase, its update callback does not skip flag bit 0. Validate those
        // references too, including links which did not cause the pending work.
        for(auto index:retry.pending_entities) {
            out.failed_entity=index;tick();
            for(const auto &payload:input.entities[index].dependency_payloads) {
                tick();require(payload.size()>=8,"truncated_dependency_header");
                Reader header(payload);header.u16();header.u16();const auto flags=header.u16();
                if(((flags>>10)&15)>8 || (flags&0x8000))continue;
                require(!(flags&0x4000),"required_link_handler_requires_context");
                for(const auto &entry:direct_dependency_entries(payload,remaining,false)) {
                    (void)entry;tick();
                }
            }
        }
    } catch(const std::exception &e) {return fail(e.what());}
    NativeDependencyNormalizationInput normal;
    normal.dependents=retry.dependents;normal.scheduled_pairs=retry.scheduled_pairs;
    normal.input_complete=normal.standard_entities_known=normal.removal_work_known_empty=true;
    normal.max_work_items=input.max_work_items;
    auto normalized=normalization_impl(normal,remaining);
    if(!normalized.resolved) {out.failed_entity.reset();return fail(normalized.reason);}
    out.dependents=std::move(normalized.dependents);
    out.monitored_entities=retry.monitored_entities;
    // registry30 -> registryc0 -> registry140 is consumed by this iteration.
    // service40 is separate and retains the unresolved membership.
    out.failed_entity.reset();out.resolved=true;return out;
}
} // namespace p3d
