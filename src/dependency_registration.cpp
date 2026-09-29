#include "internal.hpp"
#include <p3d/dependency_registration.hpp>
#include <unordered_map>

namespace p3d {
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
                    tick();require(payload.size()>=8,"truncated_dependency_header");
                    Reader header(payload);
                    const auto owner=header.u16(),relation=header.u16(),flags=header.u16(),count=header.u16();
                    const auto format=(flags>>10)&15u;
                    if(format>8 || (flags&1) || count==0)
                        continue;
                    require(format<=1,"dependency_format_requires_context");
                    require(!(format==0 && owner==10000 && relation==4),
                            "dependency_owner_path_requires_context");
                    require(count<=remaining,"work_limit_exceeded");
                    const std::size_t needed=8+std::size_t(count)*(format==0?8:16);
                    require(payload.size()>=needed,"truncated_dependency_entries");
                    // Reuse the actual record decoder. Unused suffix bytes are
                    // irrelevant to registration and need not be copied/decoded.
                    const auto decoded=native_dependency_link(slice(payload,0,needed));
                    // Format 1's second word and non-dispatch flag bits remain
                    // unassigned by the source decoder, but do not affect this
                    // callback's direct-ID lookup.
                    require(!decoded.contains("error") && decoded.at("entries").size()==count,
                            "truncated_dependency_entries");
                    for(const auto &entry:decoded.at("entries")) {
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
} // namespace p3d
