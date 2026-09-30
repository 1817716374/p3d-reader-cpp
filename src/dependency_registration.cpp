#include "internal.hpp"
#include <p3d/dependency_registration.hpp>
#include <unordered_map>

namespace p3d {
namespace {
void dependency_path_header(const Bytes &source,std::uint64_t id) {
    require(source.size()>=38 && Reader(source,4).u16()==47 && Reader(source,6).u16()==4 &&
            Reader(source,12).u32()==17 && std::uint64_t(Reader(source,8).u32())*2+4==source.size(),
            "owner_path_source_profile_required");
    require(Reader(source,20).u64()==id,"owner_path_source_id_mismatch");
    require(Reader(source,28).u64()==0,"owner_path_source_profile_required");
}
struct DependencyOwnerProgram {bool valid=false;unsigned format=0;std::vector<std::uint64_t> ids;};
DependencyOwnerProgram dependency_owner_program(const Bytes &source,std::size_t &remaining) {
    // Header/profile already validated. Native rejects these discriminators
    // before examining any link, and never falls back past the first match.
    if(Reader(source,16).u32()!=20 || Reader(source,36).u16()!=0x56e6)return {};
    for(std::size_t at=38;at<source.size();) {
        require(remaining!=0,"work_limit_exceeded");--remaining;
        require(source.size()-at>=4,"truncated_owner_path_linkage");
        const auto h=Reader(source,at).u16(),app=Reader(source,at+2).u16();
        std::size_t bytes=8;
        if(h&0x1000) {
            const auto words=(h&0x4000)?std::size_t(h&255)<<((h>>8)&15):std::size_t(h&255)+1;
            require(words>=2 && words<=65535,"invalid_owner_path_linkage_length");bytes=words*2;
        }
        require(bytes<=source.size()-at,"truncated_owner_path_linkage");
        const auto payload=at+4,size=bytes-4;at+=bytes;
        if(!(h&0x1000) || app!=0x56d0)continue;
        if(size>=2 && Reader(source,payload).u16()!=10000)continue;
        require(size>=4,"truncated_owner_path_linkage_key");
        if(Reader(source,payload+2).u16()!=4)continue;
        require(size>=8,"truncated_owner_path_header");
        const auto format=(Reader(source,payload+4).u16()>>10)&15u;
        const auto count=Reader(source,payload+6).u16();
        if(format!=0 && format!=6)return {};
        if(format==6 && count!=1)return {};
        require(format==0 || size>=24,"truncated_owner_path_header");
        const auto n=format==0?std::uint32_t(count):Reader(source,payload+8).u32();
        if(format==6 && n==0)return {};
        const std::size_t offset=format==0?8:24;
        require(n<=(size-offset)/8,"truncated_owner_path_ids");
        require(n<=remaining,"work_limit_exceeded");remaining-=n;
        DependencyOwnerProgram result{true,format,{}};result.ids.reserve(n);
        Reader ids(source,payload+offset);
        for(std::uint32_t i=0;i<n;++i)result.ids.push_back(ids.u64());
        return result;
    }
    return {};
}
Json dependency_owner_source(const Bytes &source,std::uint64_t id) {
    require(source.size()==372,"owner_reference_source_profile_required");
    require(Reader(source,4).u16()==13 &&
            (Reader(source,6).u16()==0x20 || Reader(source,6).u16()==0x60) &&
            Reader(source,8).u32()==184 && Reader(source,12).u32()==184 &&
            Reader(source,16).u32()==0,"owner_reference_source_profile_required");
    require(Reader(source,20).u64()==id,"owner_reference_source_id_mismatch");
    for(std::size_t at=28;at<source.size();++at)
        if(at<172 || at>=300)require(source[at]==0,"owner_reference_source_profile_required");
    // The profile excludes nonfinite inputs; retaining their bytes in the
    // source decoder does not prove that this runtime branch is evaluable.
    for(std::size_t at=172;at<300;at+=8)
        require(std::isfinite(Reader(source,at).f64()),"owner_reference_source_nonfinite");
    auto decoded=native_reference_input(source);
    require(decoded.value("status","")=="decoded" &&
            decoded.at("transform").value("status","")=="computed",
            "owner_reference_source_transform_unresolved");
    return decoded;
}
Json direct_dependency_entries(const Bytes &payload,std::size_t remaining,bool honor_disabled=true,bool model_indices=false) {
    require(payload.size()>=8,"truncated_dependency_header");
    Reader header(payload);
    const auto owner=header.u16(),relation=header.u16(),flags=header.u16(),count=header.u16();
    const auto format=(flags>>10)&15u;
    if(format>8 || (honor_disabled && (flags&1)) || count==0)return Json::array();
    require(format<=1 || (model_indices && format==8),"dependency_format_requires_context");
    require(!(format==0 && owner==10000 && relation==4),"dependency_owner_path_requires_context");
    require(count<=remaining,"work_limit_exceeded");
    const std::size_t needed=8+std::size_t(count)*(format==0?8:16);
    require(payload.size()>=needed,"truncated_dependency_entries");
    // Source decoding may remain partial for other flag bits, format 1's
    // second word, or a suffix; the direct-ID field is nevertheless known.
    auto decoded=native_dependency_link(slice(payload,0,needed));
    require(!decoded.contains("error") && decoded.at("entries").size()==count,"truncated_dependency_entries");
    if(format==8)for(auto &entry:decoded.at("entries")) {
        const auto reference=entry.at("references")[0];
        entry["element_id"]=reference.at("element_id");entry["model_id"]=reference.at("model_index");
    }
    return std::move(decoded.at("entries"));
}
struct LoadDependencyEntries {
    Json references;
    std::vector<std::uint64_t> path;
    unsigned path_format=9;
};
LoadDependencyEntries load_dependency_entries(const Bytes &payload,std::size_t &remaining) {
    require(payload.size()>=8,"truncated_dependency_header");
    const auto flags=Reader(payload,4).u16(),count=Reader(payload,6).u16();
    const auto format=(flags>>10)&15u;
    if(format>8 || (flags&1) || count==0)
        return {direct_dependency_entries(payload,remaining,true,true),{},9};
    if(format==6 || (format==0 && Reader(payload).u16()==10000 && Reader(payload,2).u16()==4)) {
        require(payload.size()>=(format==6?24u:8u),"truncated_dependency_owner_path");
        const auto n=format==6?Reader(payload,8).u32():std::uint32_t(count);
        const std::size_t offset=format==6?24:8;
        require(n<=(payload.size()-offset)/8,"truncated_dependency_owner_path");
        require(n<=remaining,"work_limit_exceeded");remaining-=n;
        const auto slots=std::size_t(count)*(format==6?2u:1u);
        require(slots<=remaining,"work_limit_exceeded");
        LoadDependencyEntries out{Json::array(),{},format};out.path.reserve(n);
        Reader path(payload,offset);
        for(std::uint32_t i=0;i<n;++i)out.path.push_back(path.u64());
        // One shared path, not a copy per iteration (special format 0 would
        // otherwise allocate quadratic memory before its first transition).
        for(std::size_t i=0;i<slots;++i)
            out.references.push_back({{"element_id",UINT64_MAX},{"path_slot",i}});
        return out;
    }
    if(format<=1 || format==8)return {direct_dependency_entries(payload,remaining,true,true),{},9};
    static constexpr unsigned strides[]={8,16,40,48,16,24,0,24,16};
    require(count<=remaining,"work_limit_exceeded");
    const auto needed=8+std::size_t(count)*strides[format];
    require(payload.size()>=needed,"truncated_dependency_entries");
    const auto decoded=native_dependency_link(slice(payload,0,needed));
    require(!decoded.contains("error") && decoded.at("entries").size()==count,"truncated_dependency_entries");
    Json references=Json::array();
    for(const auto &entry:decoded.at("entries")) {
        require(!entry.contains("reference_status"),"compact_dependency_selector_requires_context");
        for(const auto &reference:entry.at("references")) {
            require(references.size()<remaining,"work_limit_exceeded");
            references.push_back(reference);
        }
    }
    return {std::move(references),{},9};
}
}
NativeDependencyLoadResult project_native_dependency_load(const NativeDependencyLoadInput &input) {
    NativeDependencyLoadResult out;
    auto fail=[&](const std::string &reason) {
        out.reason=reason;out.dependents.clear();out.pending_entities.clear();out.batches.clear();out.system_dependents.clear();out.file_dependents.clear();
        return out;
    };
    if(!input.input_complete)return fail("incomplete_input");
    if(!input.monitored_entity_set_known_empty)return fail("monitored_entities_require_context");
    if(input.entities.size()>input.max_work_items)return fail("work_limit_exceeded");
    std::size_t remaining=input.max_work_items;
    auto tick=[&](){require(remaining!=0,"work_limit_exceeded");--remaining;};
    try {
        std::unordered_map<std::uint64_t,std::size_t> system_registry;
        if(input.system_registry) {
            require(!input.system_registry_known_empty || input.system_registry->empty(),"contradictory_system_registry");
            require(input.system_registry->size()<=remaining,"work_limit_exceeded");
            out.system_dependents.resize(input.system_registry->size());
            for(std::size_t i=0;i<input.system_registry->size();++i) {
                tick();const auto id=input.system_registry->at(i).assigned_id;
                if(id)require(system_registry.emplace(id,i).second,"system_assigned_id_collision");
            }
        }
        std::unordered_map<std::uint64_t,std::size_t> registry;
        std::vector<std::unordered_map<std::uint64_t,std::size_t>> file_registries;
        std::map<std::int32_t,std::size_t> file_models;
        if(input.file_context) {
            const auto &context=*input.file_context;
            require(context.complete,"file_model_registry_requires_context");
            require(context.current_model_id!=-1,"current_model_must_be_ordinary");
            require(!input.file_fallback_disabled || !context.current_file_fallback_enabled.value_or(false),"contradictory_file_fallback");
            require(context.models.size()<=remaining,"work_limit_exceeded");
            file_registries.resize(context.models.size());out.file_dependents.resize(context.models.size());
            for(std::size_t m=0;m<context.models.size();++m) {
                tick();const auto &model=context.models[m];
                require(model.model_id!=-1 && model.model_id!=context.current_model_id && file_models.emplace(model.model_id,m).second,
                        "file_model_id_collision");
                require(model.entities.size()<=remaining,"work_limit_exceeded");
                out.file_dependents[m].resize(model.entities.size());
                for(std::size_t i=0;i<model.entities.size();++i) {
                    tick();const auto id=model.entities[i].assigned_id;
                    if(id)require(file_registries[m].emplace(id,i).second,"file_assigned_id_collision");
                }
            }
            file_models.emplace(context.current_model_id,context.models.size());
        }
        struct Target {unsigned scope;std::size_t model,index;}; // 0 current, 1 system, 2 file model
        auto system_lookup=[&](std::uint64_t id)->std::optional<Target> {
            require(input.system_registry.has_value() || input.system_registry_known_empty,"system_registry_requires_context");
            const auto found=system_registry.find(id);
            if(found==system_registry.end())return {};
            return Target{1,0,found->second};
        };
        auto model_lookup=[&](std::uint64_t id,unsigned scope,std::size_t model)->std::optional<Target> {
            if(scope==1)return system_lookup(id);
            const auto &index=scope==0?registry:file_registries.at(model);
            const auto found=index.find(id);
            if(found!=index.end())return Target{scope,model,found->second};
            return system_lookup(id);
        };
        auto target_flags=[&](const Target &target) {
            if(target.scope==0)return input.entities.at(target.index).runtime_flags_10;
            if(target.scope==1)return input.system_registry->at(target.index).runtime_flags_10;
            return input.file_context->models.at(target.model).entities.at(target.index).runtime_flags_10;
        };
        struct OwnerContext {unsigned scope=0;std::size_t model=0;std::optional<std::size_t> reference;};
        auto model_context=[&](std::int32_t id) {
            if(id==-1)return OwnerContext{1,0,{}};
            require(input.file_context.has_value(),"file_model_registry_requires_context");
            if(id==input.file_context->current_model_id)return OwnerContext{};
            const auto found=file_models.find(id);
            require(found!=file_models.end(),"owner_bound_model_requires_context");
            return OwnerContext{2,found->second,{}};
        };
        using ContextKey=std::pair<unsigned,std::size_t>;
        auto context_key=[](const OwnerContext &context)->ContextKey {
            return context.reference?ContextKey{3,*context.reference}:ContextKey{context.scope,context.model};
        };
        std::map<ContextKey,std::vector<std::size_t>> reference_children;
        if(input.owner_references) {
            const auto &refs=*input.owner_references;
            require(refs.size()<=remaining,"work_limit_exceeded");
            for(std::size_t i=0;i<refs.size();++i) {
                tick();const auto &parent=refs[i].parent;
                require(!(parent.model_id && parent.reference_index),"ambiguous_owner_reference_parent");
                ContextKey key{0,0};
                if(parent.reference_index) {
                    require(*parent.reference_index<refs.size(),"owner_reference_parent_out_of_range");
                    key={3,*parent.reference_index};
                } else if(parent.model_id)key=context_key(model_context(*parent.model_id));
                reference_children[key].push_back(i);
            }
            // Parent links describe object ownership, not model bindings.
            // Validate in linear time, including forward references.
            std::vector<unsigned char> state(refs.size());
            for(std::size_t i=0;i<refs.size();++i) {
                std::vector<std::size_t> chain;std::optional<std::size_t> p=i;
                while(p && state[*p]!=2) {
                    tick();require(state[*p]!=1,"owner_reference_parent_cycle");
                    state[*p]=1;chain.push_back(*p);p=refs[*p].parent.reference_index;
                }
                for(auto index:chain)state[index]=2;
            }
        }
        struct OwnerProfile {unsigned type;const Bytes *source=nullptr;};
        auto owner_profile=[&](const Target &owner) {
            bool plain=false,reference=false;
            const std::optional<Bytes> *source=nullptr,*path=nullptr;
            std::uint64_t id=0;
            if(owner.scope==0) {
                const auto &entity=input.entities[owner.index];
                plain=entity.standard_type33_root_owner;reference=entity.standard_type13_identity_root_owner;
                source=&entity.standard_type13_root_source;path=&entity.standard_type47_root_source;id=entity.assigned_id;
            } else {
                const auto &entity=owner.scope==1?input.system_registry->at(owner.index):
                    input.file_context->models[owner.model].entities[owner.index];
                plain=entity.standard_type33_root_owner;reference=entity.standard_type13_identity_root_owner;
                source=&entity.standard_type13_root_source;path=&entity.standard_type47_root_source;id=entity.assigned_id;
            }
            require(unsigned(plain)+unsigned(reference)+unsigned(source->has_value())+unsigned(path->has_value())<=1,
                    "contradictory_owner_profile");
            if(*path) {tick();dependency_path_header(**path,id);return OwnerProfile{47,&**path};}
            if(*source) {tick();dependency_owner_source(**source,id);reference=true;}
            if(plain)return OwnerProfile{33};
            require(reference,"dependency_owner_path_requires_context");
            return OwnerProfile{13};
        };
        auto owner_path_context=[&](std::uint64_t id,const OwnerContext &initial)->std::optional<OwnerContext> {
            auto find_owner=[&](std::uint64_t target_id,const std::optional<OwnerContext> &context,bool outer)->std::optional<Target> {
                if(!context)return {};
                const auto owner=model_lookup(target_id,context->scope,context->model);
                if(!owner)return {};
                const auto flags=target_flags(*owner);
                require(flags.has_value(),"owner_runtime_flags_require_context");
                if(*flags&8) {
                    if(!outer)return {}; // Nested type-47 paths always reject deleted nodes.
                    require(input.owner_lookup_includes_deleted.has_value(),"owner_lookup_mode_requires_context");
                    if(!*input.owner_lookup_includes_deleted)return {};
                }
                return owner;
            };
            const auto first=find_owner(id,initial,true);
            if(!first)return {};
            // A shared native collector may succeed without an owner. Keep its
            // state across child expansions; do not substitute the caller for
            // a format-6 terminal or discard a previous context on empty paths.
            std::optional<OwnerContext> collected_owner;
            using ActiveKey=std::tuple<unsigned,std::size_t,std::size_t,unsigned,std::size_t>;
            std::set<ActiveKey> active;
            struct Frame {
                Target target;std::optional<OwnerContext> caller;
                bool entered=false,waiting=false;
                DependencyOwnerProgram program{};
                std::size_t cursor=0;
                ActiveKey key{};
            };
            std::vector<Frame> stack{{*first,initial}};
            bool initial_step=true; // The caller already charged the outer selector/transition.
            while(!stack.empty()) {
                if(!initial_step)tick();
                initial_step=false;auto &frame=stack.back();
                if(!frame.entered) {
                    const auto profile=owner_profile(frame.target);
                    if(profile.type==33) {
                        if(frame.caller)collected_owner=frame.caller;
                        stack.pop_back();continue;
                    }
                    if(profile.type==13) {
                        require(frame.caller.has_value(),"owner_reference_caller_requires_context");
                        require(input.owner_references.has_value() && input.owner_reference_lists_complete,"owner_reference_list_requires_context");
                        const auto &caller=*frame.caller;
                        const auto source_id=frame.target.scope==0?input.entities[frame.target.index].assigned_id:
                            frame.target.scope==1?input.system_registry->at(frame.target.index).assigned_id:
                            input.file_context->models[frame.target.model].entities[frame.target.index].assigned_id;
                        const auto found=reference_children.find(context_key(caller));
                        bool selected=false;
                        if(found!=reference_children.end())for(auto index:found->second) {
                            tick();const auto &ref=input.owner_references->at(index);
                            if(ref.source_id!=source_id)continue;
                            require(ref.bound_model_id.has_value(),"owner_reference_loading_requires_context");
                            auto next=model_context(*ref.bound_model_id);next.reference=index;collected_owner=next;selected=true;break;
                        }
                        require(selected,"owner_reference_creation_requires_context");
                        stack.pop_back();continue;
                    }
                    const auto key=context_key(*frame.caller);
                    frame.key={frame.target.scope,frame.target.model,frame.target.index,key.first,key.second};
                    require(active.insert(frame.key).second,"cyclic_owner_reference_path");
                    frame.program=dependency_owner_program(*profile.source,remaining);
                    if(!frame.program.valid)return {};
                    frame.cursor=frame.program.ids.size();frame.entered=true;
                    if(frame.program.format==0 && frame.program.ids.empty() && !collected_owner)
                        collected_owner=frame.caller;
                }
                if(frame.waiting) {frame.caller=collected_owner;frame.waiting=false;}
                const auto limit=frame.program.format==6?1u:0u;
                if(frame.cursor>limit) {
                    const auto next=find_owner(frame.program.ids[--frame.cursor],frame.caller,false);
                    if(!next)return {};
                    frame.waiting=true;
                    const auto caller=frame.caller;
                    stack.push_back({*next,caller});continue;
                }
                if(frame.program.format==6) {
                    const auto terminal=find_owner(frame.program.ids.front(),frame.caller,false);
                    if(!terminal)return {};
                    // Audit the appended terminal; type-13/47 is not expanded.
                    owner_profile(*terminal);
                }
                active.erase(frame.key);stack.pop_back();
            }
            // All audited collected roots are non-block, parent-free records.
            // A single standard type-33/13/47 has no custom local transform;
            // a type-47 terminal is not expanded again by its transform query.
            // The remaining transform is the selected reference parent chain.
            return collected_owner;
        };
        auto owner_transition=[&](std::uint64_t id,const OwnerContext &context)->std::optional<OwnerContext> {
            tick();auto next=owner_path_context(id,context);
            if(!next || next->reference)return next;
            require(input.standard_model_owner_transition_known_null,"model_owner_transition_requires_context");
            return {}; // Original ordinary model virtual +58 returns null.
        };
        auto lookup=[&](std::uint64_t id,unsigned scope,std::size_t model)->std::optional<Target> {
            if(auto target=model_lookup(id,scope,model))return target;
            std::optional<bool> fallback;
            if(input.file_context) {
                const auto &context=*input.file_context;
                fallback=scope==0?context.current_file_fallback_enabled:
                    scope==1?context.system_file_fallback_enabled:context.models.at(model).file_fallback_enabled;
            }
            if(scope==0 && input.file_fallback_disabled)fallback=false;
            require(fallback.has_value(),"file_fallback_requires_context");
            if(!*fallback)return {};
            require(input.file_context.has_value(),"file_model_registry_requires_context");
            // 129a00 searches the system once, then resident models in signed
            // int32 key order. It calls 19a8f0, not recursive file fallback.
            if(auto target=system_lookup(id))return target;
            for(const auto &[key,m]:file_models) {
                tick();
                if(auto target=model_lookup(id,m==file_registries.size()?0:2,m==file_registries.size()?0:m))return target;
            }
            return {};
        };
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
                    const auto entries=load_dependency_entries(payload,remaining);
                    for(const auto &entry:entries.references) {
                        tick();auto id=entry.at("element_id").get<std::uint64_t>();
                        std::optional<Target> target;
                        if(entries.path_format!=9) {
                            const auto &path=entries.path;
                            const auto slot=entry.at("path_slot").get<std::size_t>();
                            if(entries.path_format==0 || (!path.empty() && slot==0)) {
                                const std::size_t terminal=entries.path_format==0?slot:1;
                                std::optional<OwnerContext> context=OwnerContext{};
                                OwnerContext previous;
                                for(std::size_t i=path.size();i>terminal;--i) {
                                    previous=*context;context=owner_transition(path[i-1],*context);
                                    if(!context)break;
                                }
                                if(context) {
                                    id=entries.path_format==0?path[slot]:path.front();
                                    const auto &selected=entries.path_format==0?previous:*context;
                                    target=model_lookup(id,selected.scope,selected.model);
                                }
                            } else if(!path.empty() && slot==1) {
                                id=path.back();target=model_lookup(id,0,0);
                            }
                            // Later format-6 iterations retain MAX/null slots,
                            // even after the first iteration found a target.
                        } else if(entry.value("target_lookup",std::string())=="skipped_maximum_id") {
                            // 1f14a0 returns a null target but preserves the ID;
                            // a nonzero missing ID still queues this dependent.
                        } else if(entry.contains("owner_reference_id") && entry.at("owner_reference_id")!=0) {
                            if(auto context=owner_path_context(entry.at("owner_reference_id").get<std::uint64_t>(),OwnerContext{})) {
                                // 1f14a0 also evaluates the selected reference
                                // and its ancestors before looking up the ID.
                                auto ancestor=context->reference;
                                std::vector<Json> transforms;
                                while(ancestor) {
                                    tick();const auto &ref=input.owner_references->at(*ancestor);
                                    require(!(ref.standard_identity_input && ref.affine_input),"contradictory_owner_reference_transform");
                                    require(ref.standard_identity_input || ref.affine_input,"owner_reference_transform_requires_context");
                                    if(ref.affine_input) {
                                        const auto &affine=*ref.affine_input;
                                        require(affine.context.force_z_scale,"owner_reference_chain_requires_forced_z_scale");
                                        require(affine.context.origin.model_attached!=std::optional<bool>(false),
                                                "owner_reference_affine_attachment_mismatch");
                                        auto result=reference_affine_transform(dependency_owner_source(affine.source_record,ref.source_id),affine.context);
                                        require(result.value("status","")=="computed",
                                                "owner_reference_affine_unresolved: "+result.value("reason",std::string()));
                                        transforms.push_back(std::move(result));
                                    } else {
                                        transforms.push_back({{"profile","bimbase_2025_reference_affine_query"},
                                            {"status","computed"},{"force_z_scale",true},
                                            {"matrix",Matrix4{{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}}}});
                                    }
                                    ancestor=ref.parent.reference_index;
                                }
                                const auto chain=compose_owner_reference_chain_transforms(transforms);
                                require(chain.value("status","")=="computed",
                                        "owner_reference_chain_unresolved: "+chain.value("reason",std::string()));
                                target=model_lookup(id,context->scope,context->model);
                            }
                        } else if(entry.value("lookup_profile",std::string())=="owner_system")target=model_lookup(id,0,0);
                        else if(!entry.contains("model_id"))target=lookup(id,0,0);
                        else {
                            const auto model_id=entry.at("model_id").get<std::int32_t>();
                            if(model_id==-1)target=lookup(id,1,0);
                            else {
                                require(input.file_context.has_value(),"file_model_registry_requires_context");
                                if(model_id==input.file_context->current_model_id)target=lookup(id,0,0);
                                else if(const auto found=file_models.find(model_id);found!=file_models.end())
                                    target=lookup(id,2,found->second);
                                // Missing resident model passes null to 1f12b0;
                                // it does not search the system or other models.
                            }
                        }
                        if(!target) {
                            if(id!=0 && !pending[index]) {
                                pending[index]=true;result.newly_pending_entities.push_back(index);
                            }
                            continue;
                        }
                        const auto t=target->index,m=target->model;
                        const auto flags=target_flags(*target);
                        require(flags.has_value(),target->scope==0?"target_runtime_flags_require_context":
                            target->scope==1?"system_target_runtime_flags_require_context":"file_target_runtime_flags_require_context");
                        // Mode 1 and a fresh service skip rejected targets. They
                        // are distinct from a missing nonzero ID, which queues.
                        if(*flags&0x20008u)continue;
                        if(target->scope==0) {
                            out.dependents[t].push_back(index);result.added_edges.push_back({t,index});
                        } else if(target->scope==1) {
                            out.system_dependents[t].push_back(index);result.added_system_edges.push_back({t,index});
                        } else {
                            out.file_dependents[m][t].push_back(index);result.added_file_edges.push_back({m,t,index});
                        }
                    }
                }
            }
            out.batches.push_back(std::move(result));
        }
        require(std::all_of(registered.begin(),registered.end(),[](bool v){return v;}),"entity_not_registered");
        for(auto &list:out.dependents)std::reverse(list.begin(),list.end());
        for(auto &list:out.system_dependents)std::reverse(list.begin(),list.end());
        for(auto &model:out.file_dependents)for(auto &list:model)std::reverse(list.begin(),list.end());
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
