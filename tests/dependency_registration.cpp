#include "internal.hpp"
#include <p3d/dependency_registration.hpp>
#include "dependency_registration_oracle.hpp"
#include "dependency_retry_oracle.hpp"
#include "dependency_cycle_oracle.hpp"
#include "dependency_held_cycle_oracle.hpp"
#include "dependency_flush_oracle.hpp"
#include "dependency_system_oracle.hpp"
#include "dependency_models_oracle.hpp"
#include "dependency_selectors_oracle.hpp"
#include "dependency_paths_oracle.hpp"
#include "dependency_reference_owners_oracle.hpp"
#include "dependency_reference_affines_oracle.hpp"
#include "dependency_type47_oracle.hpp"

using namespace p3d;
unsigned dependency_registration_tests() {
    unsigned checks=0;
    auto check=[&](bool value,const char *message){++checks;require(value,message);};
    auto unhex=[](const std::string &text){
        Bytes result;for(std::size_t i=0;i<text.size();i+=2)
            result.push_back(std::uint8_t(std::stoul(text.substr(i,2),nullptr,16)));
        return result;
    };
    const auto oracle=Json::parse(dependency_registration_oracle);
    check(oracle.at("cases").size()==172,"all original dependency forests are tested");
    const auto retry_oracle=Json::parse(dependency_retry_oracle);
    check(retry_oracle.at("cases").size()==804,"all native pending retry observations are tested");
    auto cases=oracle.at("cases");
    for(const auto &row:retry_oracle.at("cases"))cases.push_back(row);
    const auto cycle_oracle=Json::parse(dependency_cycle_oracle);
    check(cycle_oracle.at("cases").size()==176,"all complete native iteration observations are tested");
    for(const auto &row:cycle_oracle.at("cases"))cases.push_back(row);
    const auto held_oracle=Json::parse(dependency_held_cycle_oracle);
    check(held_oracle.at("cases").size()==804,"all original constructed-model iterations are tested");
    for(const auto &row:held_oracle.at("cases"))cases.push_back(row);
    const auto flush_oracle=Json::parse(dependency_flush_oracle);
    check(flush_oracle.at("cases").size()==804,"all original outer flush observations are tested");
    for(const auto &row:flush_oracle.at("cases"))cases.push_back(row);
    std::size_t case_index=0;
    for(const auto &row:cases) {
        const bool retry=row.contains("before_retry");
        const auto parents=row.at("parents").get<std::vector<int>>();
        NativeDependencyLoadInput input;
        input.input_complete=input.system_registry_known_empty=input.file_fallback_disabled=
            input.monitored_entity_set_known_empty=true;
        Json list={{"status","resolved"},{"system_bootstrap_required",true},{"roots",Json::array()}},records=Json::array();
        std::vector<std::size_t> ancestors(parents.size());
        for(std::size_t i=0;i<parents.size();++i) {
            ancestors[i]=parents[i]<0?i:ancestors.at(std::size_t(parents[i]));
            records.push_back({{"id",row.at("ids")[i]}});
            NativeDependencyLoadEntity entity;entity.runtime_flags_10=row.at("entities")[i].at("flags");
            if(retry)entity.runtime_flags_10=row.value("post_root_flags",Json::object()).value(std::to_string(i),0u);
            for(const auto &payload:row.at("dependency_payloads")[i])entity.dependency_payloads.push_back(unhex(payload));
            input.entities.push_back(entity);
        }
        for(const auto &call:row.at("calls")) {
            const auto root=call.at("root").get<std::size_t>();Json headers=Json::array();
            std::vector<std::size_t> batch;
            for(std::size_t i=0;i<parents.size();++i)if(ancestors[i]==root) {
                batch.push_back(i);
                headers.push_back({{"status","resolved"},{"native_record_index",i},
                    {"parent_record_index",parents[i]<0?Json():Json(parents[i])}});
            }
            input.batches.push_back(batch);
            list["roots"].push_back({{"native_record_index",root},{"block_number",1},{"headers",headers}});
        }
        const Json file={{"status","resolved"},{"initial_probe",{{"action","read_header_payload"},{"id_counter",row.at("initial_counter")}}}};
        const auto assigned=native_system_id_assignments(list,records,file);
        for(const auto &root:assigned.at("roots"))for(const auto &entry:root.at("records")) {
            const auto index=entry.at("native_record_index").get<std::size_t>();
            input.entities[index].assigned_id=entry.at("assigned_id");
            check(entry.at("assigned_id")==row.at("entities")[index].at("id"),"dependency lookup uses library-assigned IDs matching native registry");
        }
        const auto result=project_native_dependency_load(input);
        require(result.resolved,"dependency case "+std::to_string(case_index)+": "+result.reason);
        ++case_index;
        check(result.resolved && result.batches.size()==input.batches.size(),"complete dependency callback projection resolves");
        std::vector<std::vector<std::size_t>> observed_lists(parents.size());std::set<std::size_t> pending;
        for(std::size_t b=0;b<result.batches.size();++b) {
            for(const auto &edge:result.batches[b].added_edges)
                observed_lists[edge.target_entity].insert(observed_lists[edge.target_entity].begin(),edge.dependent_entity);
            for(auto index:result.batches[b].newly_pending_entities)pending.insert(index);
            check(row.at("calls")[b].at("dependents")==observed_lists,"reverse lists match native after each complete root callback");
            check(row.at("calls")[b].at("context").at("pending_entities")==std::vector<std::size_t>(pending.begin(),pending.end()),
                  "missing nonzero targets queue dependents without resolving forward roots prematurely");
        }
        check(result.dependents==observed_lists,"final reverse lists preserve prepending order");
        check(result.pending_entities==std::vector<std::size_t>(pending.begin(),pending.end()),"final pending membership is preserved");
        if(!retry) {
            for(std::size_t i=0;i<parents.size();++i)
                check(row.at("entities")[i].at("dependents")==result.dependents[i],"duplicate and self-reference nodes match original allocation");
        } else {
            NativeDependencyRetryInput next;
            next.entities=input.entities;next.dependents=result.dependents;
            next.input_complete=next.system_registry_known_empty=next.file_fallback_disabled=next.monitored_entity_set_known_empty=true;
            next.model_notified=row.at("notice_list_flag").get<unsigned>()!=0 || row.at("notice_model_flag").get<unsigned>()!=0;
            next.pending_iteration_order=row.at("before_retry").at("context").at("pending_iteration_order").get<std::vector<std::size_t>>();
            auto membership=next.pending_iteration_order;std::sort(membership.begin(),membership.end());
            check(membership==result.pending_entities,"native pointer order only orders the library's computed pending membership");
            check(row.at("before_retry").at("dependents")==result.dependents,"retry starts with library-computed reverse lists");
            if(row.contains("pre_retry_flags"))for(auto it=row.at("pre_retry_flags").begin();it!=row.at("pre_retry_flags").end();++it)
                next.entities[std::stoul(it.key())].runtime_flags_10=it.value().get<std::uint32_t>();
            const auto retried=project_native_dependency_retry(next);
            require(retried.resolved,"retry case "+std::to_string(case_index)+": "+retried.reason);
            const auto &after=row.at("after_retry");
            const bool cycle=row.contains("cycle_return");
            if(cycle) {
                NativeDependencyCycleInput complete;
                static_cast<NativeDependencyRetryInput &>(complete)=next;
                complete.other_work_queues_known_empty=complete.standard_entities_known=
                    complete.caller_holds_model=complete.link_update_handlers_known_absent=true;
                const auto completed=project_native_dependency_cycle(complete);
                require(completed.resolved,"complete iteration case "+std::to_string(case_index)+": "+completed.reason);
                check(after.at("dependents")==completed.dependents,"complete projection agrees with original iteration graph");
                check(after.at("context").at("pending_entities")==completed.pending_entities,
                      "complete iteration consumes pending work even with missing targets");
                check(after.at("context").at("monitored_entities")==completed.monitored_entities,
                      "unresolved references remain monitored after pending work is consumed");
                check(completed.scheduled_pairs.empty(),"complete iteration consumes scheduled pairs");
                if(row.contains("flush_state")) {
                    const auto &state=row.at("flush_state");
                    check(state.at("config_vtable")=="0x52edf8" && state.at("notification_vtable")=="0x535cf0" &&
                          state.at("transaction_vtable")=="0x52ec90","outer flush uses original config, host notification and default transaction objects");
                    check(state.at("transaction_restored")==true && state.at("transaction_stack_size")==0 &&
                          state.at("running")==0 && state.at("callback_depth")==0,
                          "outer flush restores the transaction and balances configuration stack and execution state");
                }
                if(row.contains("native_after")) {
                    const auto &meta=row.at("native_after");
                    check(meta.at("file_vtable")=="0x52eef8" && meta.at("model_vtable")=="0x5333c8" &&
                          meta.at("model_host_vtable")=="0x544230","original constructors provide actual file/model/host services");
                    check(meta.at("model_reference_count")==1 && meta.at("file_reference_count")==1 &&
                          meta.at("file_held_model_count")==1 && meta.at("file_model_count")==1,
                          "iteration restores the caller hold and preserves active model membership");
                    std::size_t bytes=0;
                    for(const auto &entity:row.at("entities"))bytes+=(entity.at("source_header").get<std::string>().size()/2+7)&~std::size_t(7);
                    check(meta.at("storage_page_used")==bytes && meta.at("storage_page_bytes").get<std::size_t>()>=bytes,
                          "original storage allocator retains every aligned record");
                }
                NativeDependencyNormalizationInput normalized_input;
                normalized_input.dependents=retried.dependents;normalized_input.scheduled_pairs=retried.scheduled_pairs;
                normalized_input.input_complete=normalized_input.standard_entities_known=normalized_input.removal_work_known_empty=true;
                const auto normalized=project_native_dependency_normalization(normalized_input);
                check(normalized.resolved && after.at("dependents")==normalized.dependents,
                      "library load, retry and normalization reproduce a complete native service iteration");
                check(row.at("cycle_return")==0 && after.at("context").at("registry_work_count")==0,"native iteration completes and drains work");
                check(after.at("context").at("scheduled_pairs").empty(),"scheduled pairs are consumed by complete iteration");
                check(after.at("context").at("registry_set_counts")==std::vector<unsigned>(18,0),"all work sets are empty after this bounded iteration");
                for(std::size_t i=0;i<normalized.dependents.size();++i) {
                    std::vector<std::size_t> replay;
                    for(auto position:normalized.retained_positions[i])replay.push_back(retried.dependents[i].at(position));
                    check(replay==normalized.dependents[i],"retained positions identify the exact surviving list occurrences");
                }
            } else check(after.at("dependents")==retried.dependents,"retry preserves duplicates and matches native reverse-list ordering");
            if(!cycle)check(after.at("context").at("pending_entities")==retried.pending_entities,"retry requeues missing or excluded targets");
            check(after.at("context").at("monitored_entities")==retried.monitored_entities,"remaining pending members become monitored");
            Json pairs=Json::array();for(const auto &edge:retried.scheduled_pairs)pairs.push_back({edge.target_entity,edge.dependent_entity});
            if(!cycle) {
                check(after.at("context").at("scheduled_pairs")==pairs,"retry schedules distinct target-dependent pairs");
                check(after.at("context").at("registry_work_count").get<std::size_t>()==1+retried.added_edges.size(),
                      "each inserted reference contributes work even if a pair was already scheduled");
            }
            auto applied=result.dependents;
            for(const auto &edge:retried.added_edges)applied[edge.target_entity].insert(applied[edge.target_entity].begin(),edge.dependent_entity);
            check(applied==retried.dependents,"retry action order reconstructs final lists");
        }
    }
    const auto system_oracle=Json::parse(dependency_system_oracle);
    check(system_oracle.at("cases").size()==80,"all original populated-system experiments are tested");
    const auto models_oracle=Json::parse(dependency_models_oracle);
    check(models_oracle.at("cases").size()==72,"all original format-8 resident-model experiments are tested");
    auto model_cases=system_oracle.at("cases");
    for(const auto &row:models_oracle.at("cases"))model_cases.push_back(row);
    const auto selectors_oracle=Json::parse(dependency_selectors_oracle);
    check(selectors_oracle.at("cases").size()==344,"all original selector/owner experiments are tested");
    for(const auto &row:selectors_oracle.at("cases"))model_cases.push_back(row);
    const auto paths_oracle=Json::parse(dependency_paths_oracle);
    check(paths_oracle.at("cases").size()==1024,"all original owner-path experiments are tested");
    for(const auto &row:paths_oracle.at("cases"))model_cases.push_back(row);
    for(const auto &row:model_cases) {
        NativeDependencyLoadInput input;
        input.input_complete=input.file_fallback_disabled=input.monitored_entity_set_known_empty=true;
        input.standard_model_owner_transition_known_null=row.value("standard_model_owner_transition_known_null",false);
        input.system_registry.emplace();
        for(const auto &id:row.at("system_ids"))
            input.system_registry->push_back({id.get<std::uint64_t>(),row.at("system_flags").get<std::uint32_t>()});
        if(row.value("plain_root_owner_profile",false)) {
            input.owner_lookup_includes_deleted=row.value("owner_lookup_includes_deleted",false);
            for(auto &entity:*input.system_registry)entity.standard_type33_root_owner=true;
        }
        std::vector<std::vector<std::vector<std::size_t>>> file_lists;
        if(row.contains("file_models")) {
            input.file_context.emplace();auto &context=*input.file_context;
            context.complete=true;context.current_model_id=7;
            context.current_file_fallback_enabled=false;context.system_file_fallback_enabled=true;
            for(const auto &model:row.at("file_models")) {
                NativeDependencyFileModel m;m.model_id=model.at("model_id");m.file_fallback_enabled=false;
                for(const auto &id:model.at("ids"))m.entities.push_back({id.get<std::uint64_t>(),model.at("flags").get<std::uint32_t>()});
                file_lists.emplace_back(m.entities.size());context.models.push_back(std::move(m));
            }
        }
        for(std::size_t i=0;i<row.at("ids").size();++i) {
            NativeDependencyLoadEntity entity;
            entity.assigned_id=row.at("ids")[i];
            entity.runtime_flags_10=entity.assigned_id==41?row.at("local_flags").get<std::uint32_t>():0;
            entity.standard_type33_root_owner=row.value("plain_root_owner_profile",false);
            check(*entity.runtime_flags_10==row.at("runtime_flags")[i],"explicit target state agrees with native system experiment");
            for(const auto &payload:row.at("dependency_payloads")[i])entity.dependency_payloads.push_back(unhex(payload));
            input.entities.push_back(std::move(entity));input.batches.push_back({i});
        }
        const auto result=project_native_dependency_load(input);
        require(result.resolved,"populated system projection: "+result.reason);
        check(result.batches.size()==input.batches.size(),"all roots with populated system registry resolve");
        std::vector<std::vector<std::size_t>> local(input.entities.size()),system(input.system_registry->size());
        std::set<std::size_t> pending;
        for(std::size_t i=0;i<result.batches.size();++i) {
            for(const auto &edge:result.batches[i].added_edges)
                local.at(edge.target_entity).insert(local.at(edge.target_entity).begin(),edge.dependent_entity);
            for(const auto &edge:result.batches[i].added_system_edges)
                system.at(edge.target_entity).insert(system.at(edge.target_entity).begin(),edge.dependent_entity);
            for(const auto &edge:result.batches[i].added_file_edges) {
                auto &list=file_lists.at(edge.model_index).at(edge.target_entity);
                list.insert(list.begin(),edge.dependent_entity);
            }
            for(auto p:result.batches[i].newly_pending_entities)pending.insert(p);
            const auto &native=row.at("calls")[i];
            const std::vector<std::vector<std::size_t>> loaded(local.begin(),local.begin()+i+1);
            check(native.at("dependents")==loaded,"local targets take precedence over system targets only after registration");
            check(native.at("system_dependents")==system,"system reverse prefixes match original callbacks after every root");
            if(row.contains("file_models"))
                check(native.at("file_dependents")==file_lists,"format-8 selection and system file fallback match original cross-model edges");
            check(native.at("pending_entities")==std::vector<std::size_t>(pending.begin(),pending.end()),
                  "system hits and excluded targets do not spuriously queue missing-reference work");
        }
        check(result.dependents==local && result.system_dependents==system,"both target namespaces preserve native duplicate and prepend order");
        if(row.contains("file_models"))check(result.file_dependents==file_lists,"other-model prefixes preserve target model identity and head order");
        check(result.pending_entities==std::vector<std::size_t>(pending.begin(),pending.end()),"final pending membership matches native system experiment");
        for(const auto &lookup:row.at("lookups")) {
            Json target;
            for(std::size_t i=0;i<input.entities.size();++i)
                if(input.entities[i].assigned_id==lookup.at("id"))target=Json::array({"local",i});
            if(target.is_null())for(std::size_t i=0;i<input.system_registry->size();++i)
                if(input.system_registry->at(i).assigned_id==lookup.at("id"))target=Json::array({"system",i});
            check(target==lookup.at("target"),"original 19a8f0 and 1f12b0 choose the expected model-qualified target");
        }
    }
    const auto bound_oracle=Json::parse(dependency_reference_owners_oracle);
    check(bound_oracle.at("cases").size()==408,"all original bound-reference callback experiments are tested");
    const auto affine_oracle=Json::parse(dependency_reference_affines_oracle);
    auto bound_cases=bound_oracle.at("cases");
    check(affine_oracle.at("dependency_cases").size()==288,"all nonidentity original graph callbacks are tested");
    for(const auto &row:affine_oracle.at("dependency_cases"))bound_cases.push_back(row);
    const auto path_oracle=Json::parse(dependency_type47_oracle);
    check(path_oracle.at("cases").size()==2030,"all original type47 callback cases are tested");
    for(auto row:path_oracle.at("cases")) {
        row["source_headers"]=Json::array();
        for(const auto &index:row.at("source_indices"))row["source_headers"].push_back(path_oracle.at("source_catalog").at(index.get<std::size_t>()));
        bound_cases.push_back(std::move(row));
    }
    NativeDependencyLoadInput bound_sample,affine_sample,path_sample;
    for(const auto &row:bound_cases) {
        const bool affine=row.contains("transform_case");
        std::map<std::uint64_t,Bytes> affine_sources;
        if(affine)for(const auto &profile:affine_oracle.at("cases"))
            if(profile.at("transform_case")==row.at("transform_case"))
                for(const auto &source:profile.at("source_headers")) {
                    auto data=unhex(source.at("header_hex"));data.insert(data.begin(),4,0);
                    affine_sources[source.at("id").get<std::uint64_t>()]=std::move(data);
                }
        NativeDependencyLoadInput input;
        input.input_complete=input.system_registry_known_empty=input.monitored_entity_set_known_empty=true;
        input.standard_model_owner_transition_known_null=true;
        input.owner_lookup_includes_deleted=row.at("owner_lookup_includes_deleted").get<bool>();
        input.file_context.emplace();auto &file=*input.file_context;
        file.complete=true;file.current_model_id=7;file.current_file_fallback_enabled=false;
        for(int id:{9,10}) {NativeDependencyFileModel m;m.model_id=id;m.file_fallback_enabled=false;file.models.push_back(m);}
        for(const auto &source:row.at("source_headers")) {
            const auto id=source.at("id").get<std::uint64_t>();
            const auto type=source.at("type").get<unsigned>();
            const auto model=source.at("model_id").get<int>();
            auto flags=model==7?(id==42?row.at("owner_flags").get<unsigned>():0):
                (id==41?row.at("target_flags").get<unsigned>():0);
            if(type==47)for(const auto &path:row.at("path_owners"))
                if(path.at("id")==id)flags=path.at("runtime_flags").get<unsigned>();
            if(model!=7) {
                NativeDependencySystemTarget entity{id,flags,type==33,type==13 && !affine};
                if(type==13 && affine)entity.standard_type13_root_source=affine_sources.at(id);
                file.models.at(model==9?0:1).entities.push_back(std::move(entity));
            }
            else {
                NativeDependencyLoadEntity entity{id,flags,{},type==33,type==13 && !affine};
                if(type==13 && affine)entity.standard_type13_root_source=affine_sources.at(id);
                if(type==47) {
                    auto bytes=unhex(source.at("header_hex"));bytes.insert(bytes.begin(),4,0);
                    entity.standard_type47_root_source=std::move(bytes);
                    for(const auto &path:row.at("path_owners"))if(path.at("id")==id)
                        for(const auto &payload:path.at("links"))entity.dependency_payloads.push_back(unhex(payload));
                }
                if(id==77 || id==78)entity.dependency_payloads.push_back(unhex(row.at("payload_hex")));
                input.batches.push_back({input.entities.size()});input.entities.push_back(std::move(entity));
            }
        }
        input.owner_reference_lists_complete=true;
        input.owner_references=std::vector<NativeDependencyOwnerReference>{
            {{},42,9,true},{{9,{}},43,7,true},{{{},0},43,10,true}};
        if(affine)for(auto &ref:*input.owner_references) {
            ref.standard_identity_input=false;
            ref.affine_input.emplace();ref.affine_input->source_record=affine_sources.at(ref.source_id);
            ref.affine_input->context.provider_id=0;
            ref.affine_input->context.origin.model_coordinates=Json{{"status","decoded"},
                {"reference_origin",{{"value",Point3{0,0,0}}}}};
        }
        const auto result=project_native_dependency_load(input);
        require(result.resolved,"bound reference projection: "+result.reason);
        check(result.batches.size()==row.at("calls").size(),"all roots with bound references complete");
        std::vector<std::vector<std::size_t>> local(input.entities.size());
        std::vector<std::vector<std::vector<std::size_t>>> others{{{},{}},{{}}};
        std::set<std::size_t> pending;
        for(std::size_t i=0;i<result.batches.size();++i) {
            const auto &batch=result.batches[i];
            for(const auto &edge:batch.added_edges)local.at(edge.target_entity).insert(local.at(edge.target_entity).begin(),edge.dependent_entity);
            for(const auto &edge:batch.added_file_edges) {
                auto &list=others.at(edge.model_index).at(edge.target_entity);list.insert(list.begin(),edge.dependent_entity);
            }
            for(auto p:batch.newly_pending_entities)pending.insert(p);
            const auto &native=row.at("calls")[i];
            check(native.at("local_dependents")==std::vector<std::vector<std::size_t>>(local.begin(),local.begin()+i+1),
                  "bound owner lookup preserves local registration timing and duplicate order");
            check(native.at("file_dependents")==others,"reference context selects its own child list and bound model");
            check(native.at("pending_entities")==std::vector<std::size_t>(pending.begin(),pending.end()),
                  "bound owner paths preserve original pending members after every root");
            check(batch.added_system_edges.empty(),"bound reference cases have no system targets");
        }
        check(result.dependents==local && result.file_dependents==others && result.system_dependents.empty(),
              "bound-reference final graph matches every original target identity");
        check(result.pending_entities==std::vector<std::size_t>(pending.begin(),pending.end()),"bound-reference final pending set agrees");
        if(bound_sample.entities.empty())bound_sample=input;
        if(affine && row.at("format")==4 && !row.at("disabled").get<bool>() && affine_sample.entities.empty())affine_sample=input;
        if(row.contains("path_program") && row.at("path_program")=="nested_reference" && row.at("format")==4 && path_sample.entities.empty())path_sample=input;
    }
    NativeDependencyLoadInput base;
    base.input_complete=base.system_registry_known_empty=base.file_fallback_disabled=base.monitored_entity_set_known_empty=true;
    base.entities={{1,0,{}},{2,0,{unhex("e7030100000001000100000000000000")}}};base.batches={{0},{1}};
    auto rejected=[&](const NativeDependencyLoadInput &input,const char *reason){
        auto result=project_native_dependency_load(input);
        check(!result.resolved && result.reason==reason && result.dependents.empty() && result.pending_entities.empty() && result.batches.empty() && result.system_dependents.empty() && result.file_dependents.empty(),
              "unknown or invalid dependency input never publishes a partial graph");
    };
    auto input=base;input.input_complete=false;rejected(input,"incomplete_input");
    check(!path_sample.entities.empty(),"native nested type47 case supplies guard inputs");
    auto put_path=[](Bytes &bytes,std::size_t offset,std::uint64_t value,unsigned size) {
        for(unsigned i=0;i<size;++i)bytes.at(offset+i)=std::uint8_t(value>>(8*i));
    };
    auto make_path_record=[&](std::uint64_t id,const Bytes &payload) {
        const auto &sample=*path_sample.entities[3].standard_type47_root_source;
        Bytes bytes(sample.begin(),sample.begin()+38);bytes.resize(42+payload.size());
        put_path(bytes,8,(bytes.size()-4)/2,4);put_path(bytes,20,id,8);
        put_path(bytes,38,0x1000+(payload.size()+4)/2-1,2);put_path(bytes,40,0x56d0,2);
        std::copy(payload.begin(),payload.end(),bytes.begin()+42);return bytes;
    };
    input=path_sample;input.entities[3].standard_type33_root_owner=true;rejected(input,"contradictory_owner_profile");
    input=path_sample;input.entities[3].standard_type47_root_source->at(20)=99;rejected(input,"owner_path_source_id_mismatch");
    input=path_sample;input.entities[3].standard_type47_root_source->at(6)|=0x80;rejected(input,"owner_path_source_profile_required");
    input=path_sample;input.entities[3].standard_type47_root_source->at(28)=1;rejected(input,"owner_path_source_profile_required");
    input=path_sample;input.entities[3].standard_type47_root_source->resize(56);
    put_path(*input.entities[3].standard_type47_root_source,8,26,4);rejected(input,"truncated_owner_path_linkage");
    input=path_sample;input.entities[3].standard_type47_root_source=make_path_record(44,unhex("10270400"));
    rejected(input,"truncated_owner_path_header");
    input=path_sample;put_path(*input.entities[3].standard_type47_root_source,48,2,2);rejected(input,"truncated_owner_path_ids");
    input=path_sample;put_path(*input.entities[3].standard_type47_root_source,50,44,8);rejected(input,"cyclic_owner_reference_path");
    input=path_sample;put_path(*input.entities[4].standard_type47_root_source,50,44,8);rejected(input,"cyclic_owner_reference_path");
    input=path_sample;input.entities[4].runtime_flags_10.reset();rejected(input,"owner_runtime_flags_require_context");
    const auto empty_path=unhex("1027040001000000");
    input=path_sample;
    NativeDependencySystemTarget same_id{44,0};same_id.standard_type47_root_source=make_path_record(44,empty_path);
    input.file_context->models[0].entities.push_back(same_id);
    input.entities[3].standard_type47_root_source=make_path_record(44,unhex("10270400010002002c000000000000002a00000000000000"));
    auto type47_result=project_native_dependency_load(input);
    check(type47_result.resolved && type47_result.file_dependents[0][0]==std::vector<std::size_t>{5},
          "same path ID in another model/reference context is not a recursion cycle and empty nested path preserves owner");
    input=path_sample;
    same_id.assigned_id=45;same_id.standard_type47_root_source=make_path_record(45,empty_path);
    input.file_context->models[0].entities.push_back(same_id);
    input.entities[3].standard_type47_root_source=make_path_record(44,unhex("1027040001180100020000000000000000000000000000002d000000000000002a00000000000000"));
    type47_result=project_native_dependency_load(input);
    check(type47_result.resolved && type47_result.file_dependents[0][0]==std::vector<std::size_t>{5},
          "single standard type47 terminal has no local transform and retains the reference owner");
    input.file_context->models[0].entities.back().standard_type47_root_source.reset();
    rejected(input,"dependency_owner_path_requires_context");
    input.file_context->models[0].entities.back().standard_type47_root_source=same_id.standard_type47_root_source;
    input.entities[5].dependency_payloads={unhex("10270400001801000200000000000000000000000000000029000000000000002c00000000000000")};
    check(project_native_dependency_load(input).resolved,"path-only transition does not invent a terminal type47 transform requirement");
    input=affine_sample;
    auto dependent=input.entities.back();input.entities.pop_back();input.batches.pop_back();
    put_path(dependent.dependency_payloads[0],16,1000,8);
    for(std::uint64_t id=1000;id<2000;++id) {
        auto payload=unhex("10270400010001002a00000000000000");
        put_path(payload,8,id==1999?42:id+1,8);
        NativeDependencyLoadEntity node{id,0,{payload}};node.standard_type47_root_source=make_path_record(id,payload);
        input.batches.push_back({input.entities.size()});input.entities.push_back(std::move(node));
    }
    input.batches.push_back({input.entities.size()});input.entities.push_back(std::move(dependent));
    type47_result=project_native_dependency_load(input);
    check(type47_result.resolved && type47_result.file_dependents[0][0]==std::vector<std::size_t>{1003},
          "one thousand nested path roots resolve with an explicit stack rather than native call-stack recursion");
    auto deep_path=input;input.max_work_items=6000;rejected(input,"work_limit_exceeded");
    input=deep_path;put_path(*input.entities[1002].standard_type47_root_source,50,1000,8);
    rejected(input,"cyclic_owner_reference_path");
    check(!affine_sample.entities.empty(),"a native nonidentity paired selector supplies guard tests");
    input=affine_sample;input.entities[1].standard_type13_identity_root_owner=true;rejected(input,"contradictory_owner_profile");
    input=affine_sample;input.entities[1].standard_type13_root_source->pop_back();rejected(input,"owner_reference_source_profile_required");
    input=affine_sample;input.entities[1].standard_type13_root_source->at(28)=1;rejected(input,"owner_reference_source_profile_required");
    input=affine_sample;input.entities[1].standard_type13_root_source->at(20)=99;rejected(input,"owner_reference_source_id_mismatch");
    input=affine_sample;input.owner_references->at(0).standard_identity_input=true;rejected(input,"contradictory_owner_reference_transform");
    input=affine_sample;input.owner_references->at(0).affine_input.reset();rejected(input,"owner_reference_transform_requires_context");
    input=affine_sample;input.owner_references->at(0).affine_input->source_record.at(20)=99;rejected(input,"owner_reference_source_id_mismatch");
    input=affine_sample;input.owner_references->at(0).affine_input->source_record.at(370)=4;rejected(input,"owner_reference_source_profile_required");
    input=affine_sample;input.owner_references->at(0).affine_input->context.force_z_scale=false;rejected(input,"owner_reference_chain_requires_forced_z_scale");
    input=affine_sample;input.owner_references->at(0).affine_input->context.origin.model_attached=false;rejected(input,"owner_reference_affine_attachment_mismatch");
    input=affine_sample;input.owner_references->at(0).affine_input->context.provider_id.reset();
    rejected(input,"owner_reference_affine_unresolved: reference_scale_provider_identity_unknown");
    input=affine_sample;input.owner_references->at(0).affine_input->context.origin={};
    rejected(input,"owner_reference_affine_unresolved: reference_origin_correction_unresolved");
    input=affine_sample;input.owner_references->at(0).affine_input->context.provider_id=0x10000;
    rejected(input,"owner_reference_affine_unresolved: reference_scale_provider_result_unknown");
    input=affine_sample;
    const double infinity=std::numeric_limits<double>::infinity();
    std::memcpy(input.entities[1].standard_type13_root_source->data()+292,&infinity,8);
    rejected(input,"owner_reference_source_nonfinite");
    input=affine_sample;
    std::memcpy(input.owner_references->at(0).affine_input->source_record.data()+196,&infinity,8);
    rejected(input,"owner_reference_source_nonfinite");
    input=affine_sample;
    const double maximum=std::numeric_limits<double>::max();
    std::memcpy(input.owner_references->at(0).affine_input->source_record.data()+292,&maximum,8);
    rejected(input,"owner_reference_affine_unresolved: reference affine point translation overflow");
    input=affine_sample;input.entities[1].standard_type13_root_source->at(6)=0x60;
    input.owner_references->at(0).affine_input->source_record.at(6)=0x60;
    check(project_native_dependency_load(input).resolved,"prepared loaded type13 flag is accepted without changing transform provenance");
    input=affine_sample;input.owner_references->at(0).affine_input->context={};
    for(auto i:{0u,3u})input.entities[i].dependency_payloads={unhex("10270400001801000200000000000000000000000000000029000000000000002a00000000000000")};
    check(project_native_dependency_load(input).resolved,"format6 source transitions do not demand an unused affine service context");
    input=affine_sample;input.entities[1].runtime_flags_10=8;input.owner_lookup_includes_deleted=false;
    input.entities[1].standard_type13_root_source->at(28)=1;
    check(project_native_dependency_load(input).resolved,"deleted owner rejection precedes source profile validation");
    input=bound_sample;input.owner_references.reset();rejected(input,"owner_reference_list_requires_context");
    input=bound_sample;input.owner_reference_lists_complete=false;rejected(input,"owner_reference_list_requires_context");
    input=bound_sample;input.owner_references->clear();rejected(input,"owner_reference_creation_requires_context");
    input=bound_sample;input.owner_references->at(0).bound_model_id.reset();rejected(input,"owner_reference_loading_requires_context");
    input=bound_sample;input.owner_references->at(0).bound_model_id=99;rejected(input,"owner_bound_model_requires_context");
    input=bound_sample;input.owner_references->at(0).parent.reference_index=99;rejected(input,"owner_reference_parent_out_of_range");
    input=bound_sample;input.owner_references->at(0).parent.reference_index=0;rejected(input,"owner_reference_parent_cycle");
    input=bound_sample;input.owner_references->at(0).parent.reference_index=2;rejected(input,"owner_reference_parent_cycle");
    input=bound_sample;input.owner_references->at(1).parent.reference_index=0;rejected(input,"ambiguous_owner_reference_parent");
    input=bound_sample;input.entities[1].standard_type33_root_owner=true;rejected(input,"contradictory_owner_profile");
    input=bound_sample;input.owner_references->at(0).standard_identity_input=false;
    check(project_native_dependency_load(input).resolved,"path-only transitions do not invoke the paired transform service");
    const auto paired=unhex("e70301000010010029000000000000002a00000000000000");
    input.entities[3].dependency_payloads={paired};rejected(input,"owner_reference_transform_requires_context");
    input=bound_sample;
    input.entities[3].dependency_payloads={unhex("10270400001801000300000000000000000000000000000029000000000000002b000000000000002a00000000000000")};
    auto nested=project_native_dependency_load(input);
    check(nested.resolved && nested.file_dependents[1][0]==std::vector<std::size_t>{3} && nested.dependents[2].empty(),
          "same source ID in model and reference contexts does not redirect a nested target to current model");
    input.owner_references->at(2).bound_model_id=7;
    nested=project_native_dependency_load(input);
    check(nested.resolved && nested.file_dependents[1][0].empty() && nested.dependents[2]==std::vector<std::size_t>{3},
          "changing the actual nested binding changes target identity even when the source ID is equal");
    input=bound_sample;input.owner_references->insert(input.owner_references->begin()+1,{{},42,10,true});
    input.owner_references->at(3).parent.reference_index=0;
    nested=project_native_dependency_load(input);
    check(nested.resolved && nested.file_dependents[0][0]==std::vector<std::size_t>{3} && nested.file_dependents[1][0].empty(),
          "first matching reference in the caller's ordered list wins");
    input=bound_sample;input.max_work_items=20;rejected(input,"work_limit_exceeded");
    input=bound_sample;input.owner_references->at(1).parent.model_id=99;rejected(input,"owner_bound_model_requires_context");
    input=bound_sample;input.file_context->complete=false;rejected(input,"file_model_registry_requires_context");
    input=bound_sample;input.owner_references->at(0).parent.model_id=7;
    check(project_native_dependency_load(input).resolved,"explicit current-model parent and implicit current parent share context identity");
    input=base;input.system_registry_known_empty=false;input.entities[1].dependency_payloads[0][8]=3;
    rejected(input,"system_registry_requires_context");
    input=base;input.file_fallback_disabled=false;input.entities[1].dependency_payloads[0][8]=3;
    rejected(input,"file_fallback_requires_context");
    input=base;input.system_registry_known_empty=input.file_fallback_disabled=false;
    auto local=project_native_dependency_load(input);
    check(local.resolved && local.dependents==std::vector<std::vector<std::size_t>>{{1},{}},
          "known local hit does not require unused system or file fallback context");
    input.entities[1].dependency_payloads.push_back(unhex("e7030100000001000300000000000000"));
    rejected(input,"system_registry_requires_context");
    input.entities[1].dependency_payloads.pop_back();
    input.batches={{1},{0}};
    rejected(input,"system_registry_requires_context");
    input.batches={{0,1}};
    check(project_native_dependency_load(input).resolved,"whole subtree registration precedes local dependency lookup");
    input.entities[1].dependency_payloads[0][4]=1;
    input.batches={{1},{0}};
    check(project_native_dependency_load(input).resolved,"disabled dependency never requires lookup fallback");
    input.entities[1].dependency_payloads.clear();
    check(project_native_dependency_load(input).resolved,"no dependency payload needs no fallback context");
    input=base;input.monitored_entity_set_known_empty=false;rejected(input,"monitored_entities_require_context");
    auto system_input=base;
    system_input.system_registry_known_empty=system_input.file_fallback_disabled=false;
    system_input.system_registry=std::vector<NativeDependencySystemTarget>{{42,0}};
    system_input.entities[1].dependency_payloads[0][8]=42;
    auto system_result=project_native_dependency_load(system_input);
    check(system_result.resolved && system_result.system_dependents==std::vector<std::vector<std::size_t>>{{1}} &&
          system_result.pending_entities.empty(),"system hit requires no unused file fallback context");
    system_input.entities[1].dependency_payloads.push_back(unhex("e7030100000001000300000000000000"));
    rejected(system_input,"file_fallback_requires_context");
    system_input.entities[1].dependency_payloads.pop_back();
    input=system_input;input.system_registry->at(0).runtime_flags_10.reset();
    rejected(input,"system_target_runtime_flags_require_context");
    input.entities[1].dependency_payloads[0][8]=1;
    check(project_native_dependency_load(input).resolved,"unused system target flags are not required for a local hit");
    input=system_input;input.system_registry->push_back({42,0});rejected(input,"system_assigned_id_collision");
    input=system_input;input.system_registry_known_empty=true;rejected(input,"contradictory_system_registry");
    input=system_input;input.system_registry->clear();rejected(input,"file_fallback_requires_context");
    input.file_fallback_disabled=true;
    check(project_native_dependency_load(input).pending_entities==std::vector<std::size_t>{1},"complete empty system registry proves a genuine miss");
    input=system_input;input.system_registry->at(0).assigned_id=0;input.entities[1].dependency_payloads[0][8]=0;
    input.file_fallback_disabled=true;
    system_result=project_native_dependency_load(input);
    check(system_result.resolved && system_result.system_dependents==std::vector<std::vector<std::size_t>>{{}} && system_result.pending_entities.empty(),
          "zero ID is not a registered system target and does not create pending work");
    input=system_input;input.max_work_items=1;rejected(input,"work_limit_exceeded");
    auto model_input=base;
    model_input.entities[1].dependency_payloads={unhex("e70301000020010009000000000000002a00000000000000")};
    rejected(model_input,"file_model_registry_requires_context");
    model_input.file_context.emplace();auto &model_context=*model_input.file_context;
    model_context.current_model_id=7;model_context.current_file_fallback_enabled=false;
    model_context.models={{9,{{42,0}},false}};
    rejected(model_input,"file_model_registry_requires_context");
    model_context.complete=true;
    auto model_result=project_native_dependency_load(model_input);
    check(model_result.resolved && model_result.file_dependents==std::vector<std::vector<std::vector<std::size_t>>>{{{1}}},
          "format 8 uses the supplied resident model and keeps its identity");
    input=model_input;input.file_context->models[0].entities[0].runtime_flags_10.reset();
    rejected(input,"file_target_runtime_flags_require_context");
    input=model_input;input.file_context->models[0].model_id=-1;rejected(input,"file_model_id_collision");
    input=model_input;input.file_context->models[0].model_id=7;rejected(input,"file_model_id_collision");
    input=model_input;input.file_context->models.push_back(input.file_context->models[0]);rejected(input,"file_model_id_collision");
    input=model_input;input.file_context->models[0].entities.push_back({42,0});rejected(input,"file_assigned_id_collision");
    input=model_input;input.file_context->current_model_id=-1;rejected(input,"current_model_must_be_ordinary");
    input=model_input;input.file_context->current_file_fallback_enabled=true;rejected(input,"contradictory_file_fallback");
    input=model_input;input.file_context->models[0].file_fallback_enabled.reset();
    input.entities[1].dependency_payloads.push_back(unhex("e70301000020010009000000000000002b00000000000000"));
    rejected(input,"file_fallback_requires_context");
    input=model_input;input.system_registry_known_empty=false;input.entities[1].dependency_payloads[0][8]=123;
    model_result=project_native_dependency_load(input);
    check(model_result.resolved && model_result.pending_entities==std::vector<std::size_t>{1} && model_result.file_dependents[0][0].empty(),
          "missing model never consults an unknown system registry or falls back to another resident model");
    input=model_input;input.system_registry_known_empty=false;
    input.entities[1].dependency_payloads[0][16]=43;rejected(input,"system_registry_requires_context");
    input=model_input;
    for(unsigned i=8;i<12;++i)input.entities[1].dependency_payloads[0][i]=255;
    rejected(input,"file_fallback_requires_context");
    input.file_context->system_file_fallback_enabled=true;
    model_result=project_native_dependency_load(input);
    check(model_result.resolved && model_result.file_dependents[0][0]==std::vector<std::size_t>{1},
          "explicit system model falls through to the complete resident-model registry");
    input.file_context.reset();input.system_registry_known_empty=false;
    input.system_registry=std::vector<NativeDependencySystemTarget>{{42,0}};
    check(project_native_dependency_load(input).resolved,"system hit in format 8 needs no unrelated resident model registry");
    input=model_input;input.max_work_items=2;
    input.file_context->models[0].entities={{42,0},{43,0}};rejected(input,"work_limit_exceeded");
    auto selector_input=base;
    selector_input.entities[1].dependency_payloads={unhex("e70301000010010001000000000000000000000000000000")};
    selector_input.file_fallback_disabled=false;
    check(project_native_dependency_load(selector_input).resolved,"zero-owner selector adds a system-only owner lookup without requiring file fallback");
    selector_input.entities[1].dependency_payloads[0][16]=1;
    rejected(selector_input,"dependency_owner_path_requires_context");
    selector_input.entities[0].standard_type33_root_owner=true;
    auto selector_result=project_native_dependency_load(selector_input);
    check(selector_result.resolved && selector_result.dependents[0]==std::vector<std::size_t>{1,1},
          "paired target and additional owner reference remain two independent reverse nodes");
    input=selector_input;input.entities[0].runtime_flags_10.reset();rejected(input,"owner_runtime_flags_require_context");
    input=selector_input;input.entities[0].runtime_flags_10=8;rejected(input,"owner_lookup_mode_requires_context");
    input.owner_lookup_includes_deleted=false;
    selector_result=project_native_dependency_load(input);
    check(selector_result.resolved && selector_result.pending_entities==std::vector<std::size_t>{1} && selector_result.dependents[0].empty(),
          "deleted owner rejection queues the paired target while the excluded owner edge is skipped");
    input.owner_lookup_includes_deleted=true;
    selector_result=project_native_dependency_load(input);
    check(selector_result.resolved && selector_result.pending_entities.empty() && selector_result.dependents[0].empty(),
          "permitted deleted owner can resolve a target that mode 1 subsequently filters");
    input=selector_input;input.entities[0].standard_type33_root_owner=false;input.entities[1].dependency_payloads[0][16]=99;
    selector_result=project_native_dependency_load(input);
    check(selector_result.resolved && selector_result.pending_entities==std::vector<std::size_t>{1},
          "a genuinely missing owner requires neither a path profile nor file fallback");
    input=selector_input;input.entities[1].dependency_payloads.push_back(unhex("e70301000010010001000000000000000200000000000000"));
    rejected(input,"dependency_owner_path_requires_context");
    input=selector_input;input.entities[1].dependency_payloads[0][5]=28;
    input.entities[1].dependency_payloads[0].resize(32);input.entities[1].dependency_payloads[0][16]=4;
    rejected(input,"compact_dependency_selector_requires_context");
    input=selector_input;input.entities[0].assigned_id=UINT64_MAX;
    for(unsigned i=8;i<16;++i)input.entities[1].dependency_payloads[0][i]=255;
    input.entities[1].dependency_payloads[0][16]=0;
    selector_result=project_native_dependency_load(input);
    check(selector_result.resolved && selector_result.pending_entities==std::vector<std::size_t>{1} && selector_result.dependents[0].empty(),
          "paired maximum ID suppresses target lookup but still produces native nonzero pending work");
    input=selector_input;input.max_work_items=6;rejected(input,"work_limit_exceeded");
    input.max_work_items=7;
    check(project_native_dependency_load(input).resolved,"work budget counts both emitted selector references");
    auto path_payload=[](std::initializer_list<std::uint64_t> ids) {
        Bytes raw(24+ids.size()*8);raw[0]=0x10;raw[1]=0x27;raw[2]=4;raw[5]=24;raw[6]=1;
        raw[8]=std::uint8_t(ids.size());std::size_t offset=24;
        for(auto id:ids)for(unsigned i=0;i<8;++i)raw[offset++]=std::uint8_t(id>>(i*8));
        return raw;
    };
    auto path_input=base;path_input.file_fallback_disabled=false;
    path_input.entities[1].dependency_payloads={path_payload({1})};
    auto path_result=project_native_dependency_load(path_input);
    check(path_result.resolved && path_result.dependents[0]==std::vector<std::size_t>{1,1},
          "single-ID path does not traverse an owner and preserves two original nodes");
    path_input.entities[1].dependency_payloads={path_payload({1,1})};
    rejected(path_input,"dependency_owner_path_requires_context");
    path_input.entities[0].standard_type33_root_owner=true;
    rejected(path_input,"model_owner_transition_requires_context");
    path_input.standard_model_owner_transition_known_null=true;
    path_result=project_native_dependency_load(path_input);
    check(path_result.resolved && path_result.dependents[0]==std::vector<std::size_t>{1} && path_result.pending_entities==std::vector<std::size_t>{1},
          "ordinary model null transition keeps the outer owner edge and queues the failed path");
    input=path_input;input.entities[0].runtime_flags_10.reset();rejected(input,"owner_runtime_flags_require_context");
    input=path_input;input.entities[0].runtime_flags_10=8;rejected(input,"owner_lookup_mode_requires_context");
    input.standard_model_owner_transition_known_null=false;input.owner_lookup_includes_deleted=false;
    path_result=project_native_dependency_load(input);
    check(path_result.resolved && path_result.dependents[0].empty() && path_result.pending_entities==std::vector<std::size_t>{1},
          "rejected deleted path owner never requires the model transition");
    input.owner_lookup_includes_deleted=true;rejected(input,"model_owner_transition_requires_context");
    input=base;input.file_fallback_disabled=false;input.entities[1].dependency_payloads={path_payload({1,99})};
    path_result=project_native_dependency_load(input);
    check(path_result.resolved && path_result.pending_entities==std::vector<std::size_t>{1},
          "missing path owner stops without unknown profile or file fallback");
    input.entities[1].dependency_payloads={path_payload({})};
    path_result=project_native_dependency_load(input);
    check(path_result.resolved && path_result.pending_entities==std::vector<std::size_t>{1},
          "active empty path returns two MAX/null slots without context-free reader underflow");
    input.entities[1].dependency_payloads={path_payload({0})};
    path_result=project_native_dependency_load(input);
    check(path_result.resolved && path_result.pending_entities.empty(),"missing zero single-path IDs do not queue");
    input=path_input;input.entities[1].dependency_payloads[0].pop_back();rejected(input,"truncated_dependency_owner_path");
    input=path_input;input.max_work_items=9;rejected(input,"work_limit_exceeded");
    input.max_work_items=10;
    check(project_native_dependency_load(input).resolved,"path work budget includes source IDs, slots and attempted transition");
    input=base;input.entities[1].dependency_payloads.push_back(path_payload({1,1}));
    rejected(input,"dependency_owner_path_requires_context");
    input=base;input.entities[0].runtime_flags_10.reset();rejected(input,"target_runtime_flags_require_context");
    input=base;input.entities[1].assigned_id=1;rejected(input,"assigned_id_collision");
    input=base;input.batches={{0},{0,1}};rejected(input,"entity_registered_more_than_once");
    input=base;input.batches={{0},{2}};rejected(input,"entity_index_out_of_range");
    input=base;input.batches={{0}};rejected(input,"entity_not_registered");
    for(std::size_t length=0;length<16;++length) {
        input=base;input.entities[1].dependency_payloads[0].resize(length);
        rejected(input,length<8?"truncated_dependency_header":"truncated_dependency_entries");
    }
    input=base;input.entities[1].dependency_payloads[0]=unhex("10270400000001000100000000000000");
    rejected(input,"dependency_owner_path_requires_context");
    for(unsigned format=2;format<=8;++format) {
        input=base;input.entities[1].dependency_payloads[0][5]=std::uint8_t(format<<2);
        rejected(input,format==6?"truncated_dependency_owner_path":"truncated_dependency_entries");
        input.entities[1].dependency_payloads[0][4]=1;input.entities[1].dependency_payloads[0].resize(8);
        check(project_native_dependency_load(input).resolved,"disabled unsupported formats do not access entries");
    }
    for(unsigned format=9;format<16;++format) {
        input=base;input.entities[1].dependency_payloads[0][5]=std::uint8_t(format<<2);
        input.entities[1].dependency_payloads[0].resize(8);
        check(project_native_dependency_load(input).resolved,"out-of-dispatch format skips payload references");
    }
    for(std::size_t budget=0;budget<6;++budget) {
        input=base;input.max_work_items=budget;rejected(input,"work_limit_exceeded");
    }
    input=base;input.max_work_items=6;check(project_native_dependency_load(input).resolved,"exact work budget is accepted");
    input=base;input.entities.clear();input.batches.clear();input.max_work_items=0;
    check(project_native_dependency_load(input).resolved,"known empty initial input resolves without work");
    NativeDependencyRetryInput retry;
    retry.entities=base.entities;retry.dependents.resize(2);retry.pending_iteration_order={1};retry.model_notified=true;
    retry.input_complete=retry.system_registry_known_empty=retry.file_fallback_disabled=retry.monitored_entity_set_known_empty=true;
    auto reject_retry=[&](const NativeDependencyRetryInput &v,const char *reason){
        const auto r=project_native_dependency_retry(v);
        check(!r.resolved && r.reason==reason && r.dependents.empty() && r.pending_entities.empty() &&
              r.monitored_entities.empty() && r.added_edges.empty() && r.scheduled_pairs.empty(),"failed retry never exposes partial state");
    };
    auto v=retry;v.input_complete=false;reject_retry(v,"incomplete_input");
    v=retry;v.system_registry_known_empty=false;reject_retry(v,"system_registry_requires_context");
    v=retry;v.file_fallback_disabled=false;reject_retry(v,"file_fallback_requires_context");
    v=retry;v.monitored_entity_set_known_empty=false;reject_retry(v,"monitored_entities_require_context");
    v=retry;v.model_notified.reset();reject_retry(v,"model_notification_requires_context");
    v=retry;v.dependents.clear();reject_retry(v,"reverse_list_count_mismatch");
    v=retry;v.dependents[0]={2};reject_retry(v,"dependent_index_out_of_range");
    v=retry;v.entities[1].assigned_id=1;reject_retry(v,"assigned_id_collision");
    v=retry;v.pending_iteration_order={2};reject_retry(v,"entity_index_out_of_range");
    v=retry;v.pending_iteration_order={1,1};reject_retry(v,"pending_entity_repeated");
    v=retry;v.entities[1].runtime_flags_10.reset();reject_retry(v,"pending_runtime_flags_require_context");
    v=retry;v.entities[0].runtime_flags_10.reset();reject_retry(v,"target_runtime_flags_require_context");
    v=retry;v.entities[1].runtime_flags_10=0x100000;reject_retry(v,"attribute_dependencies_require_context");
    v.entities[1].runtime_flags_10=0x100008;check(project_native_dependency_retry(v).resolved,"excluded pending entities bypass attribute traversal");
    for(std::size_t length=0;length<16;++length) {
        v=retry;v.entities[1].dependency_payloads[0].resize(length);
        reject_retry(v,length<8?"truncated_dependency_header":"truncated_dependency_entries");
    }
    for(std::size_t budget=0;budget<5;++budget) {v=retry;v.max_work_items=budget;reject_retry(v,"work_limit_exceeded");}
    v=retry;v.max_work_items=5;check(project_native_dependency_retry(v).resolved,"exact retry work budget is accepted");
    v=retry;v.entities.clear();v.dependents.clear();v.pending_iteration_order.clear();v.max_work_items=0;
    check(project_native_dependency_retry(v).resolved,"empty retry consumes no entity work");
    NativeDependencyNormalizationInput normal;
    normal.input_complete=normal.standard_entities_known=normal.removal_work_known_empty=true;
    normal.dependents={{1,2,1,2,0,1},{0,0},{}};normal.scheduled_pairs={{0,1},{0,1},{2,2}};
    auto normalized=project_native_dependency_normalization(normal);
    check(normalized.resolved && normalized.dependents==std::vector<std::vector<std::size_t>>{{1,2,2,0},{0,0},{}},
          "only scheduled pairs are normalized; unrelated duplicates and absent pairs are preserved");
    check(normalized.retained_positions==std::vector<std::vector<std::size_t>>{{0,1,3,4},{0,1},{}},"normalization keeps the first scheduled occurrence");
    auto reject_normal=[&](const NativeDependencyNormalizationInput &n,const char *reason) {
        auto r=project_native_dependency_normalization(n);
        check(!r.resolved && r.reason==reason && r.dependents.empty() && r.retained_positions.empty(),"normalization failure publishes no partial lists");
    };
    auto n=normal;n.input_complete=false;reject_normal(n,"incomplete_input");
    n=normal;n.standard_entities_known=false;reject_normal(n,"target_entity_interface_requires_context");
    n=normal;n.removal_work_known_empty=false;reject_normal(n,"dependency_removal_requires_context");
    n=normal;n.scheduled_pairs={{3,0}};reject_normal(n,"scheduled_entity_index_out_of_range");
    n=normal;n.scheduled_pairs={{0,3}};reject_normal(n,"scheduled_entity_index_out_of_range");
    n=normal;n.dependents[1].push_back(3);reject_normal(n,"dependent_index_out_of_range");
    for(std::size_t budget=0;budget<14;++budget) {n=normal;n.max_work_items=budget;reject_normal(n,"work_limit_exceeded");}
    n=normal;n.max_work_items=14;check(project_native_dependency_normalization(n).resolved,"exact normalization work budget accepted");
    n=normal;n.dependents.clear();n.scheduled_pairs.clear();n.max_work_items=0;
    check(project_native_dependency_normalization(n).resolved,"empty normalization consumes no work");
    NativeDependencyCycleInput complete;
    static_cast<NativeDependencyRetryInput &>(complete)=retry;
    complete.other_work_queues_known_empty=complete.standard_entities_known=
        complete.caller_holds_model=complete.link_update_handlers_known_absent=true;
    complete.entities[1].dependency_payloads={unhex("e7030100000002000100000000000000e703000000000000")};
    auto completed=project_native_dependency_cycle(complete);
    check(completed.resolved && completed.dependents==std::vector<std::vector<std::size_t>>{{1},{}} &&
          completed.pending_entities.empty() && completed.monitored_entities==std::vector<std::size_t>{1},
          "missing ID remains observable after successful complete iteration");
    auto reject_cycle=[&](const NativeDependencyCycleInput &c,const char *reason) {
        const auto r=project_native_dependency_cycle(c);
        check(!r.resolved && r.reason==reason && r.dependents.empty() && r.pending_entities.empty() &&
              r.monitored_entities.empty() && r.scheduled_pairs.empty(),"failed complete iteration publishes no partial graph or queues");
    };
    auto c=complete;c.other_work_queues_known_empty=false;reject_cycle(c,"dependency_work_queues_require_context");
    c=complete;c.standard_entities_known=false;reject_cycle(c,"target_entity_interface_requires_context");
    c=complete;c.caller_holds_model=false;reject_cycle(c,"model_release_requires_context");
    c=complete;c.link_update_handlers_known_absent=false;reject_cycle(c,"link_update_handlers_require_context");
    c=complete;c.input_complete=false;reject_cycle(c,"incomplete_input");
    c=complete;c.entities[1].dependency_payloads[0][5]=0x40;reject_cycle(c,"required_link_handler_requires_context");
    // The disabled second linkage does not participate in retry, but the full
    // update callback still dispatches it when the first link remains missing.
    c=complete;c.entities[1].dependency_payloads.push_back(unhex("e703010001000100"));
    reject_cycle(c,"truncated_dependency_entries");
    c.entities[1].dependency_payloads.back()[5]=8;reject_cycle(c,"dependency_format_requires_context");
    c=complete;c.entities[1].dependency_payloads.push_back(unhex("10270400010001000100000000000000"));
    reject_cycle(c,"dependency_owner_path_requires_context");
    for(std::size_t budget=0;budget<14;++budget) {c=complete;c.max_work_items=budget;reject_cycle(c,"work_limit_exceeded");}
    c=complete;c.max_work_items=14;
    check(project_native_dependency_cycle(c).resolved,"complete iteration accepts the exact shared phase budget");
    c=complete;c.entities.clear();c.dependents.clear();c.pending_iteration_order.clear();c.max_work_items=0;
    check(project_native_dependency_cycle(c).resolved,"known empty complete iteration needs no entity work");
    return checks;
}
