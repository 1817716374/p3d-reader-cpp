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
    for(const auto &row:model_cases) {
        NativeDependencyLoadInput input;
        input.input_complete=input.file_fallback_disabled=input.monitored_entity_set_known_empty=true;
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
    NativeDependencyLoadInput base;
    base.input_complete=base.system_registry_known_empty=base.file_fallback_disabled=base.monitored_entity_set_known_empty=true;
    base.entities={{1,0,{}},{2,0,{unhex("e7030100000001000100000000000000")}}};base.batches={{0},{1}};
    auto rejected=[&](const NativeDependencyLoadInput &input,const char *reason){
        auto result=project_native_dependency_load(input);
        check(!result.resolved && result.reason==reason && result.dependents.empty() && result.pending_entities.empty() && result.batches.empty() && result.system_dependents.empty() && result.file_dependents.empty(),
              "unknown or invalid dependency input never publishes a partial graph");
    };
    auto input=base;input.input_complete=false;rejected(input,"incomplete_input");
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
        rejected(input,format==6?"dependency_owner_path_requires_context":"truncated_dependency_entries");
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
