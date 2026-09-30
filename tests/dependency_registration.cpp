#include "internal.hpp"
#include <p3d/dependency_registration.hpp>
#include "dependency_registration_oracle.hpp"
#include "dependency_retry_oracle.hpp"
#include "dependency_cycle_oracle.hpp"

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
            check(after.at("context").at("pending_entities")==retried.pending_entities,"retry requeues missing or excluded targets");
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
    NativeDependencyLoadInput base;
    base.input_complete=base.system_registry_known_empty=base.file_fallback_disabled=base.monitored_entity_set_known_empty=true;
    base.entities={{1,0,{}},{2,0,{unhex("e7030100000001000100000000000000")}}};base.batches={{0},{1}};
    auto rejected=[&](const NativeDependencyLoadInput &input,const char *reason){
        auto result=project_native_dependency_load(input);
        check(!result.resolved && result.reason==reason && result.dependents.empty() && result.pending_entities.empty() && result.batches.empty(),
              "unknown or invalid dependency input never publishes a partial graph");
    };
    auto input=base;input.input_complete=false;rejected(input,"incomplete_input");
    input=base;input.system_registry_known_empty=false;rejected(input,"system_registry_requires_context");
    input=base;input.file_fallback_disabled=false;rejected(input,"file_fallback_requires_context");
    input=base;input.monitored_entity_set_known_empty=false;rejected(input,"monitored_entities_require_context");
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
        rejected(input,"dependency_format_requires_context");
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
    return checks;
}
