#include "internal.hpp"
#include <p3d/dependency_registration.hpp>
#include "dependency_registration_oracle.hpp"

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
    std::size_t case_index=0;
    for(const auto &row:oracle.at("cases")) {
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
        for(std::size_t i=0;i<parents.size();++i)
            check(row.at("entities")[i].at("dependents")==result.dependents[i],"duplicate and self-reference nodes match original allocation");
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
    return checks;
}
