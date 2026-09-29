#include "internal.hpp"
#include <p3d/model_bounds.hpp>
#include <p3d/default_view_table.hpp>
#include "entity_registration_oracle.hpp"
#include "entity_input_ids_oracle.hpp"
#include "entity_file_input_oracle.hpp"

using namespace p3d;
unsigned entity_registration_tests() {
    unsigned checks=0;
    auto check=[&](bool value,const char *message){++checks;require(value,message);};
    auto put=[](Bytes &bytes,std::size_t at,auto value){std::memcpy(bytes.data()+at,&value,sizeof(value));};
    auto from_hex=[](const std::string &text) {
        Bytes result;
        for(std::size_t i=0;i<text.size();i+=2)
            result.push_back(static_cast<std::uint8_t>(std::stoul(text.substr(i,2),nullptr,16)));
        return result;
    };
    for (unsigned profile=0;profile<3;++profile) {
    const bool prepare_ids=profile==1,file_entry=profile==2;
    const auto oracle=Json::parse(file_entry?entity_file_input_oracle:prepare_ids?entity_input_ids_oracle:entity_registration_oracle);
    check(oracle.at("cases").size()==(file_entry?352u:prepare_ids?160u:128u),"all entity forest observations are tested");
    for (const auto &row:oracle.at("cases")) {
        Json list={{"status","resolved"},{"system_bootstrap_required",true},{"roots",Json::array()}};
        Json records=Json::array();
        const auto parents=row.at("parents").get<std::vector<int>>();
        const auto roots=row.at("root_indices").get<std::vector<std::size_t>>();
        std::vector<std::size_t> list_roots(parents.size());
        std::vector<std::uint32_t> descendants(parents.size());
        std::vector<Bytes> projected_headers(parents.size());
        for(std::size_t i=parents.size();i-->0;)
            if(parents[i]>=0)descendants.at(std::size_t(parents[i]))+=1+descendants[i];
        for (std::size_t i=0;i<parents.size();++i) {
            list_roots[i]=parents[i]<0?i:list_roots.at(std::size_t(parents[i]));
            records.push_back({{"id",row.at("ids")[i]}});
        }
        for (auto root:roots) {
            Json headers=Json::array();
            for (std::size_t i=0;i<parents.size();++i)
                if (list_roots[i]==root)
                    headers.push_back({{"status","resolved"},{"native_record_index",i},
                        {"parent_record_index",parents[i]<0?Json():Json(parents[i])}});
            list["roots"].push_back({{"native_record_index",root},{"block_number",1},{"headers",headers}});
        }
        // The older oracle starts after identity ID preparation. The file-
        // service oracle executes the original recursive preparation, including
        // zeros, a counter below source IDs and wraparound, before each root.
        Json file={{"status","resolved"},{"initial_probe",{{"action","read_header_payload"},{"id_counter",row.at("initial_counter")}}}};
        const auto assigned=native_system_id_assignments(list,records,file);
        check(assigned.at("status")=="resolved" && assigned.at("final_id_counter")==row.at("counter"),
              "original native ID registry matches prepared initial input including uint64 wrap");
        std::vector<bool> seen(parents.size());
        std::size_t root_index=0;
        for (const auto &root:assigned.at("roots")) {
            const auto &call=row.at("calls")[root_index++];
            if (file_entry) {
                const auto &context=call.at("context");
                check(call.at("input_return")==0 && context.at("callback_depth")==0 &&
                      context.at("transaction_count")==root_index && context.at("transaction_status")==0,
                      "original file entry executes with balanced dependency callback and TLS transaction bookkeeping");
                check(context.at("registry_set_counts")==std::vector<unsigned>(18,0),
                      "no-linkage input leaves original dependency registries empty");
            }
            if (prepare_ids)
                check(root.at("counter_after_subtree_preparation")==call.at("prepared_counter"),
                      "complete original file-service preparation sets the same subtree counter");
            for (const auto &record:root.at("records")) {
                const auto index=record.at("native_record_index").get<std::size_t>();seen.at(index)=true;
                const auto &actual=row.at("entities")[index];
                if (prepare_ids)
                    check(record.at("prepared_id")==call.at("prepared_ids")[index],
                          "original recursive file-service preparation assigns the same ID before registration");
                check(record.at("assigned_id")==actual.at("id"),"native-created entity receives the same registered ID");
                check(record.at("parent_input_occurrence_index")==actual.at("parent"),"native entity parent pointer matches input occurrence identity");
                const auto ancestor=record.at("entity_list_root_input_occurrence_index").get<std::size_t>();
                const auto ordinal=actual.at("ordinal").get<std::size_t>();
                check(ancestor==roots.at(ordinal) && record.at("entity_is_top_level")==actual.at("parent").is_null(),
                      "children share root physical ordinal and do not become top-level list entries");
                std::vector<std::size_t> children;
                for (std::size_t i=0;i<parents.size();++i) if (parents[i]==int(index)) children.push_back(i);
                check(actual.at("children")==children,"native allocated child-vector order matches input siblings");
                if (file_entry) {
                    const auto source=from_hex(actual.at("source_header"));
                    Bytes loaded(4);loaded.insert(loaded.end(),source.begin(),source.end());
                    const Json header_record={{"data",rawbytes(loaded)},
                        {"element_type",Reader(loaded,4).u16()},{"element_flags",Reader(loaded,6).u16()}};
                    const auto prepared=native_list_record_header(header_record,Json(),parents[index]>=0,
                                                                 !children.empty(),descendants[index],false);
                    check(prepared.at("status")=="resolved","file entry header preparation is resolved");
                    put(loaded,6,prepared.at("output_element_flags").get<std::uint16_t>());
                    put(loaded,8,prepared.at("output_record_word_count").get<std::uint32_t>());
                    put(loaded,12,prepared.at("output_base_word_count").get<std::uint32_t>());
                    put(loaded,20,record.at("assigned_id").get<std::uint64_t>());
                    const auto &count=prepared.at("descendant_count_update");
                    if (!count.is_null() && count.at("written")==true)
                        put(loaded,4+count.at("header_offset").get<std::size_t>(),count.at("output_value").get<std::uint32_t>());
                    check(Bytes(loaded.begin()+4,loaded.end())==from_hex(actual.at("loaded_header")),
                          "library header and ID preparation reproduce every byte copied by the original file entry and callback");
                    projected_headers[index]=std::move(loaded);
                }
            }
        }
        check(std::all_of(seen.begin(),seen.end(),[](bool value){return value;}),"every native-created entity is checked");
        NativeModelBoundsProviderInput bounds;
        bounds.record_list_complete=true;bounds.model_query_68=row.at("spatial");
        bounds.record_indices=roots;
        for (std::size_t i=0;i<parents.size();++i) {
            NativeModelBoundsRecord record;record.record_base.resize(132);
            put(record.record_base,6,std::uint16_t(0x20|(parents[i]<0?0:0x80)));
            put(record.record_base,36,row.at("extended_flags").get<std::uint16_t>());
            for (unsigned axis=0;axis<6;++axis)
                put(record.record_base,60+8*axis,row.at("source_ranges")[i][axis].get<std::int64_t>());
            record.runtime_flags_10=row.at("entities")[i].at("flags");
            if(file_entry)record.record_base=projected_headers.at(i);
            bounds.records.push_back(record);
        }
        const auto provider=project_native_model_bounds_provider(bounds);
        NativeCachedModelBoundsInput cached;cached.range_pointer_known=provider.resolved;
        cached.integer_range=provider.integer_range;cached.spatial_byte_78=row.at("spatial").get<bool>()?1:0;
        const auto projected=project_native_cached_model_bounds(cached);
        check(projected.resolved && projected.return_code==row.at("bounds_result").get<int>() &&
                  projected.range.value_or(std::array<double,6>{11,22,33,44,55,66})==row.at("bounds").get<std::array<double,6>>(),
              "actual registered top-level entities feed the same full native model range; descendants are not added again");
    }
    }
    return checks;
}
