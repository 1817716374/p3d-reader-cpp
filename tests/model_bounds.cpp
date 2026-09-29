#include "internal.hpp"
#include <p3d/model_bounds.hpp>
#include <p3d/default_view_table.hpp>
#include "model_bounds_provider_oracle.hpp"

using namespace p3d;
unsigned model_bounds_tests() {
    unsigned checks=0;
    auto check=[&](bool value,const char *message) { ++checks; require(value,message); };
    auto put=[](Bytes &bytes,std::size_t offset,auto value) { std::memcpy(bytes.data()+offset,&value,sizeof(value)); };
    const auto oracle=Json::parse(model_bounds_provider_oracle);
    check(oracle.at("cases").size()==1314,"all full lazy-provider observations are tested");
    unsigned split_cases=0;
    for (const auto &row:oracle.at("cases")) {
        NativeModelBoundsProviderInput input;
        input.record_list_complete=true; input.model_query_68=row.at("model_query_68");
        for (const auto &block:row.at("blocks"))
            for (const auto &index:block) input.record_indices.push_back(index.get<std::size_t>());
        for (const auto &item:row.at("records")) {
            NativeModelBoundsRecord record;record.record_base.resize(132);
            auto &bytes=record.record_base;
            put(bytes,4,std::uint16_t{33}); put(bytes,6,item.at("element_flags").get<std::uint16_t>());
            put(bytes,8,std::uint32_t{64});put(bytes,12,std::uint32_t{64});put(bytes,20,std::uint64_t{1});
            put(bytes,36,item.at("extended_flags").get<std::uint16_t>());
            for (unsigned i=0;i<7;++i) put(bytes,60+8*i,item.at("words")[i].get<std::int64_t>());
            record.runtime_flags_10=item.at("runtime_flags");record.override_lookup_known=true;
            if (item.at("key_mode")=="exact" || item.at("key_mode")=="sandwich")
                record.override_range=item.at("override").get<std::array<std::int64_t,6>>();
            input.records.push_back(record);
        }
        const auto out=project_native_model_bounds_provider(input);
        check(out.resolved && out.integer_range==row.at("integer_range").get<std::array<std::int64_t,6>>(),
              "full native initial cache creation matches record selection and integer union");
        NativeCachedModelBoundsInput cached;
        cached.range_pointer_known=true;cached.integer_range=out.integer_range;
        cached.spatial_byte_78=row.at("spatial_byte_78");
        const auto projected=project_native_cached_model_bounds(cached);
        check(projected.resolved && projected.return_code==row.at("return_code").get<int>(),
              "independent provider dimension and final model dimension match full getter");
        check(projected.range.value_or(std::array<double,6>{11,22,33,44,55,66})==
                  row.at("double_range").get<std::array<double,6>>(),
              "actual provider output converts to native doubles or preserves destination on failure");
        split_cases+=row.at("root_kind")==0;
    }
    check(split_cases==24,"oracle exercises native tree splitting with 33,65,129,257 records in each dimension combination");
    NativeModelBoundsProviderInput input;
    check(!project_native_model_bounds_provider(input).resolved,"unknown list is not empty");
    input.record_list_complete=true;
    check(!project_native_model_bounds_provider(input).resolved,"empty provider still reads model dimension query");
    input.model_query_68=false;
    check(project_native_model_bounds_provider(input).resolved,"complete empty model produces original integer sentinel");
    input.record_indices={0};
    check(project_native_model_bounds_provider(input).reason=="model_bounds_record_index_out_of_range","invalid identity is diagnosed");
    input.records.resize(1);
    check(!project_native_model_bounds_provider(input).resolved,"runtime flags cannot be inferred from source flags");
    auto &record=input.records[0];record.runtime_flags_10=8;
    check(project_native_model_bounds_provider(input).resolved,"excluded runtime record requires no header bytes");
    record.runtime_flags_10=0;
    check(!project_native_model_bounds_provider(input).resolved,"included runtime record needs record flags");
    record.record_base.resize(8);
    check(project_native_model_bounds_provider(input).resolved,"nonextended header skips further reads");
    put(record.record_base,6,std::uint16_t{0x20});
    check(!project_native_model_bounds_provider(input).resolved,"extended flags must fit");
    record.record_base.resize(38);put(record.record_base,36,std::uint16_t{0x400});
    check(project_native_model_bounds_provider(input).resolved,"header-excluded record needs no range or override");
    put(record.record_base,36,std::uint16_t{0});
    check(!project_native_model_bounds_provider(input).resolved,"ordinary range cannot read beyond base bytes");
    record.record_base.resize(108);record.runtime_flags_10=0x400000;
    check(project_native_model_bounds_provider(input).reason=="model_bounds_override_lookup_unknown","unknown override is not a null lookup");
    record.override_lookup_known=true;
    check(project_native_model_bounds_provider(input).reason=="model_bounds_shifted_range_truncated","special null override shifts source by eight bytes");
    record.override_range=std::array<std::int64_t,6>{-1,-2,-3,4,5,6};
    record.record_base.resize(38);
    check(project_native_model_bounds_provider(input).integer_range==record.override_range,"present override bypasses missing inline range bytes");
    input.record_indices={0,0};
    check(project_native_model_bounds_provider(input).included_record_indices==std::vector<std::size_t>{0,0},"repeated record identity stays ordered");
    input.max_record_visits=1;
    check(project_native_model_bounds_provider(input).reason=="model_bounds_record_visit_limit","bounded traversal counts repeated visits");
    input.max_record_visits=10;input.record_indices={0,1};
    const auto partial=project_native_model_bounds_provider(input);
    check(!partial.resolved && !partial.integer_range && partial.included_record_indices.empty() && partial.failed_record==1,
          "failure never publishes partial bounds or accepted record list");
    Bytes raw(132);put(raw,4,std::uint16_t{127});put(raw,6,std::uint16_t{0x20});
    put(raw,8,std::uint32_t{64});put(raw,12,std::uint32_t{64});
    const std::array<std::int64_t,6> persisted{-11,-22,33,44,55,-66};
    for (unsigned i=0;i<6;++i) put(raw,60+8*i,persisted[i]);
    const auto header=decode_native_record_bounds_header(raw);
    check(header.at("integer_range")==persisted && header.at("valid_xy")==true && header.at("valid_xyz")==false &&
          header.at("runtime_inclusion")=="not_evaluated","persisted range preserves invalid Z and does not assert runtime inclusion");
    const auto records=parse_native(raw);
    check(records.at(0).at("bounds_header")==header && bytesof(records.at(0).at("data"))==raw,
          "normal record parsing exposes source bounds and preserves all original bytes");
    for (std::size_t size=0;size<108;++size)
        check(decode_native_record_bounds_header(slice(raw,0,size)).at("status")=="invalid",
              "truncated extended source header never consumes following linkages");
    put(raw,6,std::uint16_t{0});
    check(decode_native_record_bounds_header(raw).at("status")=="not_present","nonextended record payload is not misidentified as a range");
    return checks;
}
