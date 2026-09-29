#include "internal.hpp"
#include <p3d/default_view_table.hpp>
#include "default_view_table_oracle.hpp"
using namespace p3d;
unsigned default_view_table_tests() {
    unsigned checks=0;
    auto check=[&](bool value,const char *message){++checks;require(value,message);};
    auto near=[](double a,double b){return std::abs(a-b)<=2e-12*(1+std::abs(a)+std::abs(b));};
    const auto oracle=Json::parse(default_view_table_oracle);
    for (const auto &row:oracle.at("cases")) {
        NativeDefaultViewTableInput input;
        unsigned mask=row.at("existing_mask");
        for (unsigned i=0;i<8;++i) {
            input.slots[i]=(mask&(1u<<i))?NativeViewSlotPresence::Present:NativeViewSlotPresence::Absent;
            input.layouts[i]=input.slots[i];
        }
        input.model_present=row.at("model_present"); input.model_query_68=row.at("model_query_68");
        input.model_record_value_1c=1; input.suppress_selection=row.at("suppress_selection");
        input.directory.owning_file_present=row.at("special_directory");
        input.directory.model_id=7; input.directory.entries_complete=true;
        if (row.at("special_directory").get<bool>()) input.directory.entries={{7,0,1,0,1}};
        NativeCachedModelBoundsInput bounds;
        bounds.range_pointer_known=true; bounds.spatial_byte_78=row.at("spatial_byte_78").get<bool>()?1:0;
        if (!row.at("integer_bounds").is_null()) bounds.integer_range=row.at("integer_bounds").get<std::array<std::int64_t,6>>();
        const auto extracted=project_native_cached_model_bounds(bounds);
        check(extracted.resolved && extracted.return_code==row.at("bounds_result").get<int>(),
              "cached integer range validity agrees with full original model bounds getter");
        const double maximum=std::numeric_limits<double>::max();
        input.combined_model_range=extracted.range.value_or(std::array<double,6>{maximum,maximum,maximum,-maximum,-maximum,-maximum});
        check(*input.combined_model_range==row.at("combined_range").get<std::array<double,6>>(),
              "bounds conversion or unchanged empty initializer feeds the same no-child-model range to table fill");
        const auto &scale=row.at("fallback_scale");
        input.fallback_scale.record_present=!scale.is_null();
        if (!scale.is_null()) {
            auto &c=input.fallback_scale;
            c.value_28=scale.at("value_28");c.code_30=scale.at("code_30");c.code_68=scale.at("code_68");
            c.value_38=scale.at("value_38");c.value_40=scale.at("value_40");
            c.value_70=scale.at("value_70");c.value_78=scale.at("value_78");
        }
        const auto out=project_native_default_view_table(input);
        check(out.resolved && out.reason.empty(),"full default table projection resolves native observed input");
        check(out.preserved_slot_mask==mask && out.created_slot_mask==(*input.model_present?(255^mask):0) &&
              out.copied_slot_mask==(*input.model_present?0:(255^mask)),"native table keep/create/copy masks match");
        for (unsigned i=0;i<8;++i) {
            const auto &actual=row.at("slots")[i];const auto &slot=out.slots[i];
            if (mask&(1u<<i)) {
                check(slot.action==NativeDefaultTableSlotAction::Keep && !slot.created_view && !slot.copy_source_slot,
                      "existing native view and layout remain untouched");
            } else if (!*input.model_present) {
                unsigned first=0;while (!(mask&(1u<<first)))++first;
                check(slot.action==NativeDefaultTableSlotAction::Copy && slot.copy_source_slot==first && !slot.created_view,
                      "absent model copies lowest original view/layout pair");
                auto expected=row.at("existing").at(std::to_string(first));
                expected["flags"][0]=expected.at("flags")[0].get<std::uint32_t>() & ~slot.copy_clear_flags_10;
                expected["slot_index"]=i;
                check(expected==actual,"native copied scalar frame and layout match source with flag clear and destination slot");
            } else {
                check(slot.action==NativeDefaultTableSlotAction::Create && slot.created_view && !slot.copy_source_slot,
                      "present model constructs each missing slot");
                const auto &view=*slot.created_view;const auto &frame=view.frame;
                bool equal=view.flags==actual.at("flags").get<std::array<std::uint32_t,3>>() &&
                           view.values_a8_to_c0==actual.at("values_a8_to_c0").get<std::array<double,4>>() &&
                           view.state_188==actual.at("state_188") && frame.slot_index==actual.at("slot_index") &&
                           near(frame.half_depth,actual.at("half_depth").get<double>());
                for (unsigned j=0;j<3;++j) equal &= near(frame.origin[j],actual.at("origin")[j].get<double>()) &&
                                                                 near(frame.delta[j],actual.at("delta")[j].get<double>());
                for (unsigned j=0;j<9;++j) equal &= near(frame.orientation[j],actual.at("orientation")[j].get<double>());
                check(equal,"new default view fields match complete original table fill including range fallback");
                bool layout=slot.layout_words==actual.at("layout_words").get<std::array<std::uint16_t,7>>();
                for (unsigned j=0;j<4;++j) layout &= near(slot.normalized_layout[j],actual.at("layout_normalized")[j].get<double>());
                check(layout,"default slot rectangle normalization and seven layout words match native table");
            }
        }
    }
    for (const auto &row:oracle.at("range_predicates")) {
        std::array<double,6> range{};
        for (unsigned i=0;i<6;++i) {
            const auto hex=row.at("range_bits")[i].get<std::string>();std::uint64_t bits=0;
            for (unsigned b=0;b<8;++b) bits |= std::uint64_t(std::stoul(hex.substr(2*b,2),nullptr,16))<<(8*b);
            std::memcpy(&range[i],&bits,8);
        }
        NativeDefaultRangeScaleContext c;c.record_present=false;
        const auto out=select_native_default_view_range(range,c);
        const bool fallback=row.at("invalid").get<bool>() || row.at("point").get<bool>();
        check(out.resolved && out.used_fallback==fallback,"range fallback matches original invalid/point predicates including NaN and threshold");
    }
    NativeDefaultViewTableInput in;
    check(!project_native_default_view_table(in).resolved,"unknown slots cannot be treated as empty");
    in.slots.fill(NativeViewSlotPresence::Present);
    check(project_native_default_view_table(in).resolved,"all slots present bypass missing model, range, layouts and selection context");
    in.slots[0]=NativeViewSlotPresence::Absent;
    check(!project_native_default_view_table(in).resolved,"missing slot needs model presence");
    in.model_present=false;
    check(project_native_default_view_table(in).reason=="default_table_copy_source_layout_unknown","copy needs known source layout");
    in.layouts[1]=NativeViewSlotPresence::Absent;
    check(project_native_default_view_table(in).reason=="default_table_copy_source_layout_absent","known null source layout is a native dereference");
    in.layouts[1]=NativeViewSlotPresence::Present;
    check(project_native_default_view_table(in).resolved,"copy path skips range, model queries and selection mode");
    in.slots.fill(NativeViewSlotPresence::Absent);
    check(!project_native_default_view_table(in).resolved,"all-empty table with no model cannot be synthesized");
    in.model_present=true;
    check(project_native_default_view_table(in).reason=="combined_model_range_not_established","base model range alone is not a proved union range");
    NativeDefaultRangeScaleContext scale;
    const std::array<double,6> point{1,2,3,1,2,3};
    check(!select_native_default_view_range(point,scale).resolved,"fallback requires scale context");
    scale.record_present=true;scale.value_28=4;scale.code_30=1;scale.code_68=2;
    check(select_native_default_view_range(point,scale).range==std::array<double,6>{-4,-4,-4,4,4,4},"different codes skip ratio fields");
    scale.code_68=1;scale.value_38=0;
    check(select_native_default_view_range(point,scale).resolved,"first nonpositive ratio term skips later unknown fields");
    scale.value_38=2;
    check(!select_native_default_view_range(point,scale).resolved,"positive prefix requires the next ratio field");
    NativeCachedModelBoundsInput bounds;
    check(!project_native_cached_model_bounds(bounds).resolved,"unknown provider pointer cannot become missing bounds");
    bounds.range_pointer_known=true;
    check(project_native_cached_model_bounds(bounds).return_code==1 && !project_native_cached_model_bounds(bounds).range,
          "known null provider range skips spatial byte and does not write output");
    bounds.integer_range=std::array<std::int64_t,6>{0,0,30,1,1,-30};
    check(!project_native_cached_model_bounds(bounds).resolved,"known range needs spatial byte before axis validity");
    bounds.spatial_byte_78=0;
    check(project_native_cached_model_bounds(bounds).range==std::array<double,6>{0,0,0,1,1,0},"2D bounds ignore and zero invalid Z interval");
    bounds.spatial_byte_78=255;
    check(project_native_cached_model_bounds(bounds).return_code==1 && !project_native_cached_model_bounds(bounds).range,
          "all nonzero spatial bytes require valid Z and leave output untouched on failure");
    return checks;
}
