#include "internal.hpp"
#include <p3d/reference_bounds.hpp>
#include "reference_bounds_oracle.hpp"
using namespace p3d;
unsigned reference_bounds_tests() {
    unsigned checks=0;
    auto check=[&](bool value,const char *message){++checks;require(value,message);};
    auto near=[](double a,double b){
        if (a==b) return true;
        if (!std::isfinite(a) || !std::isfinite(b) || std::abs(a)>=1e100 || std::abs(b)>=1e100) return false;
        return std::abs(a-b)<=3e-12*(1+std::abs(a)+std::abs(b));
    };
    auto bounds=[](const Json &value,bool spatial) {
        NativeCachedModelBoundsInput b;b.range_pointer_known=true;b.spatial_byte_78=spatial?1:0;
        if (!value.is_null()) b.integer_range=value.get<std::array<std::int64_t,6>>();
        return b;
    };
    const auto oracle=Json::parse(reference_bounds_oracle);
    NativeModelReferenceBoundsInput sample;
    for (const auto &row:oracle.at("cases")) {
        NativeModelReferenceBoundsInput input;
        input.root_bounds=bounds(row.at("root_bounds"),true);input.root_references_complete=true;
        input.root_references=row.at("roots").get<std::vector<std::size_t>>();
        for (const auto &n:row.at("nodes")) {
            NativeReferenceBoundsNode node;
            node.target_model_present=n.at("target_present");node.no_model_provider_present=false;
            node.target_bounds=bounds(n.at("bounds"),n.at("spatial"));
            node.children=n.at("children").get<std::vector<std::size_t>>();node.children_complete=true;
            Matrix3 matrix{};
            for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j) matrix[i][j]=n.at("matrix")[3*i+j];
            node.runtime_flags_9c=n.at("perspective").get<bool>()?0x1000:0;
            node.reference_input={{"status","decoded"},{"transform",{{"status","computed"},{"scale",n.at("scale")},{"matrix",matrix}}},
                                  {"affine_inputs",{{"translation_point",{{"value",n.at("translation")}}},{"reference_point",{{"value",n.at("reference_point")}}}}},
                                  {"origin_inputs",{{"primary_flags",0},{"secondary_flags",*node.runtime_flags_9c}}}};
            node.affine_context.provider_id=0;node.affine_context.force_z_scale=false; // bounds must force true itself
            node.affine_context.origin.model_attached=n.at("attached");
            if (n.at("attached").get<bool>()) node.affine_context.origin.model_coordinates=Json{
                {"status","decoded"},{"reference_origin",{{"value",n.at("origin")}}},{"auxiliary_origin",{{"value",Point3{0,0,0}}}}};
            node.perspective_point_1b0=n.at("eye").get<Point3>();node.perspective_distance_1c8=n.at("distance");
            node.clip_count_2d8=0;node.clip_pointer_278_present=false;
            input.references.push_back(node);
        }
        const auto out=project_native_model_reference_bounds(input);
        check(out.resolved && out.range && out.reason.empty(),"bounded native recursive reference graph resolves");
        bool equal=true;
        for (unsigned i=0;i<6;++i) equal &= near((*out.range)[i],row.at("combined_range")[i].get<double>());
        if (!equal) throw std::runtime_error("reference bounds mismatch: actual="+Json(*out.range).dump()+" oracle="+row.dump());
        check(equal,"root and recursive affine/perspective bounds match full original traversal");
        NativeDefaultViewTableInput table;
        table.slots.fill(NativeViewSlotPresence::Absent);table.model_present=true;table.model_query_68=true;
        table.directory.owning_file_present=false;table.combined_model_range=*out.range;table.suppress_selection=false;
        table.fallback_scale.record_present=true;table.fallback_scale.value_28=100000;
        table.fallback_scale.code_30=1;table.fallback_scale.code_68=2;
        const auto projected=project_native_default_view_table(table);
        check(projected.resolved,"computed recursive bounds feed complete default table projection");
        const auto &actual=row.at("table_slot0");const auto &view=*projected.slots[0].created_view;
        equal=view.flags==actual.at("flags").get<std::array<std::uint32_t,3>>();
        for (unsigned i=0;i<3;++i) equal &= near(view.frame.origin[i],actual.at("origin")[i].get<double>()) &&
                                                near(view.frame.delta[i],actual.at("delta")[i].get<double>());
        check(equal,"recursive bounds through range fallback and frame agree with original full table constructor");
        if (sample.references.empty() && !input.references.empty()) sample=input;
    }
    auto in=sample;
    in.root_references_complete=false;
    check(!project_native_model_reference_bounds(in).resolved,"incomplete root reference list is not an empty list");
    in=sample;in.references[0].target_model_present.reset();
    check(!project_native_model_reference_bounds(in).resolved,"unknown target presence cannot skip reference");
    in=sample;in.references[0].children_complete=false;
    check(!project_native_model_reference_bounds(in).resolved,"unknown descendants may change local model bounds");
    in=sample;in.references[0].children={0};
    check(project_native_model_reference_bounds(in).reason=="reference_bounds_cycle","active-path cycle is diagnosed without native recursion");
    in=sample;in.max_depth=0;
    check(project_native_model_reference_bounds(in).reason=="reference_bounds_depth_limit","reference recursion is bounded");
    in=sample;in.max_visits=0;
    check(project_native_model_reference_bounds(in).reason=="reference_bounds_visit_limit","repeated visits obey a global budget");
    in=sample;in.root_references={1};
    check(project_native_model_reference_bounds(in).reason=="reference_bounds_index_out_of_range","invalid graph identity is not missing geometry");
    in=sample;in.references[0].clip_count_2d8=1;
    check(project_native_model_reference_bounds(in).reason=="reference_bounds_active_clip_required","active clip cannot be silently ignored");
    in=sample;in.references[0].clip_pointer_278_present.reset();
    check(!project_native_model_reference_bounds(in).resolved,"unknown clipping pointer keeps valid transformed range unresolved");
    in=sample;in.references[0].target_bounds.integer_range.reset();in.references[0].reference_input=Json();
    in.references[0].runtime_flags_9c.reset();in.references[0].clip_count_2d8.reset();
    check(project_native_model_reference_bounds(in).resolved,"invalid local bounds after complete child traversal skip transform and clip dependencies");
    in.references[0].children_complete=false;
    check(!project_native_model_reference_bounds(in).resolved,"children could rescue invalid local bounds and cannot be omitted");
    in.references[0].target_model_present=false;
    check(project_native_model_reference_bounds(in).resolved,"missing model and known absent provider skip children and transform");
    in.references[0].no_model_provider_present=true;
    check(project_native_model_reference_bounds(in).reason=="reference_bounds_provider_callback_required","provider callback contribution cannot be guessed");
    in=sample;in.root_references={0,0};
    check(project_native_model_reference_bounds(in).visit_order==std::vector<std::size_t>{0,0},"shared reference identities are traversed in order, not globally deduplicated");
    in=sample;in.references[0].runtime_flags_9c=0x1000;in.references[0].perspective_distance_1c8.reset();
    check(!project_native_model_reference_bounds(in).resolved,"perspective bit requires its separate distance input");
    in=sample;in.references[0].runtime_flags_9c=0x1000;
    in.references[0].perspective_point_1b0=Point3{std::numeric_limits<double>::max(),0,0};
    check(project_native_model_reference_bounds(in).reason=="reference_bounds_perspective_point_sentinel",
          "perspective disconnect marker is not an ordinary finite eye position");
    return checks;
}
