#include <p3d/reference_bounds.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
namespace p3d {
namespace {
using Range=std::array<double,6>;
Range empty_range() {
    const auto m=std::numeric_limits<double>::max(); return {m,m,m,-m,-m,-m};
}
bool invalid(const Range &range) {
    return std::any_of(range.begin(),range.end(),[](double v){return !(std::abs(v)<1e100);});
}
void extend(Range &range,const Point3 &point) {
    if (std::any_of(point.begin(),point.end(),[](double v){return v==std::numeric_limits<double>::max();})) return;
    for (unsigned i=0;i<3;++i) { range[i]=std::min(range[i],point[i]);range[i+3]=std::max(range[i+3],point[i]); }
}
void unite(Range &range,const Range &other) {
    if (invalid(other)) return;
    extend(range,{other[0],other[1],other[2]});extend(range,{other[3],other[4],other[5]});
}
Point3 corner(const Range &range,unsigned bits) {
    return {range[(bits&1)?3:0],range[(bits&2)?4:1],range[(bits&4)?5:2]};
}
Point3 transform(const Matrix4 &matrix,const Point3 &point) {
    Point3 out{};
    for (unsigned i=0;i<3;++i)
        out[i]=((point[1]*matrix[i][1]+point[0]*matrix[i][0])+point[2]*matrix[i][2])+matrix[i][3];
    return out;
}
bool finite(const Point3 &point) { return std::all_of(point.begin(),point.end(),[](double v){return std::isfinite(v);}); }
}
NativeModelReferenceBoundsResult project_native_model_reference_bounds(const NativeModelReferenceBoundsInput &in) {
    NativeModelReferenceBoundsResult out;
    auto fail=[&](const std::string &reason,std::optional<std::size_t> index=std::nullopt) {
        out.reason=reason;out.failed_reference=index;return false;
    };
    const auto root=project_native_cached_model_bounds(in.root_bounds);
    if (!root.resolved) { fail(root.reason);return out; }
    Range combined=root.range.value_or(empty_range());
    if (!in.root_references_complete) { fail("root_reference_list_incomplete");return out; }
    std::vector<bool> active(in.references.size(),false);
    std::function<bool(std::size_t,std::size_t,Range&)> visit;
    visit=[&](std::size_t index,std::size_t depth,Range &parent) {
        if (index>=in.references.size()) return fail("reference_bounds_index_out_of_range",index);
        if (depth>std::min<std::size_t>(in.max_depth,256)) return fail("reference_bounds_depth_limit",index);
        if (out.visit_order.size()>=in.max_visits) return fail("reference_bounds_visit_limit",index);
        if (active[index]) return fail("reference_bounds_cycle",index);
        out.visit_order.push_back(index);
        const auto &node=in.references[index];
        if (!node.target_model_present) return fail("reference_bounds_target_presence_unknown",index);
        if (!*node.target_model_present) {
            if (!node.no_model_provider_present) return fail("reference_bounds_provider_presence_unknown",index);
            if (*node.no_model_provider_present) return fail("reference_bounds_provider_callback_required",index);
            return true;
        }
        const auto bounds=project_native_cached_model_bounds(node.target_bounds);
        if (!bounds.resolved) return fail(bounds.reason,index);
        Range range=bounds.range.value_or(empty_range());
        if (!node.children_complete) return fail("reference_bounds_child_list_incomplete",index);
        active[index]=true;
        for (auto child:node.children) if (!visit(child,depth+1,range)) return false;
        active[index]=false;
        if (invalid(range)) return true;
        auto context=node.affine_context; context.force_z_scale=true;
        const auto affine=reference_affine_transform(node.reference_input,context);
        if (affine.value("status","")!="computed") return fail(affine.value("reason","reference_bounds_affine_unresolved"),index);
        if (affine.value("native_translation_sentinel",false)) return fail("reference_bounds_affine_translation_sentinel",index);
        const auto matrix=affine.at("matrix").get<Matrix4>();
        Range transformed=empty_range();
        for (unsigned c=0;c<8;++c) {
            const auto point=transform(matrix,corner(range,c));
            if (!finite(point)) return fail("nonfinite_transformed_reference_bounds",index);
            extend(transformed,point);
        }
        range=transformed;
        if (!node.runtime_flags_9c) return fail("reference_bounds_runtime_flags_unknown",index);
        if (*node.runtime_flags_9c&0x1000u) {
            if (!node.perspective_point_1b0 || !node.perspective_distance_1c8)
                return fail("reference_bounds_perspective_input_unknown",index);
            if (!finite(*node.perspective_point_1b0)) return fail("nonfinite_reference_perspective_point",index);
            if (std::any_of(node.perspective_point_1b0->begin(),node.perspective_point_1b0->end(),
                            [](double v){return v==std::numeric_limits<double>::max();}))
                return fail("reference_bounds_perspective_point_sentinel",index);
            const auto eye=transform(matrix,*node.perspective_point_1b0);
            const auto focal=*node.perspective_distance_1c8*node.reference_input.at("transform").at("scale").get<double>();
            if (!finite(eye) || !std::isfinite(focal)) return fail("nonfinite_reference_perspective_parameters",index);
            transformed=empty_range();
            for (unsigned c=0;c<8;++c) {
                auto point=corner(range,c);
                if (eye[2]>point[2]) {
                    const auto ratio=focal/(eye[2]-point[2]);
                    point[0]=(point[0]-eye[0])*ratio+eye[0];point[1]=(point[1]-eye[1])*ratio+eye[1];
                } else point[2]=eye[2];
                if (!finite(point)) return fail("nonfinite_reference_perspective_bounds",index);
                extend(transformed,point);
            }
            range=transformed;
        }
        // Default-view traversal has no filter, so 1dc570 returns code 1.
        if (!node.clip_count_2d8) return fail("reference_bounds_clip_count_unknown",index);
        if (*node.clip_count_2d8) return fail("reference_bounds_active_clip_required",index);
        if (!node.clip_pointer_278_present) return fail("reference_bounds_clip_pointer_unknown",index);
        if (*node.clip_pointer_278_present) return fail("reference_bounds_active_clip_required",index);
        unite(parent,range);return true;
    };
    for (auto index:in.root_references) if (!visit(index,1,combined)) return out;
    out.range=combined;out.resolved=true;return out;
}
} // namespace p3d
