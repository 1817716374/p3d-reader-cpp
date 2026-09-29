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
void intersect(Range &range,const Range &clip) {
    for (unsigned i=0;i<3;++i) {
        range[i]=std::max(range[i],clip[i]);range[i+3]=std::min(range[i+3],clip[i+3]);
    }
    for (unsigned i=0;i<3;++i) if (range[i]>=range[i+3]) {range=empty_range();return;}
}
// Returns no clip range when no non-inverted polygon supplies a range.
std::string inline_clip_range(const NativeReferenceBoundsNode &node,const Json &affine,std::optional<Range> &out) {
    const auto &clip=*node.inline_clip;
    if (clip.points.size()!=*node.clip_count_2d8) return "reference_clip_point_count_mismatch";
    if (clip.points.size()>2500) return "reference_clip_point_limit";
    if (!clip.selected_matrix) return "reference_clip_matrix_unknown";
    if (!clip.depths_allowed) return "reference_clip_depth_gate_unknown";
    Matrix4 matrix{};matrix[3][3]=1;
    const auto translation=node.reference_input.at("affine_inputs").at("translation_point").at("value").get<Point3>();
    Point3 correction{};
    if (affine.at("origin_correction").at("native_return_code")==0)
        correction=affine.at("origin_correction").at("offset").get<Point3>();
    for (unsigned i=0;i<3;++i) {
        for (unsigned j=0;j<3;++j) {
            matrix[i][j]=(*clip.selected_matrix)[i][j];
            if (!std::isfinite(matrix[i][j])) return "nonfinite_reference_clip_matrix";
        }
        matrix[i][3]=translation[i]-correction[i];
        if (!std::isfinite(matrix[i][3])) return "nonfinite_reference_clip_translation";
    }
    const bool lower_enabled=*clip.depths_allowed && (*node.runtime_flags_9c&0x400u);
    const bool upper_enabled=*clip.depths_allowed && (*node.runtime_flags_9c&0x800u);
    if ((lower_enabled&&!clip.lower_288)||(upper_enabled&&!clip.upper_280)) return "reference_clip_depth_unknown";
    double lower=lower_enabled?*clip.lower_288:-4503599627370496.;
    double upper=upper_enabled?*clip.upper_280:4503599627370495.;
    if (!std::isfinite(lower)||!std::isfinite(upper)) return "nonfinite_reference_clip_depth";
    if (lower==upper) {lower-=1e-6;upper+=1e-6;}
    std::vector<std::vector<std::array<double,2>>> loops(1);
    const auto marker=std::numeric_limits<double>::max();
    for (std::size_t i=0;i<clip.points.size();++i) {
        const auto &p=clip.points[i];
        if (p[0]==marker&&p[1]==marker) {
            if (loops.back().size()>=3 || i==0) loops.emplace_back();
            else loops.back().clear();
        } else {
            if (!std::isfinite(p[0])||!std::isfinite(p[1])||p[0]==marker||p[1]==marker)
                return "reference_clip_invalid_point";
            loops.back().push_back(p);
        }
    }
    if (loops.back().size()<3) loops.pop_back();
    if (!loops.empty()&&loops.front().empty()&&(lower_enabled||upper_enabled)) {
        // Native plane transformation solves M^T*n=eZ, normalizes, and
        // computes distance at the transformed point. Singular M leaves eZ.
        const auto &m=*clip.selected_matrix;
        const auto determinant=(m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])+
                                m[0][1]*(m[1][2]*m[2][0]-m[1][0]*m[2][2]))+
                                m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
        if (!std::isfinite(determinant)) return "nonfinite_reference_clip_plane_transform";
        Point3 normal{0,0,1};
        if (determinant!=0) {
            normal={(m[1][0]*m[2][1]-m[1][1]*m[2][0])/determinant,
                    (m[0][1]*m[2][0]-m[0][0]*m[2][1])/determinant,
                    (m[0][0]*m[1][1]-m[0][1]*m[1][0])/determinant};
            const auto length=std::sqrt((normal[0]*normal[0]+normal[1]*normal[1])+normal[2]*normal[2]);
            if (!(length>0)||!std::isfinite(length)) return "nonfinite_reference_clip_plane_normal";
            for (auto &v:normal) v/=length;
        }
        Range range{-1e20,-1e20,-1e20,1e20,1e20,1e20};const auto initial=range;
        for (unsigned end=0;end<2;++end) {
            if (!(end?upper_enabled:lower_enabled)) continue;
            auto n=normal; if (end) for (auto &v:n) v=-v;
            const auto point=transform(matrix,{0,0,end?upper:lower});
            const auto distance=(point[1]*n[1]+point[0]*n[0])+point[2]*n[2];
            if (!finite(point)||!std::isfinite(distance)) return "nonfinite_reference_clip_plane_distance";
            const auto squared=(n[1]*n[1]+n[0]*n[0])+n[2]*n[2];
            for (unsigned axis=0;axis<3;++axis) {
                double cross_squared=0;
                for (unsigned k=0;k<3;++k) if (k!=axis) cross_squared+=n[k]*n[k];
                if (cross_squared<=squared*1e-24) {
                    if (n[axis]>0) range[axis]=std::max(range[axis],distance);
                    else range[axis+3]=std::min(range[axis+3],-distance);
                }
            }
        }
        if (range==initial) return {}; // Plane range helper reports no bounded axis.
        if (invalid(range)) return {}; // 3530 rejects out-of-range plane bounds.
        // 3530 extends both endpoints into an empty range, normalizing even
        // reversed depth planes before the collection tests range ordering.
        for (unsigned i=0;i<3;++i) if (range[i]>range[i+3]) std::swap(range[i],range[i+3]);
        out=range;return {};
    }
    if (loops.empty()||loops.front().empty()) return {};
    Range range=empty_range();
    for (const auto &p:loops.front()) for (double z:{lower,upper}) {
        const auto q=transform(matrix,{p[0],p[1],z});
        if (!finite(q)) return "nonfinite_reference_clip_bounds";
        extend(range,q);
    }
    out=range;return {};
}
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
        if (!node.clip_pointer_278_present) return fail("reference_bounds_clip_pointer_unknown",index);
        if (*node.clip_pointer_278_present) return fail("reference_bounds_active_clip_required",index);
        if (*node.clip_count_2d8) {
            if (!node.inline_clip) return fail("reference_bounds_active_clip_required",index);
            std::optional<Range> clip;
            const auto reason=inline_clip_range(node,affine,clip);
            if (!reason.empty()) return fail(reason,index);
            if (clip) intersect(range,*clip);
        }
        unite(parent,range);return true;
    };
    for (auto index:in.root_references) if (!visit(index,1,combined)) return out;
    out.range=combined;out.resolved=true;return out;
}
} // namespace p3d
