#include "internal.hpp"
#include <p3d/model_bounds.hpp>
#include <limits>

namespace p3d {
namespace {
std::array<std::int64_t,6> range_at(const Bytes &data, std::size_t offset) {
    Reader reader(data,offset);
    std::array<std::int64_t,6> range{};
    for (auto &value:range) value=reader.i64();
    return range;
}
bool valid_range(const std::array<std::int64_t,6> &range, bool spatial) {
    for (unsigned axis=0;axis<(spatial?3u:2u);++axis)
        if (range[axis]>range[axis+3] || range[axis]==std::numeric_limits<std::int64_t>::max() ||
            range[axis+3]==std::numeric_limits<std::int64_t>::min()) return false;
    return true;
}
} // namespace

Json decode_native_record_bounds_header(const Bytes &data) {
    Json out={{"scope","persisted_record_extended_header"},
              {"source_offsets_include_stream_prefix",true},
              {"runtime_inclusion","not_evaluated"}};
    if (data.size()<8) { out["status"]="invalid"; out["reason"]="record_flags_truncated"; return out; }
    if (!(Reader(data,6).u16()&0x20)) { out["status"]="not_present"; return out; }
    if (data.size()<38) { out["status"]="invalid"; out["reason"]="extended_flags_truncated"; return out; }
    const auto flags=Reader(data,36).u16();
    out.update({{"extended_flags",flags},{"extended_flags_offset",36},
                {"provider_header_excluded",bool(flags&0xc00)}, {"integer_range_offset",60}});
    if (data.size()<108) { out["status"]="invalid"; out["reason"]="integer_range_truncated"; return out; }
    const auto source=range_at(data,60);
    auto range=source;
    // 107060 expands persisted upper deltas in place before entity input.
    // x64 ADD wraps modulo 2^64; avoid C++ signed-overflow undefined behavior.
    for (unsigned axis=0;axis<3;++axis) {
        const auto upper=static_cast<std::uint64_t>(source[axis])+
                         static_cast<std::uint64_t>(source[axis+3]);
        std::memcpy(&range[axis+3],&upper,sizeof(upper));
    }
    out.update({{"status","decoded"},{"integer_range",range},
                {"source_encoding","lower_and_upper_delta_modulo_2_64"},
                {"source_integer_values",source},
                {"valid_xy",valid_range(range,false)}, {"valid_xyz",valid_range(range,true)}});
    return out;
}

NativeModelBoundsProviderResult project_native_model_bounds_provider(const NativeModelBoundsProviderInput &in) {
    NativeModelBoundsProviderResult out;
    auto fail=[&](const char *reason) { out.reason=reason; out.included_record_indices.clear(); return out; };
    if (!in.record_list_complete) return fail("model_bounds_record_list_incomplete");
    if (!in.model_query_68) return fail("model_bounds_dimension_query_unknown");
    if (in.record_indices.size()>in.max_record_visits) return fail("model_bounds_record_visit_limit");
    constexpr std::int64_t low=(std::int64_t{1}<<52)-1, high=-(std::int64_t{1}<<52);
    std::array<std::int64_t,6> combined{low,low,low,high,high,high};
    for (auto index:in.record_indices) {
        out.failed_record=index;
        if (index>=in.records.size()) return fail("model_bounds_record_index_out_of_range");
        const auto &record=in.records[index];
        if (!record.runtime_flags_10) return fail("model_bounds_runtime_flags_unknown");
        const auto runtime=*record.runtime_flags_10;
        if (runtime&0x20008) continue;
        const auto &data=record.record_base;
        if (data.size()<8) return fail("model_bounds_record_flags_truncated");
        if (!(Reader(data,6).u16()&0x20)) continue;
        if (data.size()<38) return fail("model_bounds_extended_flags_truncated");
        if (Reader(data,36).u16()&0xc00) continue;
        std::array<std::int64_t,6> range{};
        if ((runtime&0xc00000)==0x400000) {
            if (!record.override_lookup_known) return fail("model_bounds_override_lookup_unknown");
            if (record.override_range) range=*record.override_range;
            else {
                if (data.size()<116) return fail("model_bounds_shifted_range_truncated");
                range=range_at(data,68);
            }
        } else {
            if (data.size()<108) return fail("model_bounds_range_truncated");
            range=range_at(data,60);
        }
        if (!valid_range(range,*in.model_query_68)) continue;
        for (unsigned axis=0;axis<3;++axis) {
            combined[axis]=std::min(combined[axis],range[axis]);
            combined[axis+3]=std::max(combined[axis+3],range[axis+3]);
        }
        out.included_record_indices.push_back(index);
    }
    out.failed_record.reset();out.integer_range=combined;out.resolved=true;return out;
}
} // namespace p3d
