#pragma once
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <p3d/reader.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
namespace p3d {
inline void require(bool v, const std::string &s) {
    if (!v)
        throw std::runtime_error(s);
}
struct Reader {
    const Bytes &b;
    std::size_t p = 0;
    Reader(const Bytes &b_, std::size_t p_ = 0) : b(b_), p(p_) {
        require(p <= b.size(), "reader offset");
    }
    void need(std::size_t n) const {
        require(p <= b.size() && n <= b.size() - p, "binary overrun at " + std::to_string(p));
    }
    template <class T> T get() {
        need(sizeof(T));
        T v;
        std::memcpy(&v, b.data() + p, sizeof(T));
        p += sizeof(T);
        return v;
    }
    template <class T> T at(std::size_t q) const {
        Reader r(b, q);
        return r.get<T>();
    }
    Bytes take(std::size_t n) {
        need(n);
        Bytes v(b.begin() + p, b.begin() + p + n);
        p += n;
        return v;
    }
    void skip(std::size_t n) {
        need(n);
        p += n;
    }
    std::size_t left() const {
        return b.size() - p;
    }
    std::uint8_t u8() {
        return get<std::uint8_t>();
    }
    std::uint16_t u16() {
        return get<std::uint16_t>();
    }
    std::uint32_t u32() {
        return get<std::uint32_t>();
    }
    std::uint64_t u64() {
        return get<std::uint64_t>();
    }
    std::int32_t i32() {
        return get<std::int32_t>();
    }
    std::int64_t i64() {
        return get<std::int64_t>();
    }
    double f64() {
        return get<double>();
    }
    std::uint64_t count(char fmt = 'Q', std::size_t minimum = 1) {
        auto n = fmt == 'I' ? u32() : fmt == 'H' ? u16() : u64();
        require(n <= left() / minimum, "collection count overrun");
        return n;
    }
    Json number(const std::string &fmt = "I") {
        Json a = Json::array();
        unsigned n = 0;
        for (char c : fmt) {
            if (c >= '0' && c <= '9') {
                n = n * 10 + c - '0';
                continue;
            }
            for (unsigned i = 0; i < (n ? n : 1); ++i) {
                switch (c) {
                case 'B':
                    a.push_back(u8());
                    break;
                case 'H':
                    a.push_back(u16());
                    break;
                case 'I':
                    a.push_back(u32());
                    break;
                case 'Q':
                    a.push_back(u64());
                    break;
                case 'i':
                    a.push_back(i32());
                    break;
                case 'q':
                    a.push_back(i64());
                    break;
                case 'd':
                    a.push_back(f64());
                    break;
                case 'f':
                    a.push_back(get<float>());
                    break;
                case 'h':
                    a.push_back(get<std::int16_t>());
                    break;
                default:
                    throw std::runtime_error("binary format");
                }
            }
            n = 0;
        }
        require(n == 0 && !a.empty(), "binary format count");
        return a.size() == 1 ? a[0] : a;
    }
    Json expect(const std::string &fmt, const Json &expected) {
        auto v = number(fmt);
        require(v == expected, "unsupported binary header at " + std::to_string(p));
        return v;
    }
    std::string string(char fmt = 'Q');
    std::string zstring();
    Json doubles(std::size_t n) {
        require(n <= left() / 8, "double array overrun");
        Json a = Json::array();
        for (std::size_t i = 0; i < n; ++i)
            a.push_back(f64());
        return a;
    }
    Json uints(std::size_t n, unsigned width = 4) {
        require(n <= left() / width, "integer array overrun");
        Json a = Json::array();
        for (std::size_t i = 0; i < n; ++i)
            a.push_back(width == 8 ? u64() : width == 2 ? u16() : width == 1 ? u8() : u32());
        return a;
    }
    void finish() const {
        require(p == b.size(), "trailing binary bytes at " + std::to_string(p));
    }
};
Json electrical_parameters(Reader &, unsigned);
Bytes slice(const Bytes &, std::size_t, std::size_t);
std::string hex(const Bytes &);
std::string utf16(const Bytes &);
std::string latin1(const Bytes &);
std::string utf8(const Bytes &);
std::string gb18030(const Bytes &);
std::string native_string(const Bytes &);
inline Json rawbytes(const Bytes &b) {
    return Json{{"base64", base64(b)}, {"bytes", b.size()}};
}
inline Bytes bytesof(const Json &v) {
    return unbase64(v.at("base64").get<std::string>());
}
Json xml_tree(const std::string &);
Json native_block_transform(const Bytes &);
Json native_record_input_filter(const Json &, const Bytes &);
Json native_record_input_subtree(std::size_t, const Json &, std::uint32_t);
Json command_fields(unsigned, const Bytes &);
Json decode_attribute(unsigned, unsigned, const Bytes &, unsigned index = 0);
Json decode_material_assignment(unsigned index, const Bytes &);
Json material_assignment_records(const Json &graphics);
Json decode_display_attribute(unsigned, unsigned, const Bytes &);
Json decode_layer_group_attribute(unsigned, const Bytes &);
Json native_display_state(unsigned type, const Bytes &);
Json decode_terrain(const Json &);
Json decode_symbology(const Bytes &);
Json decode_curve_identifier(const Bytes &);
Json decode_native_layer(const Bytes &, const Json &);
Json decode_native_layer_table(const Bytes &, const Json &);
Json build_layer_tables(const Json &, const Json &, const Json &, const Json & = Json::object());
Json decode_symbology_extension(const Bytes &);
void apply_symbology_extension(Json &, const Json &);
void apply_symbology(Json &, const Json &);
Json decode_polyface(const Bytes &);
Json decode_mesh_channels(const Bytes &, const Json &indices, const Json &polygons,
                          std::uint32_t num_per_face);
bool has_mesh_channels(const Json &);
Json mesh_triangle_channels(const Json &, const std::vector<std::optional<std::uint32_t>> &polygons,
                            const std::vector<Triangle> &corners);
void reverse_mesh_channel_corners(Json &);
Json decode_bgfb(const Bytes &);
Json decode_inline_material(const Bytes &);
Json parse_relationships(const Bytes &);
Json build_object_graph(const Document &);
Json build_graph_records(const Json &, const Json &, const Json &, const std::set<std::uint64_t> &,
                         const std::set<std::string> &);
Json build_scene(const Document &, unsigned);
Json read_materials(const Document &);
Json native_material_references(const Json &native_records);
Json native_block_header(const Bytes &);
Json native_file_header(const Bytes &index_stream, const Bytes &header_payload);
Json native_dependency_link(const Bytes &payload);
Json native_reference_path(const Bytes &base, const Json &links);
Json native_application_record(const Bytes &base);
Json native_system_id_assignments(const Json &list, const Json &records, const Json &file_header);
Json native_list_record_header(const Json &, const Json &conversion, bool child,
                              bool compound, std::uint32_t descendants, bool system);
Json native_list_input_preparation(const Json &container, const Json &records);
Json native_input_containers(const std::vector<Stream> &, const Json &index, const Json &records);
Json native_material_catalog_records(const Json &native_records,
                                    const std::vector<std::size_t> *selected_members = nullptr);
Json native_material_catalog_registration(const Json &selection, const Json &sources,
                                          const MaterialCatalogOptions &);
Json native_material_catalog_tables(const Json &native_records, Json &catalog_records);
Json native_system_material_table(const Json &list, const Json &records, const Json &ids);
Json native_attribute_lookup(const Json &attributes);
Json native_material_attribute(const Json &member, const Json &input);
Json decode_material_auxiliary_records(const Bytes &);
Json material_xml_integer(const Json &, const std::string &, bool signed_value);
Json material_root_input(const Json &, const Json &mode);
Json material_version_conversion(const Json &input, const Json &maps, const Json &bindings,
                                 const Json &mode);
Json native_system_attribute_input(const std::vector<Stream> &, const Json &index,
                                   const StreamPath &system, const Json &ids);
Json material_settings(const Json &tree);
Json material_parameter_semantics(const Json &attributes);
Json material_map_semantics(const Json &attributes);
Json material_layer_semantics(const Json &attributes);
Json material_replicator_nodes(const Json &owner);
Json material_replicator_input(const Json &package);
Json material_replicator_copies(const Json &map);
Json material_reader_paths(const Json &tree, Json &maps);
Json material_legacy_parameters(const Json &owner, const Json &dispatch);
Json material_resource_reference(const Json &source);
Json material_texture_references(const Json &tree, const Json &settings);
Json material_procedure_nodes(const Json &owner);
Json material_map_bindings(const Json &maps);
Json embedded_texture_records(const Json &graphics, const Json &materials);
Json read_schemas(const Document &);
Json read_models(const Document &);
void enrich_tree(Json &, const std::string & = "", const Json & = Json::object());
Json leaf_fields(const Json &);
Json text_json(const Bytes &);
const Json &bgfb_schema();
const Json &default_palette();
} // namespace p3d
