#pragma once
#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>
namespace p3d {
using Json = nlohmann::json;
using Bytes = std::vector<std::uint8_t>;
using StreamPath = std::vector<std::string>;
struct Options {
    unsigned threads = 1;
    std::size_t max_stream_bytes = 256u * 1024 * 1024;
    bool decode_binary_fields = true;
};
struct Stream {
    StreamPath path;
    std::shared_ptr<const Bytes> raw;
    std::shared_ptr<const Bytes> decoded;
    std::size_t compression_offset = static_cast<std::size_t>(-1);
};
using Point3 = std::array<double, 3>;
using Point2 = std::array<double, 2>;
using Triangle = std::array<std::uint32_t, 3>;
using Matrix4 = std::array<std::array<double, 4>, 4>;
// Validated knot direction shared by curves and tensor-product surfaces.
class BsplineDirection {
  public:
    unsigned order() const {
        return order_;
    }
    bool closed() const {
        return closed_;
    }
    std::size_t pole_count() const {
        return pole_count_;
    }
    const std::vector<double> &source_knots() const {
        return source_knots_;
    }
    const std::vector<double> &knots() const {
        return knots_;
    }
    std::array<double, 2> knot_domain() const;
    int periodic_pole_shift() const {
        return pole_shift_;
    }

  private:
    friend class BsplineCurve;
    friend class BsplineSurface;
    BsplineDirection() = default;
    static BsplineDirection from_data(unsigned, bool, std::size_t, std::vector<double>);
    unsigned order_ = 0;
    bool closed_ = false;
    std::size_t pole_count_ = 0;
    int pole_shift_ = 0;
    std::vector<double> source_knots_, knots_;
};
// Owns a validated copy of a decoded BGFB BsplineCurve table.
// Rational source poles are weighted XYZ, not Cartesian control points.
class BsplineCurve {
  public:
    static BsplineCurve from_bgfb(const Json &table);
    const BsplineDirection &direction() const {
        return direction_;
    }
    unsigned order() const {
        return direction_.order();
    }
    bool closed() const {
        return direction_.closed();
    }
    bool rational() const {
        return !weights_.empty();
    }
    const std::vector<Point3> &poles() const {
        return poles_;
    }
    const std::vector<double> &weights() const {
        return weights_;
    }
    const std::vector<double> &source_knots() const {
        return direction_.source_knots();
    }
    const std::vector<double> &knots() const {
        return direction_.knots();
    }
    std::array<double, 2> knot_domain() const {
        return direction_.knot_domain();
    }
    int periodic_pole_shift() const {
        return direction_.periodic_pole_shift();
    }
    // Fraction is in [0, 1]. Internal knots use the right-hand value;
    // fraction 1 uses the left-hand endpoint value. No implicit wrapping.
    std::array<double, 4> homogeneous_at(double fraction) const;
    // Throws at zero evaluated weight or a non-finite Cartesian result.
    Point3 point_at(double fraction) const;

  private:
    BsplineCurve() = default;
    BsplineDirection direction_;
    std::vector<Point3> poles_;
    std::vector<double> weights_;
};
// Evaluates the underlying surface; trim containment/meshing is a separate step.
// Source pole/weight index is v * u().pole_count() + u.
class BsplineSurface {
  public:
    static BsplineSurface from_bgfb(const Json &table);
    const BsplineDirection &u() const {
        return u_;
    }
    const BsplineDirection &v() const {
        return v_;
    }
    bool rational() const {
        return !weights_.empty();
    }
    const std::vector<Point3> &poles() const {
        return poles_;
    }
    const std::vector<double> &weights() const {
        return weights_;
    }
    const Json &boundaries() const {
        return boundaries_;
    }
    int hole_origin() const {
        return hole_origin_;
    }
    bool outer_boundary_active() const {
        return hole_origin_ == 0;
    }
    int num_rules_u() const {
        return num_rules_u_;
    }
    int num_rules_v() const {
        return num_rules_v_;
    }
    // Fractions in [0,1] map independently to each full active knot domain.
    // Does not test whether the point belongs to the trimmed region.
    std::array<double, 4> homogeneous_at(double fraction_u, double fraction_v) const;
    Point3 point_at(double fraction_u, double fraction_v) const;

  private:
    BsplineSurface() = default;
    BsplineDirection u_, v_;
    std::vector<Point3> poles_;
    std::vector<double> weights_;
    Json boundaries_;
    int hole_origin_ = 0, num_rules_u_ = 0, num_rules_v_ = 0;
};
struct Tessellation {
    unsigned full_circle_segments = 64;
    std::optional<double> chord_tolerance;
    unsigned max_segments = 100000;
    unsigned min_full_circle_segments = 8;
    unsigned segments(double radius_bound, double sweep = 6.2831853071795864769) const;
};
struct Geometry {
    std::vector<Point3> vertices;
    std::vector<Triangle> faces;
    std::vector<std::vector<Point3>> lines;
    Json texts = Json::array(), unknown = Json::array(), notes = Json::array(),
         primitive_ranges = Json::array();
    std::vector<std::optional<std::array<Point2, 3>>> face_uvs;
    std::vector<std::optional<std::uint32_t>> face_source_polygons;
};
struct GeometryDefinition {
    std::string source_key;
    Json source;
    Geometry geometry;
};
struct GeometryInstance {
    std::size_t definition;
    Matrix4 matrix;
    Json geometry_id, source, style = Json::object();
    bool apply_placement = true;
};
struct SceneElement {
    Json metadata;
    std::vector<GeometryInstance> instances;
};
struct PrimitiveView {
    std::size_t element_index, instance_index, definition_index;
    const Geometry *geometry = nullptr;
    const Json *source_range = nullptr;
    Matrix4 matrix;
    bool winding_reversed = false;
    Json style, appearance;
    std::vector<std::size_t> material_candidates;
    std::string material_status, topology, uv_status;
};
struct NativeScene {
    std::vector<std::shared_ptr<const GeometryDefinition>> definitions;
    std::vector<SceneElement> elements;
    Json metadata;
    // Explicit expansion for clients that require baked vertex arrays.
    // The native representation above never merges unrelated source definitions.
    Json expanded() const;
    Json summary() const;
    // Views borrow geometry/ranges from this scene; callback must not mutate it.
    void for_each_primitive(const std::function<void(const PrimitiveView &)> &) const;
};
// Parsed once in source stream order. All IDs retain their original scopes.
// References returned by accessors are immutable and remain valid for the document lifetime.
class Document {
  public:
    explicit Document(const std::filesystem::path &, Options = {});
    const std::vector<Stream> &streams() const;
    const Json &index() const;
    const Json &models() const;
    const Json &objects() const;
    const Json &native_records() const;
    const Json &graphics_records() const;
    const Json &bindings() const;
    const Json &relationships() const;
    const Json &materials() const;
    const Json &schemas() const;
    const Json &color_tables() const;
    // Source table membership and scoped attribute references; no inferred merging.
    const Json &layer_tables() const;
    Json binary_fields() const;
    Json inline_materials() const;
    // Embedded files with source attribute identity; does not read or write external files.
    Json embedded_textures() const;
    // Native advanced/legacy name and part assignments, with source identity.
    Json material_assignments() const;
    Json object_graph() const;
    Json scene(unsigned segments = 64) const;
    NativeScene native_scene(Tessellation = {}) const;
    Json summary() const;

  private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};
Json decode_binary_field(const std::string &name, const Bytes &, const std::string &root_class = "",
                         const Json &root_fields = Json::object());
Json parse_dex(const Bytes &, std::size_t &offset, bool enrich = true);
Json parse_native(const Bytes &);
Json parse_graphics(const Bytes &);
Json parse_commands(const Bytes &);
std::string base64(const Bytes &);
Bytes unbase64(const std::string &);
Json element_context(const Json &graph, std::uint64_t model_id, std::uint64_t element_id);
Json get_object(const Json &graph, std::uint64_t class_id, std::uint64_t object_id);
} // namespace p3d
