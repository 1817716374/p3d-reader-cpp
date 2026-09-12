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
// Native Akima point data plus its derived, open cubic B-spline. The first/last
// two retained points are tangent supports, not interpolated endpoints.
class AkimaCurve {
  public:
    static AkimaCurve from_bgfb(const Json &table);
    const std::vector<Point3> &source_points() const {
        return source_points_;
    }
    const std::vector<std::size_t> &retained_point_indices() const {
        return retained_indices_;
    }
    const BsplineCurve &bspline() const {
        return bspline_;
    }
    const Json &report() const {
        return report_;
    }

  private:
    explicit AkimaCurve(BsplineCurve curve) : bspline_(std::move(curve)) {}
    BsplineCurve bspline_;
    std::vector<Point3> source_points_;
    std::vector<std::size_t> retained_indices_;
    Json report_;
};
// BGFB interpolation data and its native cubic conversion. Serialized order,
// knots and endpoint tangents remain in source(); the BGFB conversion path does
// not use them. Prepared point indices include an appended closing point, if any.
class InterpolationCurve {
  public:
    static InterpolationCurve from_bgfb(const Json &table);
    const Json &source() const {
        return source_;
    }
    const std::vector<Point3> &prepared_points() const {
        return points_;
    }
    const std::vector<std::size_t> &prepared_point_indices() const {
        return indices_;
    }
    const std::vector<double> &parameters() const {
        return parameters_;
    }
    const BsplineCurve &bspline() const {
        return bspline_;
    }
    const Json &report() const {
        return report_;
    }

  private:
    explicit InterpolationCurve(BsplineCurve curve) : bspline_(std::move(curve)) {}
    BsplineCurve bspline_;
    Json source_, report_;
    std::vector<Point3> points_;
    std::vector<std::size_t> indices_;
    std::vector<double> parameters_;
};
struct SpiralEvaluation {
    Point3 point{}, derivative{}; // derivative with respect to the active [0,1] fraction
    double local_bearing = 0, local_curvature = 0;
    // Integration truncation bound in transformed coordinates; excludes roundoff.
    double quadrature_error_bound = 0;
    unsigned intervals = 0;
};
struct SpiralFit {
    BsplineCurve curve; // derived cubic, with the source affine transform applied
    Json report;        // local fit points, parameters and convergence information
};
// Underlying transition spiral, independent of the native fitted B-spline cache.
class TransitionSpiral {
  public:
    static TransitionSpiral from_bgfb(const Json &table);
    const Json &source() const {
        return source_;
    }
    const Json &report() const {
        return report_;
    }
    double length() const {
        return length_;
    } // full local spiral, before affine transform
    SpiralEvaluation evaluate(double fraction, double tolerance = 1e-8,
                              unsigned max_intervals = 65536) const;
    // Native cache fitter's local samples and endpoint constraints. The native
    // error estimate is not a guaranteed error bound. Fitting remains separate.
    Json native_fit_input(unsigned max_integration_intervals = 100000) const;
    // Reconstruct the native iterative fit. Throws on degenerate parameters,
    // the native point limit, or failure to converge within 30 iterations.
    SpiralFit native_fit(unsigned max_integration_intervals = 100000) const;

  private:
    TransitionSpiral() = default;
    double angle(double) const;
    double curvature(double) const;
    double integrand_fourth_bound(double, double) const;
    Json source_, report_;
    int type_ = 0;
    double length_ = 0, bearing_ = 0, curvature0_ = 0, curvature1_ = 0, start_ = 0, end_ = 1;
    std::array<double, 12> transform_{};
};
enum class TrimLocation { Inside, Outside, BoundaryBand, Indeterminate };
// Derived UV polylines with source provenance and a declared approximation bound.
// Original boundary curves remain available on BsplineSurface.
class BsplineTrim {
  public:
    const std::vector<std::vector<Point2>> &loops() const {
        return loops_;
    }
    const Json &report() const {
        return report_;
    }
    TrimLocation classify(Point2 uv) const;

  private:
    friend class BsplineSurface;
    BsplineTrim() = default;
    std::vector<std::vector<Point2>> loops_;
    std::vector<double> errors_;
    Json report_;
    double tolerance_ = 0;
    bool complete_ = false, outer_active_ = true;
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
    // Strokes source trim curves in UV coordinates. Incomplete results never
    // classify points as inside/outside. Does not build a surface mesh.
    BsplineTrim trim(double uv_tolerance, unsigned max_segments = 100000) const;

  private:
    BsplineSurface() = default;
    BsplineDirection u_, v_;
    std::vector<Point3> poles_;
    std::vector<double> weights_;
    Json boundaries_;
    int hole_origin_ = 0, num_rules_u_ = 0, num_rules_v_ = 0;
};
struct LoftSide {
    BsplineSurface surface;
    std::size_t loop_index = 0, primitive_index = 0;
};
// Explicit reconstruction of the side surfaces in a decoded P3DSectionLoft.
// Keeps source order and source data; does not replace the active graphics cache.
class SectionLoft {
  public:
    static SectionLoft from_bgfb(const Json &table, unsigned max_control_points = 100000);
    const Json &source() const {
        return source_;
    }
    const std::vector<LoftSide> &sides() const {
        return sides_;
    }
    const Json &report() const {
        return report_;
    }

  private:
    SectionLoft() = default;
    Json source_, report_;
    std::vector<LoftSide> sides_;
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
    // Independent source pools, concatenated without deduplication. Normal
    // values use inverse-transpose placement without unit normalization.
    // source_normals retain the original mesh values, including unused entries.
    std::vector<Point3> source_normals;
    std::vector<std::optional<Point3>> normals;
    std::vector<Point2> uvs;
    std::vector<std::optional<Triangle>> face_normal_indices, face_uv_indices;
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
struct MeshMaterialReference {
    std::uint64_t material_id = 0;
    // Indices into NativeScene::metadata["materials"]["definitions"].
    std::vector<std::size_t> candidates;
    std::string status;
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
    // One entry per source face_material_ids item; duplicates and order survive.
    // These are references, not the result of final appearance inheritance.
    std::vector<MeshMaterialReference> mesh_material_references;
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
    const Json &file_header() const;
    const Json &models() const;
    const Json &objects() const;
    const Json &native_records() const;
    // Conditional block input and record trees, initial SSYS IDs/material table;
    // does not instantiate host runtime objects or evaluate later host events.
    Json native_input_containers() const;
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
// Payload of a user linkage with app 0x4F5A, excluding its four-byte header.
// A supplied decoder converts the NUL-excluded ANSI bytes to UTF-8.
Json decode_native_material_name(
    const Bytes &payload, const std::function<std::string(const Bytes &)> &ansi_decoder = {});
Json parse_graphics(const Bytes &);
Json parse_commands(const Bytes &);
// Alternate native mesh consumer. Input is an opcode-25 decoded command.
// The limit bounds per-call generated/output corner storage, not a file constraint.
Json triangulate_native_mesh(const Json &decoded, std::size_t max_output_corners = 3000000);
std::string base64(const Bytes &);
Bytes unbase64(const std::string &);
Json element_context(const Json &graph, std::uint64_t model_id, std::uint64_t element_id);
Json get_object(const Json &graph, std::uint64_t class_id, std::uint64_t object_id);
} // namespace p3d
