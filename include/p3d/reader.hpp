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
// Supply the equality rule of the originating native wide-character locale.
// Without a callback only identical UTF-16 names are treated as known equal;
// an earlier nonidentical candidate keeps an ordered lookup unresolved.
using NativeModelNameEqual = std::function<bool(const std::u16string &, const std::u16string &)>;
Json lookup_native_model_directory(const Json &directory, const std::u16string &name,
                                   const NativeModelNameEqual &equal = {});
// Persisted reference_target views in current-to-host order. A false result
// requires the full selected host chain; later runtime changes are excluded.
Json initial_reference_file_query_gate(const std::vector<Json> &reference_targets,
                                       bool complete_host_chain);
// The two native strings of an already selected file's default specification.
// An empty string is known empty, not an unavailable file context.
struct NativeFileReference {
    std::u16string stored_reference;
    std::u16string lookup_reference;
};
// Explicit runtime inputs to the native persistence-change probe. Values are
// in the originating service's units; the parser does not read a system clock
// or infer them from a document path.
struct NativeFileChangeProbeContext {
    std::optional<bool> enabled;
    std::optional<std::int64_t> current_clock_value;
    std::optional<double> previous_check_value;
    std::optional<bool> persistence_available;
    std::optional<double> loaded_persistence_value;
    std::optional<double> current_persistence_value;
};
Json native_file_change_probe(std::uint32_t runtime_flags,
                              const NativeFileChangeProbeContext &context);
struct NativeOpenFileCandidate {
    std::optional<bool> valid;
    std::optional<NativeFileReference> reference;
    std::optional<std::uint32_t> runtime_flags;
    NativeFileChangeProbeContext change_probe;
};
struct ReferenceFileQueryContext {
    std::optional<NativeFileReference> host_file;
    std::optional<NativeFileReference> current_file;
    bool host_file_known_absent = false;
    bool current_file_known_absent = false;
    // Used only when the persisted key-48 string is empty. Supply the value
    // after host preparation; absence does not mean an empty value.
    std::optional<std::u16string> inherited_search_context;
    std::vector<Json> host_reference_targets;
    bool complete_host_chain = false;
    NativeModelNameEqual equal;
    // The resource service runs before registry lookup and can change the
    // lookup reference. Do not substitute the pre-service source string.
    std::optional<std::u16string> lookup_reference_after_resource_service;
    // Complete registry in native registration order, including duplicates.
    // Empty means known empty; nullopt means unavailable.
    std::optional<std::vector<NativeOpenFileCandidate>> open_files;
    std::optional<bool> allow_file_loading;
};
// Consumes a type-13 native record. Covers the initial default-service file
// query through host/current/registered-file reuse or file-loading handoff.
// It neither opens files nor selects host contexts or loads target models.
Json initial_reference_file_query(const Json &reference_record,
                                  const ReferenceFileQueryContext &context);
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
using Matrix3 = std::array<Point3, 3>;
// The selected referenced model supplies coordinates. A known unattached
// reference needs no model; otherwise an omitted attachment state is inferred
// only when model_coordinates is explicitly supplied. Auxiliary conditions
// are required only when they can affect the result.
struct ReferenceOriginContext {
    std::optional<Json> model_coordinates;
    std::optional<bool> model_attached;
    std::optional<std::uint16_t> auxiliary_origin_gate;
    std::optional<bool> auxiliary_origin_suppressed;
};
// Consumes a native record's reference_input and the selected model's
// coordinates. Returns a local correction, not a complete affine transform.
Json reference_origin_correction(const Json &reference_input,
                                 const ReferenceOriginContext &context);
// Explicit native query context. The chain query forces Z scaling; a local
// query that disables it needs the referenced model's Z scaling state.
struct ReferenceAffineContext {
    ReferenceOriginContext origin;
    bool force_z_scale = true;
    std::optional<bool> model_z_scale_enabled;
    std::optional<std::uint32_t> provider_id;
    std::optional<bool> provider_scale_available;
    std::optional<double> provider_y_scale;
};
Json reference_affine_transform(const Json &reference_input, const ReferenceAffineContext &context);
// Entries must be computed reference_affine_transform results in native
// current-to-host traversal order. Target selection is not inferred here.
Json compose_reference_chain_transforms(const std::vector<Json> &transforms);
// Owner-path conversion uses the same traversal order but PRE-multiplies
// subsequent references. Exclude the selected stop reference from the input.
Json compose_owner_reference_chain_transforms(const std::vector<Json> &transforms);
struct OwnerReferencePathTransformContext {
    // Runtime owner category, not the saved model_coordinates.kind field.
    std::optional<std::uint32_t> owner_kind;
    // Native collector's terminal position; preceding entries supply blocks.
    std::optional<std::uint32_t> terminal_index;
    // Required only at terminal_index == 0: status absent/computed/native_failure.
    // computed carries the affine matrix returned by the selected handler.
    Json single_object_transform;
};
// Already selected, accepted collected records in native append order; null
// entries represent known null objects. Does not resolve paths or load models.
// The explicit owner reference chain excludes its selected stop reference.
Json owner_reference_path_transform(const Json &collected_records,
                                    const std::vector<Json> &owner_reference_chain,
                                    const OwnerReferencePathTransformContext &context);
// Initial reference extension state from one complete, already selected
// persisted attribute collection. Does not merge runtime edits or locate targets.
Json reference_extension_input(const Json &attributes);
// Native CurveVector reference frame selection, preserving direct child order
// and nested arrays. Preference 1 favors endpoint axes; 2 uses endpoints only
// when their tangents are not parallel; other values use the local search.
// Reports computed/native_failure/not_evaluated; only computed has a frame.
Json native_curve_frame(const Json &curve_vector, int search_preference = 0);
// Inputs to the native projection preparation step. Geometry context is
// explicit; missing context is never replaced with guessed bounds or axes.
struct MaterialProjectionContext {
    std::int32_t mapping_mode = 0;
    std::int32_t scale_mode = 0;
    std::uint32_t layer_data_flags = 0;
    std::optional<Point3> reference_point;
    std::optional<Point3> reference_dimensions;
    std::optional<Matrix3> reference_matrix;
    std::optional<double> absolute_unit_factor;
};
// Consumes an entry of version_conversion.projection_getters. This prepares
// projection state for modes 3..7; it does not perform final point/UV mapping.
Json prepare_material_projection(const Json &getter, const MaterialProjectionContext &context);
// Continues preparation through native inversion, row scaling and the local
// explicit-matrix switch. Point mapping and texture sampling remain separate.
Json resolve_material_projection_transform(const Json &getter,
                                           const MaterialProjectionContext &context);
using Matrix2x3 = std::array<Point3, 2>;
// Inputs that depend on the selected model/render context. The source layer
// fields and U/V signs come from layer_mapping_getters, not from this context.
struct MaterialUvTransformContext {
    std::optional<double> mapping_unit_factor;
    std::optional<Point2> elevation_origin;
    std::optional<bool> geometry_projection_succeeded;
    std::optional<double> registration_unit_factor;
    std::optional<Json> mapping_model_units;
    std::optional<double> mapping_reference_scale;
    std::optional<Json> registration_model_units;
};
// Resolves the unit branch from one selected model's units. Nonzero scale modes
// except ElevationDrape require the actual reference-chain scale (including a
// known value of 1 for an unscaled context); missing context is not guessed.
Json material_uv_mapping_unit_factor(const Json &units, std::int32_t mapping_mode,
                                     std::int32_t scale_mode,
                                     const std::optional<double> &reference_scale = std::nullopt);
// Consumes one version_conversion.layer_mapping_getters entry. Computes the
// 2D affine transform including the selected registration unit branch.
Json build_material_uv_transform(const Json &layer_getter,
                                 const MaterialUvTransformContext &context);
// Explicit render inputs, not inferred from world bounds. reference_transform
// is an affine row matrix acting on reference_point. uv_transform is the
// registered 2D transform, before the render geometry scale is applied.
struct MaterialProjectionRenderContext {
    std::optional<std::int32_t> geometry_kind;
    std::optional<Matrix4> reference_transform;
    std::optional<Point3> reference_point;
    std::optional<Matrix2x3> uv_transform;
    std::optional<double> geometry_scale;
};
// Consumes resolve_material_projection_transform output. The supported render
// branch requires nonzero geometry_kind; mode 6 retains native cap selection.
Json prepare_material_projection_sampling(const Json &resolved,
                                          const MaterialProjectionRenderContext &context);
// Point and normal are in the render vertex coordinate system and are rounded
// to float as in native buffers. A normal is required only for cubic/solid and
// capped cylindrical mapping. Reuse prepared state for any number of points.
Json sample_material_projection(const Json &prepared, const Point3 &point,
                                const std::optional<Point3> &normal = std::nullopt);
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
    // Native curve derivative convention: result[0] is the point; result[k]
    // differentiates with respect to the source knot parameter, not fraction.
    // Open fractions clamp; closed fractions wrap (exact upper endpoint stays).
    // Supports native orders 2..26 and derivative orders 0..24. Throws on
    // native knot-tolerance failure, zero control/evaluated weight or overflow.
    std::vector<Point3> native_derivatives_at(double fraction, unsigned derivative_order = 3) const;
    // Native single-curve Frenet frame, including polygon/axis fallbacks.
    // The report distinguishes a computed frame from a nondegenerate basis.
    Json native_frame_at(double fraction) const;

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
struct BsplineMeshOptions {
    double uv_tolerance = 1e-6;
    double max_uv_edge = 1.0 / 16;
    unsigned max_trim_segments = 100000;
    unsigned max_vertices = 200000;
    unsigned max_triangles = 400000;
};
struct BsplineSurfaceMesh {
    std::vector<Point3> vertices;
    std::vector<Point2> parameters;
    std::vector<Triangle> faces;
    Json report;
};
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
    // Mesh the derived parity region clipped to [0,1]^2, with conforming UV
    // edge refinement. max_uv_edge is not a bound on world-space chord error.
    // Incomplete conversion returns no mesh; inspect report before use.
    BsplineSurfaceMesh mesh(const BsplineMeshOptions &options = {}) const;

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
struct LoftCapRegions {
    Json bottom, top; // Derived CurveVector trees; null unless both caps succeed.
    Json report;
};
struct LoftNativeFaces {
    std::vector<LoftSide> sides;
    LoftCapRegions caps;
    Json report;
};
struct LoftFaceIndexSet {
    std::vector<std::array<std::int64_t, 3>> indices;
    Json report;
};
struct LoftMeshOptions {
    double max_uv_edge = 1.0 / 16; // Side triangles only; not a 3D error bound.
    double join_tolerance = 1e-8;  // Absolute distance in geometry coordinates.
    double planarity_tolerance = 1e-8;
    unsigned max_vertices = 200000;
    unsigned max_triangles = 400000;
    unsigned max_cap_control_points = 100000;
};
struct LoftMeshPart {
    std::string role; // side, bottom, top
    std::optional<std::size_t> side_index;
    std::size_t first_face = 0, face_count = 0;
    // Identity within this loft solid, independent of material Entry indices.
    // Empty if the separate native face reconstruction cannot complete.
    std::optional<std::array<std::int64_t, 3>> native_face_indices;
};
struct LoftMesh {
    std::vector<Point3> vertices;
    std::vector<Triangle> faces;
    std::vector<std::optional<std::array<Point2, 3>>> face_parameters;
    std::vector<LoftMeshPart> parts; // Derived ranges, not native material part IDs.
    Json report;
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
    // Native endpoint-isocurve regions and bottom reversal, without meshing,
    // fitting a plane, filling internal gaps, or replacing source profiles.
    LoftCapRegions cap_regions(unsigned max_control_points = 100000) const;
    // Native per-face surfaces, including linear-V cleanup, followed by cap
    // construction from those surfaces. Does not modify sides() or source().
    LoftNativeFaces native_faces(unsigned max_cap_control_points = 100000) const;
    // Native cap coordinate-frame/range rules; does not test trim containment
    // or planarity. Uses numerical rational-Bezier extrema, not sampled bounds.
    Json native_cap_uv(bool top, double u, double v, unsigned max_control_points = 100000) const;
    // Caps first, then sides in source loop/primitive order. A failed requested
    // cap invalidates the entire enumeration. These are not material part IDs.
    LoftFaceIndexSet face_indices(unsigned max_cap_control_points = 100000) const;
    // Consistent side sampling and planar cap triangulation. Shares vertices
    // along known adjacent side boundaries, without source geometry deduplication.
    LoftMesh mesh(const LoftMeshOptions &options = {}) const;
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
struct MaterialCatalogContext {
    // Native ID-index key and context-comparison key can differ after lookup failure.
    std::string index_key, comparison_key;
};
struct MaterialCatalogResource {
    std::string member_name;
    std::uint32_t descriptor_mode = 0;
    // Unset denotes this system container's current resource context.
    std::optional<MaterialCatalogContext> primary_context;
};
struct MaterialCatalogOptions {
    std::optional<MaterialCatalogContext> current_context;
    // Called only when the source reference cannot determine the descriptor.
    // Arguments are the input member identity and its parsed catalog source fields.
    std::function<std::optional<MaterialCatalogResource>(const Json &, const Json &)>
        resolve_resource;
    // UTF-8 strings. Default: native CRT C-locale equality (ASCII case conversion).
    // Supply the relevant host-locale equality when reproducing another locale.
    std::function<bool(const std::string &, const std::string &)> case_equal;
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
    // Persisted MMIx order/flags, before runtime refresh or fallback rebuilding.
    Json native_model_directory() const;
    const Json &objects() const;
    const Json &native_records() const;
    // Conditional block input and trees, initial SSYS IDs, attributes/material table;
    // does not instantiate host runtime objects or evaluate later host events.
    Json native_input_containers() const;
    // Fresh model, loading control then graphics into a shared empty ID index.
    // Supply the actual counter at this load point, not the original file value
    // after intervening loads. Does not execute host callbacks or attach attributes.
    Json native_model_id_assignments(const StreamPath &model_storage,
                                     std::uint64_t initial_id_counter) const;
    // Lookup in the initial model registry, then the system registry on miss.
    // Shares the explicit counter restrictions above. Does not apply runtime
    // deletion filters, expand owner paths, or scan unrelated models.
    Json native_model_object_lookup(const StreamPath &model_storage,
                                    std::uint64_t initial_id_counter, std::uint64_t id) const;
    // Expand owner-reference paths in the fresh input graph, before runtime
    // callbacks. Retains parent occurrence identity and prepared child flags.
    // Type-13 model attachment/loading is not inferred from source IDs.
    Json native_model_reference_path(const StreamPath &model_storage,
                                     std::uint64_t initial_id_counter, std::uint64_t id) const;
    // Conditional initial SMC/SMCA then SMG/SMGA input into a fresh model.
    // Preserves collection state and uses the registry available at each step.
    // Shares the counter/context restrictions of native_model_id_assignments.
    Json native_model_attribute_input(const StreamPath &model_storage,
                                      std::uint64_t initial_id_counter) const;
    // Initial registration of each system table into an empty material catalog.
    // External resources are never searched automatically.
    Json native_material_catalog(const MaterialCatalogOptions & = {}) const;
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
