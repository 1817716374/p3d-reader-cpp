# The pinned upstream core, built into the existing static library. No downloads,
# bindings, test frameworks or runtime DLLs are required by this integration.
set(p3d_manifold_sources
 boolean2.cpp boolean2_diagnostics.cpp boolean2_offset.cpp boolean2_predicates.cpp
 boolean2_sweep.cpp boolean3.cpp boolean_result.cpp constructors.cpp cross_section.cpp
 csg_tree.cpp edge_op.cpp execution_impl.cpp face_op.cpp impl.cpp manifold.cpp
 minkowski.cpp polygon.cpp properties.cpp quickhull.cpp sdf.cpp smoothing.cpp
 sort.cpp subdivision.cpp tree2d.cpp)
list(TRANSFORM p3d_manifold_sources PREPEND "${CMAKE_CURRENT_LIST_DIR}/src/")
target_sources(p3d_reader PRIVATE ${p3d_manifold_sources})
target_include_directories(p3d_reader PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include")
# Keep the bundled implementation's symbols distinct from an application's own
# Manifold version. The public p3d API exposes no Manifold types.
set_source_files_properties(${p3d_manifold_sources} "${PROJECT_SOURCE_DIR}/src/csg_mesh.cpp"
 PROPERTIES COMPILE_DEFINITIONS "MANIFOLD_PAR=-1;manifold=p3d_bundled_manifold;linalg=p3d_bundled_linalg")
if(MSVC)
 set_source_files_properties(${p3d_manifold_sources} PROPERTIES COMPILE_OPTIONS "/bigobj;/fp:precise")
else()
 set_source_files_properties(${p3d_manifold_sources} PROPERTIES COMPILE_OPTIONS "-ffp-contract=off;-Wno-unused;-Wno-sign-compare")
endif()
