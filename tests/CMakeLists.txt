if(NOT TARGET gtest)
  add_subdirectory(
    "${PROJECT_SOURCE_DIR}/third_party/fmt/test/gtest"
    "${CMAKE_BINARY_DIR}/third_party/fmt_gtest"
    EXCLUDE_FROM_ALL
  )
endif()

set(NBODY_ORBIT_PREDICTION_POLICY_SOURCES
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/policy_integrator.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/policy_chunking.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/policy_profile.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/policy_adaptive.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/policy_ephemeris.cpp"
)

set(NBODY_ORBIT_PREDICTION_SERVICE_SOURCES
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/trajectory_utils.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/service.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/compute.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/routes.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/spacecraft.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/planned_route.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/stage_publish.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/planned_cache.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/planned_maneuver.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/chunk_attempt.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/planned.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/trajectory.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/sampling.cpp"
  ${NBODY_ORBIT_PREDICTION_POLICY_SOURCES}
)

set(NBODY_ORBIT_RENDER_CURVE_SOURCES
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_prediction_math.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve_render.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/render_curve/orbit_render_curve_pick.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_plot_util.cpp"
)

set(GAMEPLAY_NBODY_MANEUVER_SYSTEM_SOURCES
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver_nbody/maneuver_plan_model.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver_nbody/maneuver_commands.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver_nbody/maneuver_controller.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver_nbody/maneuver_system.cpp"
)

set(GAMEPLAY_NBODY_MANEUVER_RUNTIME_SOURCES
  ${GAMEPLAY_NBODY_MANEUVER_SYSTEM_SOURCES}
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver_nbody/maneuver_prediction_bridge.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver_nbody/maneuver_runtime_cache_builder.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver_nbody/gameplay_state_maneuver_nodes.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver_nbody/gameplay_state_maneuver_nodes_cache.cpp"
)

set(GAMEPLAY_NBODY_PREDICTION_SOURCES
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/gameplay_state_prediction.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/prediction_frame_context_builder.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/prediction_host_context_builder.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/prediction_subject_state_provider.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/runtime/gameplay_state_prediction_runtime.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/runtime/prediction_window_context_builder.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/runtime/gameplay_state_prediction_runtime_solver.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/runtime/prediction_lifecycle_reducer.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/runtime/prediction_request_factory.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/runtime/prediction_invalidation_controller.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/runtime/prediction_maneuver_invalidation.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/runtime/prediction_result_applier.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/runtime/prediction_solver_result_applier.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/runtime/prediction_runtime_controller.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/gameplay_prediction_state.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/prediction_system.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/runtime/gameplay_state_prediction_runtime_requests.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/runtime/gameplay_state_prediction_runtime_results.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/gameplay_state_prediction_frames.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/prediction_frame_resolver.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/prediction_frame_controller.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/draw/gameplay_state_prediction_draw_helpers.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/draw/gameplay_state_prediction_draw_prepare.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/draw/gameplay_state_prediction_draw_render_helpers.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/draw/gameplay_state_prediction_draw_shared.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/draw/gameplay_state_prediction_draw_track.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/gameplay_prediction_cache_internal.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/prediction_trajectory_sampler.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/prediction_frame_cache_builder.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/prediction_metrics_builder.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/nbody/chunk_assembly.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/gameplay_prediction_derived_service.cpp"
)

set(GAMEPLAY_NBODY_PREDICTION_CACHE_SOURCES
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/gameplay_prediction_cache_internal.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/prediction_frame_cache_builder.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/prediction_metrics_builder.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/nbody/chunk_assembly.cpp"
)

set(GAME_ORBIT_KEPLER_SOURCES
  "${PROJECT_SOURCE_DIR}/src/game/orbit/kepler/kepler_celestial_nbody.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/kepler/kepler_primary_resolver.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/kepler/kepler_arc_builder.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/kepler/kepler_arc_line_builder.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/kepler/kepler_arc_info.cpp"
)

set(GAMEPLAY_KEPLER_MANEUVER_SYSTEM_SOURCES
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver_kepler/kepler_maneuver_plan.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver_kepler/kepler_maneuver_node_resolver.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver_kepler/kepler_maneuver_orbit_pick.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver_kepler/kepler_maneuver_commands.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/maneuver_kepler/kepler_maneuver_system.cpp"
)

set(GAMEPLAY_KEPLER_PREDICTION_SOURCES
  ${GAMEPLAY_KEPLER_MANEUVER_SYSTEM_SOURCES}
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/gameplay_state_orbit_prediction.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_kepler/gameplay_state_kepler_prediction.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_kepler/kepler_prediction_subject.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_kepler/kepler_prediction_builder.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_kepler/kepler_prediction_draw.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_kepler/kepler_prediction_system.cpp"
)

add_executable(scenario_loader_tests
  scenario_loader_tests.cpp
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/scenario/scenario_loader.cpp"
  "${PROJECT_SOURCE_DIR}/src/core/util/logger.cpp"
)

set_property(TARGET scenario_loader_tests PROPERTY CXX_STANDARD 20)

target_include_directories(scenario_loader_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

set(SCENARIO_TEST_ASSETS_DIR "${CMAKE_CURRENT_BINARY_DIR}/assets/scenarios")
file(MAKE_DIRECTORY "${SCENARIO_TEST_ASSETS_DIR}")
configure_file(
  "${PROJECT_SOURCE_DIR}/tests/fixtures/scenarios/default_gameplay.json"
  "${SCENARIO_TEST_ASSETS_DIR}/default_gameplay.json"
  COPYONLY
)

target_compile_definitions(scenario_loader_tests PRIVATE
  SCENARIO_TEST_ASSETS_DIR="${SCENARIO_TEST_ASSETS_DIR}"
)

target_link_libraries(scenario_loader_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME scenario_loader_tests COMMAND scenario_loader_tests)

add_executable(planet_quadtree_tests
  planet_quadtree_tests.cpp
  "${PROJECT_SOURCE_DIR}/src/scene/planet/planet_heightmap.cpp"
  "${PROJECT_SOURCE_DIR}/src/scene/planet/planet_quadtree.cpp"
  "${PROJECT_SOURCE_DIR}/src/scene/planet/cubesphere.cpp"
  "${PROJECT_SOURCE_DIR}/src/scene/tangent_space.cpp"
)

set_property(TARGET planet_quadtree_tests PROPERTY CXX_STANDARD 20)

target_compile_definitions(planet_quadtree_tests PRIVATE
  GLM_FORCE_DEPTH_ZERO_TO_ONE
  PLANET_HEIGHTMAP_DISABLE_KTX_IO=1
)

target_include_directories(planet_quadtree_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(planet_quadtree_tests PRIVATE
  gtest
  fmt::fmt
  glm
)

add_test(NAME planet_quadtree_tests COMMAND planet_quadtree_tests)

add_executable(orbit_helpers_tests
  orbit_helpers_tests.cpp
)

set_property(TARGET orbit_helpers_tests PROPERTY CXX_STANDARD 20)

target_include_directories(orbit_helpers_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(orbit_helpers_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME orbit_helpers_tests COMMAND orbit_helpers_tests)

add_executable(kepler_orbit_tests
  kepler_orbit_tests.cpp
  ${GAME_ORBIT_KEPLER_SOURCES}
)

set_property(TARGET kepler_orbit_tests PROPERTY CXX_STANDARD 20)

target_include_directories(kepler_orbit_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(kepler_orbit_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME kepler_orbit_tests COMMAND kepler_orbit_tests)

add_executable(kepler_prediction_tests
  kepler_prediction_tests.cpp
  ${GAME_ORBIT_KEPLER_SOURCES}
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_kepler/kepler_prediction_builder.cpp"
)

set_property(TARGET kepler_prediction_tests PROPERTY CXX_STANDARD 20)

target_include_directories(kepler_prediction_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(kepler_prediction_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME kepler_prediction_tests COMMAND kepler_prediction_tests)

add_executable(kepler_maneuver_tests
  kepler_maneuver_tests.cpp
  ${GAMEPLAY_KEPLER_MANEUVER_SYSTEM_SOURCES}
)

set_property(TARGET kepler_maneuver_tests PROPERTY CXX_STANDARD 20)

target_include_directories(kepler_maneuver_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(kepler_maneuver_tests PRIVATE
  gtest
  orbitsim
)

add_test(NAME kepler_maneuver_tests COMMAND kepler_maneuver_tests)

add_executable(orbit_prediction_planner_tests
  orbit_prediction_planner_tests.cpp
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_prediction_math.cpp"
  ${NBODY_ORBIT_PREDICTION_POLICY_SOURCES}
)

set_property(TARGET orbit_prediction_planner_tests PROPERTY CXX_STANDARD 20)

target_include_directories(orbit_prediction_planner_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(orbit_prediction_planner_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME orbit_prediction_planner_tests COMMAND orbit_prediction_planner_tests)

add_executable(orbit_render_curve_tests
  orbit_render_curve_tests.cpp
  ${NBODY_ORBIT_RENDER_CURVE_SOURCES}
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_prediction_math.cpp"
)

set_property(TARGET orbit_render_curve_tests PROPERTY CXX_STANDARD 20)

target_include_directories(orbit_render_curve_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(orbit_render_curve_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME orbit_render_curve_tests COMMAND orbit_render_curve_tests)

add_executable(time_warp_state_tests
  time_warp_state_tests.cpp
)

set_property(TARGET time_warp_state_tests PROPERTY CXX_STANDARD 20)

target_include_directories(time_warp_state_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(time_warp_state_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME time_warp_state_tests COMMAND time_warp_state_tests)

add_executable(gameplay_time_warp_transition_tests
  gameplay_time_warp_transition_tests.cpp
  gameplay_state_test_stubs.cpp
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/orbital/orbital_runtime_system.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/orbital/orbiter_state_bridge.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/orbital/formation_hold_system.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/orbital/orbital_physics_system.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/gameplay_state_sim.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/prediction_nbody/gameplay_prediction_state.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_prediction_math.cpp"
  ${NBODY_ORBIT_RENDER_CURVE_SOURCES}
  ${NBODY_ORBIT_PREDICTION_SERVICE_SOURCES}
)

set_property(TARGET gameplay_time_warp_transition_tests PROPERTY CXX_STANDARD 20)

target_compile_definitions(gameplay_time_warp_transition_tests PRIVATE
  VULKAN_ENGINE_USE_JOLT=0
  VULKAN_ENGINE_GAMEPLAY_TEST_ACCESS=1
)

target_include_directories(gameplay_time_warp_transition_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(gameplay_time_warp_transition_tests PRIVATE
  gtest
  fmt::fmt
  glm
  orbitsim
)

add_test(NAME gameplay_time_warp_transition_tests COMMAND gameplay_time_warp_transition_tests)

add_executable(maneuver_system_tests
  maneuver_system_tests.cpp
  ${GAMEPLAY_NBODY_MANEUVER_SYSTEM_SOURCES}
)

set_property(TARGET maneuver_system_tests PROPERTY CXX_STANDARD 20)

target_include_directories(maneuver_system_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(maneuver_system_tests PRIVATE
  gtest
  glm
  orbitsim
)

add_test(NAME maneuver_system_tests COMMAND maneuver_system_tests)

add_executable(gameplay_prediction_maneuver_tests
  nbody/gameplay_prediction_maneuver_runtime_window_tests.cpp
  nbody/gameplay_prediction_maneuver_lifecycle_tests.cpp
  nbody/gameplay_prediction_maneuver_service_tests.cpp
  nbody/gameplay_prediction_maneuver_runtime_results_tests.cpp
  nbody/gameplay_prediction_maneuver_derived_results_tests.cpp
  nbody/gameplay_prediction_maneuver_test_stubs.cpp
  "${PROJECT_SOURCE_DIR}/src/game/entity.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_prediction_math.cpp"
  ${NBODY_ORBIT_RENDER_CURVE_SOURCES}
  "${PROJECT_SOURCE_DIR}/src/core/orbit_plot/orbit_plot.cpp"
  ${NBODY_ORBIT_PREDICTION_SERVICE_SOURCES}
  ${GAME_ORBIT_KEPLER_SOURCES}
  ${GAMEPLAY_KEPLER_PREDICTION_SOURCES}
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/gameplay_state_sim.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/orbital/orbital_physics_system.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/orbital/orbiter_state_bridge.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/orbital/orbital_runtime_system.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/states/gameplay/orbital/formation_hold_system.cpp"
  ${GAMEPLAY_NBODY_PREDICTION_SOURCES}
  ${GAMEPLAY_NBODY_MANEUVER_RUNTIME_SOURCES}
  "${PROJECT_SOURCE_DIR}/src/core/util/logger.cpp"
  "${PROJECT_SOURCE_DIR}/third_party/imgui/imgui.cpp"
  "${PROJECT_SOURCE_DIR}/third_party/imgui/imgui_draw.cpp"
  "${PROJECT_SOURCE_DIR}/third_party/imgui/imgui_tables.cpp"
  "${PROJECT_SOURCE_DIR}/third_party/imgui/imgui_widgets.cpp"
)

set_property(TARGET gameplay_prediction_maneuver_tests PROPERTY CXX_STANDARD 20)

target_compile_definitions(gameplay_prediction_maneuver_tests PRIVATE
  VULKAN_ENGINE_USE_JOLT=0
  VULKAN_ENGINE_GAMEPLAY_TEST_ACCESS=1
)

target_include_directories(gameplay_prediction_maneuver_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/src/core"
  "${PROJECT_SOURCE_DIR}/third_party/imgui"
  "${PROJECT_SOURCE_DIR}/third_party/vkbootstrap"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(gameplay_prediction_maneuver_tests PRIVATE
  gtest
  fmt::fmt
  glm
  Vulkan::Vulkan
  SDL2::SDL2
  orbitsim
)

add_test(NAME gameplay_prediction_maneuver_tests COMMAND gameplay_prediction_maneuver_tests)

add_executable(prediction_cache_internal_tests
  prediction_cache_internal_tests.cpp
  "${PROJECT_SOURCE_DIR}/src/game/orbit/orbit_prediction_math.cpp"
  ${NBODY_ORBIT_RENDER_CURVE_SOURCES}
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/trajectory.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/prediction/sampling.cpp"
  "${PROJECT_SOURCE_DIR}/src/game/orbit/nbody/trajectory_utils.cpp"
  ${GAMEPLAY_NBODY_PREDICTION_CACHE_SOURCES}
)

set_property(TARGET prediction_cache_internal_tests PROPERTY CXX_STANDARD 20)

target_include_directories(prediction_cache_internal_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src"
  "${PROJECT_SOURCE_DIR}/third_party/vma"
  "${PROJECT_SOURCE_DIR}/third_party/taskflow/3rd-party"
)

target_link_libraries(prediction_cache_internal_tests PRIVATE
  gtest
  fmt::fmt
  glm
  Vulkan::Vulkan
  orbitsim
)

add_test(NAME prediction_cache_internal_tests COMMAND prediction_cache_internal_tests)
