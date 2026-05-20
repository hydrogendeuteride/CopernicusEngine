#include "game/orbit/kepler/kepler_draw_lod_line_builder.h"

#include <glm/geometric.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace Game
{
    namespace
    {
        constexpr int kMinSplitDepth = 3;
        constexpr int kMaxSplitDepth = 24;
        constexpr double kProjectionWMin = 1.0e-9;
        constexpr double kPixelEpsilon = 1.0e-6;

        struct CachedBodyStateQuery
        {
            orbitsim::BodyId body_id{orbitsim::kInvalidBodyId};
            bool ephemeris_index_valid{false};
            std::size_t ephemeris_index{0u};
        };

        struct DrawLodSample
        {
            bool ok{false};
            KeplerArcLineVertex vertex{};
            orbitsim::KeplerStatus status{orbitsim::KeplerStatus::Ok};
        };

        struct ProjectedPoint
        {
            bool valid{false};
            glm::dvec2 px{0.0, 0.0};
        };

        CachedBodyStateQuery make_cached_body_state_query(const KeplerDrawLodLineBuildRequest &request,
                                                          const orbitsim::BodyId body_id)
        {
            CachedBodyStateQuery query{};
            query.body_id = body_id;
            if (request.ephemeris && !request.ephemeris->empty())
            {
                query.ephemeris_index_valid =
                        request.ephemeris->body_index_for_id(body_id, &query.ephemeris_index);
            }
            return query;
        }

        bool state_at_body(const KeplerDrawLodLineBuildRequest &request,
                           const CachedBodyStateQuery &query,
                           const double t_s,
                           orbitsim::State &out_state)
        {
            if (query.body_id == orbitsim::kInvalidBodyId)
            {
                return false;
            }

            if (request.ephemeris && !request.ephemeris->empty() && query.ephemeris_index_valid)
            {
                const orbitsim::State ephemeris_state =
                        request.ephemeris->body_state_at(query.ephemeris_index, t_s);
                if (kepler_finite_vec3(ephemeris_state.position_m))
                {
                    out_state = ephemeris_state;
                    return true;
                }
            }

            if (request.body_state_provider.state_at &&
                request.body_state_provider.state_at(query.body_id, t_s, out_state))
            {
                return kepler_finite_vec3(out_state.position_m);
            }
            return false;
        }

        bool finite_world(const WorldVec3 &p)
        {
            return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        }

        DrawLodSample sample_draw_lod_vertex(const KeplerDrawLodLineBuildRequest &request,
                                             const KeplerOrbitArc &game_arc,
                                             const CachedBodyStateQuery &primary_query,
                                             const CachedBodyStateQuery &reference_query,
                                             const bool reference_is_primary,
                                             const double t_s)
        {
            DrawLodSample out{};
            const orbitsim::KeplerArcSample sample =
                    orbitsim::sample_kepler_arc_state(game_arc.arc,
                                                       t_s,
                                                       request.line_options.propagation);
            out.status = sample.diagnostics.status;
            if (!sample.ok() || !kepler_finite_vec3(sample.state_relative.position_m))
            {
                return out;
            }

            orbitsim::State primary_state = game_arc.primary_state_inertial_at_t0;
            orbitsim::State moving_primary_state{};
            bool have_moving_primary = false;
            if (state_at_body(request, primary_query, sample.t_s, moving_primary_state))
            {
                primary_state = moving_primary_state;
                have_moving_primary = true;
            }

            orbitsim::State reference_state = request.world_frame.world_reference_state_inertial;
            if (request.world_frame.world_reference_body_id != orbitsim::kInvalidBodyId)
            {
                if (reference_is_primary && have_moving_primary)
                {
                    reference_state = primary_state;
                }
                else
                {
                    orbitsim::State moving_reference_state{};
                    if (state_at_body(request, reference_query, sample.t_s, moving_reference_state))
                    {
                        reference_state = moving_reference_state;
                    }
                }
            }

            if (!kepler_finite_vec3(primary_state.position_m) ||
                !kepler_finite_vec3(reference_state.position_m))
            {
                out.status = orbitsim::KeplerStatus::InvalidInitialState;
                return out;
            }

            const orbitsim::Vec3 inertial_position_m =
                    primary_state.position_m + sample.state_relative.position_m;
            const WorldVec3 position_world =
                    request.world_frame.world_reference_body_world +
                    WorldVec3(inertial_position_m - reference_state.position_m);
            if (!finite_world(position_world))
            {
                out.status = orbitsim::KeplerStatus::InvalidInitialState;
                return out;
            }

            out.ok = true;
            out.vertex.position_world = position_world;
            out.vertex.t_s = sample.t_s;
            out.vertex.primary_body_id = game_arc.arc.primary_body_id;
            return out;
        }

        ProjectedPoint project_world(const KeplerDrawLodLineBuildRequest &request,
                                     const WorldVec3 &world)
        {
            ProjectedPoint out{};
            if (!(request.viewport_width_px > 0.0) ||
                !(request.viewport_height_px > 0.0) ||
                !std::isfinite(request.viewport_width_px) ||
                !std::isfinite(request.viewport_height_px))
            {
                return out;
            }

            const glm::dvec3 local = world_to_local_d(world, request.world_origin);
            const glm::dvec4 clip = glm::dmat4(request.viewproj) * glm::dvec4(local, 1.0);
            if (!(std::abs(clip.w) > kProjectionWMin) || !std::isfinite(clip.w))
            {
                return out;
            }

            const glm::dvec2 ndc = glm::dvec2(clip) / clip.w;
            if (!std::isfinite(ndc.x) || !std::isfinite(ndc.y))
            {
                return out;
            }

            out.valid = true;
            out.px = (ndc * 0.5 + glm::dvec2(0.5)) *
                     glm::dvec2(request.viewport_width_px, request.viewport_height_px);
            return out;
        }

        double point_segment_distance_px(const glm::dvec2 &p,
                                         const glm::dvec2 &a,
                                         const glm::dvec2 &b)
        {
            const glm::dvec2 ab = b - a;
            const double ab_len2 = glm::dot(ab, ab);
            if (!(ab_len2 > kPixelEpsilon) || !std::isfinite(ab_len2))
            {
                const double dist = glm::length(p - a);
                return std::isfinite(dist) ? dist : 0.0;
            }

            const double t = std::clamp(glm::dot(p - a, ab) / ab_len2, 0.0, 1.0);
            const double dist = glm::length(p - (a + t * ab));
            return std::isfinite(dist) ? dist : 0.0;
        }

        double screen_error_px(const KeplerDrawLodLineBuildRequest &request,
                               const DrawLodSample &a,
                               const DrawLodSample &mid,
                               const DrawLodSample &b)
        {
            const ProjectedPoint pa = project_world(request, a.vertex.position_world);
            const ProjectedPoint pm = project_world(request, mid.vertex.position_world);
            const ProjectedPoint pb = project_world(request, b.vertex.position_world);
            if (!pa.valid || !pm.valid || !pb.valid)
            {
                return 0.0;
            }
            return point_segment_distance_px(pm.px, pa.px, pb.px);
        }

        uint32_t endpoint_flags(const std::size_t arc_index,
                                const std::size_t arc_count,
                                const bool start)
        {
            uint32_t flags = start
                                     ? static_cast<uint32_t>(KeplerArcLineVertexFlags::ArcStart)
                                     : static_cast<uint32_t>(KeplerArcLineVertexFlags::ArcEnd);
            if (start && arc_index == 0u)
            {
                flags |= static_cast<uint32_t>(KeplerArcLineVertexFlags::OrbitStart);
            }
            if (!start && arc_index + 1u == arc_count)
            {
                flags |= static_cast<uint32_t>(KeplerArcLineVertexFlags::OrbitEnd);
            }
            return flags;
        }

        void append_vertex(KeplerArcLineSet &out,
                           KeplerArcLineVertex vertex)
        {
            if (!out.vertices.empty() && kepler_same_sample_time(out.vertices.back().t_s, vertex.t_s))
            {
                out.vertices.back().flags |= vertex.flags;
                return;
            }
            out.vertices.push_back(vertex);
        }

        bool emit_lod_interval(const KeplerDrawLodLineBuildRequest &request,
                               const KeplerOrbitArc &arc,
                               const CachedBodyStateQuery &primary_query,
                               const CachedBodyStateQuery &reference_query,
                               const bool reference_is_primary,
                               const DrawLodSample &a,
                               const DrawLodSample &b,
                               const int depth,
                               KeplerArcLineSet &out)
        {
            const double min_dt_s = kepler_positive_or_default(request.line_options.min_time_step_s, 1.0);
            const double dt_s = std::abs(b.vertex.t_s - a.vertex.t_s);
            const bool can_split =
                    depth < kMaxSplitDepth &&
                    dt_s > min_dt_s &&
                    out.vertices.size() + 1u < request.line_options.max_vertices_total;
            if (can_split)
            {
                const double mid_t_s = 0.5 * (a.vertex.t_s + b.vertex.t_s);
                const DrawLodSample mid = sample_draw_lod_vertex(request,
                                                                 arc,
                                                                 primary_query,
                                                                 reference_query,
                                                                 reference_is_primary,
                                                                 mid_t_s);
                ++out.diagnostics.requested_samples;
                if (mid.ok)
                {
                    ++out.diagnostics.accepted_samples;
                    const double error_px = screen_error_px(request, a, mid, b);
                    const double target_error_px =
                            kepler_positive_or_default(request.render_error_px, 0.75);
                    if (depth < kMinSplitDepth || error_px > target_error_px)
                    {
                        if (!emit_lod_interval(request,
                                               arc,
                                               primary_query,
                                               reference_query,
                                               reference_is_primary,
                                               a,
                                               mid,
                                               depth + 1,
                                               out))
                        {
                            return false;
                        }
                        return emit_lod_interval(request,
                                                 arc,
                                                 primary_query,
                                                 reference_query,
                                                 reference_is_primary,
                                                 mid,
                                                 b,
                                                 depth + 1,
                                                 out);
                    }
                }
                else if (out.diagnostics.first_kepler_failure == orbitsim::KeplerStatus::Ok)
                {
                    out.diagnostics.first_kepler_failure = mid.status;
                }
            }
            else if (depth < kMaxSplitDepth && dt_s > min_dt_s)
            {
                out.diagnostics.budget_hit = true;
            }

            if (out.vertices.size() >= request.line_options.max_vertices_total)
            {
                out.diagnostics.budget_hit = true;
                return true;
            }
            append_vertex(out, b.vertex);
            return true;
        }

        bool build_arc_draw_lod_lines(const KeplerDrawLodLineBuildRequest &request,
                                      const KeplerOrbitArc &arc,
                                      const std::size_t arc_index,
                                      KeplerArcLineSet &out)
        {
            const CachedBodyStateQuery primary_query =
                    make_cached_body_state_query(request, arc.arc.primary_body_id);
            const CachedBodyStateQuery reference_query =
                    make_cached_body_state_query(request,
                                                 request.world_frame.world_reference_body_id);
            const bool reference_is_primary =
                    request.world_frame.world_reference_body_id == arc.arc.primary_body_id;

            DrawLodSample start = sample_draw_lod_vertex(request,
                                                         arc,
                                                         primary_query,
                                                         reference_query,
                                                         reference_is_primary,
                                                         arc.arc.t0_s);
            ++out.diagnostics.requested_samples;
            if (!start.ok)
            {
                out.diagnostics.status = KeplerOrbitStatus::PropagationFailed;
                out.diagnostics.failed_arc_index = arc_index;
                out.diagnostics.first_kepler_failure = start.status;
                return false;
            }
            ++out.diagnostics.accepted_samples;
            start.vertex.flags |= endpoint_flags(arc_index, request.arcs.size(), true);
            if (request.line_options.include_start || out.vertices.empty())
            {
                append_vertex(out, start.vertex);
            }

            if (out.vertices.size() >= request.line_options.max_vertices_total)
            {
                out.diagnostics.budget_hit = true;
                return true;
            }

            DrawLodSample end = sample_draw_lod_vertex(request,
                                                       arc,
                                                       primary_query,
                                                       reference_query,
                                                       reference_is_primary,
                                                       arc.arc.t1_s);
            ++out.diagnostics.requested_samples;
            if (!end.ok)
            {
                out.diagnostics.status = KeplerOrbitStatus::PropagationFailed;
                out.diagnostics.failed_arc_index = arc_index;
                if (out.diagnostics.first_kepler_failure == orbitsim::KeplerStatus::Ok)
                {
                    out.diagnostics.first_kepler_failure = end.status;
                }
                return false;
            }
            ++out.diagnostics.accepted_samples;
            end.vertex.flags |= endpoint_flags(arc_index, request.arcs.size(), false);
            if (!request.line_options.include_end && arc_index + 1u == request.arcs.size())
            {
                return true;
            }

            return emit_lod_interval(request,
                                     arc,
                                     primary_query,
                                     reference_query,
                                     reference_is_primary,
                                     start,
                                     end,
                                     0,
                                     out);
        }
    } // namespace

    KeplerArcLineSet build_kepler_draw_lod_lines(const KeplerDrawLodLineBuildRequest &request)
    {
        KeplerArcLineSet out{};
        out.diagnostics.requested_arcs = request.arcs.size();
        out.diagnostics.status = KeplerOrbitStatus::Ok;
        if (request.arcs.empty() || request.line_options.max_vertices_total < 2u)
        {
            out.diagnostics.status = KeplerOrbitStatus::InvalidInput;
            return out;
        }

        out.vertices.reserve(std::min<std::size_t>(request.line_options.max_vertices_total, 256u));
        for (std::size_t i = 0u; i < request.arcs.size(); ++i)
        {
            if (out.vertices.size() >= request.line_options.max_vertices_total)
            {
                out.diagnostics.budget_hit = true;
                break;
            }
            if (!orbitsim::kepler_arc_valid(request.arcs[i].arc))
            {
                out.diagnostics.status = KeplerOrbitStatus::InvalidArc;
                out.diagnostics.failed_arc_index = i;
                return out;
            }
            if (!build_arc_draw_lod_lines(request, request.arcs[i], i, out))
            {
                return out;
            }
            ++out.diagnostics.sampled_arcs;
        }

        if (out.vertices.size() < 2u)
        {
            out.diagnostics.status = KeplerOrbitStatus::NoSamples;
            return out;
        }

        out.valid = true;
        return out;
    }
} // namespace Game
