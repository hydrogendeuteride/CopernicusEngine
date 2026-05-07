#include "game/states/gameplay/gameplay_state.h"

#include "game/states/gameplay/prediction_nbody/gameplay_prediction_adapter.h"
#include "game/states/gameplay/prediction_nbody/prediction_host_context_builder.h"
#include "game/states/gameplay/prediction_nbody/prediction_system.h"

#include <vector>

namespace Game
{
    bool GameplayState::spacecraft_orbit_prediction_uses_kepler() const
    {
        return _spacecraft_orbit_prediction_mode == SpacecraftOrbitPredictionMode::Kepler;
    }

    bool GameplayState::spacecraft_orbit_prediction_uses_nbody() const
    {
        return _spacecraft_orbit_prediction_mode == SpacecraftOrbitPredictionMode::NBody;
    }

    void GameplayState::set_spacecraft_orbit_prediction_mode(const SpacecraftOrbitPredictionMode mode)
    {
        if (_spacecraft_orbit_prediction_mode == mode)
        {
            return;
        }

        _spacecraft_orbit_prediction_mode = mode;
        if (spacecraft_orbit_prediction_uses_kepler())
        {
            _show_nbody_orbit_debug = false;
            _show_maneuver_nodes_panel = false;
        }
        else
        {
            _show_kepler_orbit_debug = false;
            clear_kepler_prediction_runtime();

            _prediction->sync_subjects(PredictionHostContextBuilder(build_prediction_context()).build());
            const EntityId player = _orbit.player_entity();
            if (player.is_valid())
            {
                _prediction->state().selection.active_subject =
                        PredictionSubjectKey{PredictionSubjectKind::Orbiter, player.value};
            }

            _prediction->state().selection.overlay_subjects.clear();
            for (const PredictionTrackState &track : _prediction->state().tracks)
            {
                if (track.key != _prediction->state().selection.active_subject)
                {
                    _prediction->state().selection.overlay_subjects.push_back(track.key);
                }
            }
            _prediction->state().selection.selected_group_index = -1;
        }

        mark_prediction_dirty();
    }

    void GameplayState::mark_prediction_dirty()
    {
        GameplayPredictionAdapter prediction(build_prediction_access());
        const std::vector<PredictionSubjectKey> visible_subjects = prediction.collect_visible_prediction_subjects();
        _prediction->mark_visible_tracks_dirty(visible_subjects);
        mark_kepler_prediction_dirty();
        prediction.sync_prediction_dirty_flag();
    }

    void GameplayState::clear_prediction_runtime()
    {
        _prediction->clear_runtime();
        clear_kepler_prediction_runtime();
    }

    void GameplayState::update_prediction(GameStateContext &ctx, float fixed_dt)
    {
        GameplayPredictionAdapter prediction(build_prediction_access());
        prediction.rebuild_prediction_frame_options();
        prediction.rebuild_prediction_analysis_options();

        if (spacecraft_orbit_prediction_uses_kepler())
        {
            update_kepler_prediction(ctx);
            return;
        }

        PredictionHostContext nbody_host = PredictionHostContextBuilder(build_prediction_context()).build(&ctx);
        _prediction->update(nbody_host, fixed_dt);
        update_kepler_prediction(ctx);
    }

    void GameplayState::draw_prediction(GameStateContext &ctx)
    {
        if (spacecraft_orbit_prediction_uses_kepler())
        {
            draw_kepler_prediction(ctx);
            return;
        }

        GameplayPredictionAdapter prediction(build_prediction_access());
        prediction.poll_completed_prediction_results();
        prediction.emit_orbit_prediction_debug(ctx);
    }
} // namespace Game
