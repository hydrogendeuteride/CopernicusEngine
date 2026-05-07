네. 최종 계획은 **기존 prediction / maneuver / orbit render curve 계열은 n체 전용으로 격리하고, Kepler 쪽은 새로 만드는 것**으로 잡는 게 맞아요.

그리고 질문하신 `OrbitRenderCurve`는 결론적으로 이렇게 판단해요.

```txt
n체 궤도: 기존 OrbitRenderCurve 유지
Kepler 궤도: OrbitRenderCurve 사용하지 않음
```

Kepler에는 `OrbitRenderCurve`가 필요 없어요. 현재 `OrbitRenderCurve`는 Hermite trajectory segment를 받아 persistent binary-tree LOD를 만들고, 다시 render/pick line으로 풀어내는 꽤 큰 구조예요. 기존 README에서도 tree LOD selection, final line generation, pick downsample까지 맡는다고 설명되어 있어요.  Kepler 궤도는 analytic conic을 바로 샘플링할 수 있으니, 이런 복잡한 LOD 캐시를 끌고 오면 배보다 잎사귀 장식이 커져요.

Kepler 쪽은 대신 **작은 전용 tessellator**를 만들면 돼요.

```txt
KeplerArc
  -> KeplerOrbitTessellator
  -> polyline vertices
  -> draw
  -> same polyline으로 pick
```

즉 Kepler 렌더링은 `TrajectorySegment -> OrbitRenderCurve -> LOD -> line`이 아니라, `KeplerArc -> sampled line`으로 끝내요.

---

## 1. 최종 구조 원칙

이번 개편의 핵심 원칙은 이거예요.

```txt
1. 기존 n체 시스템은 거의 통째로 nbody 전용으로 이동해요.
2. Kepler 시스템은 새로 만들어요.
3. 공통 코드는 아주 얇게만 남겨요.
4. 코드 중복은 허용해요.
5. 기존 prediction cache, derived service, frame cache, streaming, render curve는 Kepler에서 쓰지 않아요.
6. 천체는 n체, 우주선은 Kepler라는 경계를 코드 구조에 박아 넣어요.
```

현재 `gameplay/prediction`은 solver, derived service, frame transforms, drawing, picking, runtime orchestration을 모두 포함하는 구조예요. README에도 solver, derived service, draw의 3층 파이프라인으로 설명되어 있어요.  이건 n체 궤도 예측에는 필요하지만, Kepler 우주선 궤도에는 과해요.

따라서 Kepler는 “기존 prediction의 solver mode 하나”가 아니라 **완전히 별도 기능**으로 봐야 해요.

---

## 2. 최종 폴더 구조

### `src/game/orbit`

기존 `src/game/orbit`는 이렇게 나눠요.

```txt
src/game/orbit/
  nbody/
    README.md

    nbody_prediction_tuning.h
    nbody_prediction_math.h
    nbody_prediction_math.cpp

    nbody_prediction_service.h
    nbody_prediction_service.cpp
    nbody_prediction_types.h

    trajectory/
      trajectory_utils.h
      trajectory_utils.cpp

    render_curve/
      nbody_orbit_render_curve.h
      nbody_orbit_render_curve_internal.h
      nbody_orbit_render_curve.cpp
      nbody_orbit_render_curve_render.cpp
      nbody_orbit_render_curve_pick.cpp

    prediction/
      nbody_prediction_service_internal.h
      nbody_prediction_diagnostics_util.h

      nbody_prediction_service_compute.cpp
      nbody_prediction_service_route_solvers.cpp
      nbody_prediction_service_spacecraft_route.cpp

      nbody_prediction_service_planned_route.cpp
      nbody_prediction_service_planned_stage_publisher.cpp
      nbody_prediction_service_planned_cache.cpp
      nbody_prediction_service_planned_maneuver.cpp
      nbody_prediction_service_planned_chunk_attempt.cpp
      nbody_prediction_service_planned.cpp

      nbody_prediction_service_trajectory.cpp
      nbody_prediction_service_sampling.cpp

      nbody_prediction_service_policy_integrator.cpp
      nbody_prediction_service_policy_chunking.cpp
      nbody_prediction_service_policy_profile.cpp
      nbody_prediction_service_policy_adaptive.cpp
      nbody_prediction_service_policy_ephemeris.cpp

  kepler/
    README.md

    kepler_types.h
    kepler_prediction_options.h

    kepler_primary_resolver.h
    kepler_primary_resolver.cpp

    kepler_orbit_builder.h
    kepler_orbit_builder.cpp

    kepler_maneuver_solver.h
    kepler_maneuver_solver.cpp

    kepler_orbit_tessellator.h
    kepler_orbit_tessellator.cpp

    kepler_orbit_metrics.h
    kepler_orbit_metrics.cpp

    kepler_debug.h
    kepler_debug.cpp

  common/
    orbit_basic_math.h
    orbit_line_types.h
```

여기서 중요한 점은 `render_curve/`도 `nbody/` 밑으로 들어간다는 거예요. Kepler에서 쓰지 않을 거라면 루트에 남겨둘 이유가 없어요.

`common/`은 아주 얇게 유지해요.

```txt
common에 둘 것:
  - finite vec/state 검사
  - safe_length
  - 간단한 angle wrap
  - draw line vertex 타입
  - 공통 색상 유틸 정도

common에 두지 않을 것:
  - OrbitRenderCurve
  - TrajectorySegment cache
  - prediction service
  - derived service
  - frame cache
  - streaming chunk
  - maneuver prediction bridge
```

공통 폴더는 작은 은수저 하나면 충분해요. 식탁 전체를 들여놓으면 또 얽혀요.

---

### `src/game/states/gameplay`

기존 `prediction`과 `maneuver`는 이름을 바꿔서 n체 전용으로 보내요.

```txt
src/game/states/gameplay/
  prediction_nbody/
    README.md

    gameplay_state_prediction_types.h
    gameplay_prediction_context.h
    gameplay_prediction_cache_internal.h
    gameplay_prediction_cache_internal.cpp

    prediction_trajectory_sampler.h
    prediction_trajectory_sampler.cpp

    prediction_frame_resolver.h
    prediction_frame_resolver.cpp
    prediction_frame_context_builder.h
    prediction_frame_context_builder.cpp
    prediction_frame_controller.h
    prediction_frame_controller.cpp
    prediction_frame_cache_builder.h
    prediction_frame_cache_builder.cpp

    prediction_metrics_builder.h
    prediction_metrics_builder.cpp

    streamed_chunk_assembly_builder.h
    streamed_chunk_assembly_builder.cpp

    gameplay_prediction_state.h
    gameplay_prediction_state.cpp
    gameplay_prediction_access.h

    prediction_host_context.h
    prediction_host_context_builder.h
    prediction_host_context_builder.cpp

    prediction_subject_state_provider.h
    prediction_subject_state_provider.cpp

    prediction_system.h
    prediction_system.cpp
    gameplay_prediction_derived_service.h
    gameplay_prediction_derived_service.cpp

    gameplay_state_prediction.cpp
    gameplay_state_prediction_frames.cpp

    runtime/
      기존 runtime 전부

    draw/
      기존 draw 전부

  prediction_kepler/
    README.md

    kepler_prediction_state.h
    kepler_prediction_system.h
    kepler_prediction_system.cpp

    kepler_prediction_subject.h
    kepler_prediction_subject.cpp

    kepler_prediction_builder.h
    kepler_prediction_builder.cpp

    kepler_prediction_draw.h
    kepler_prediction_draw.cpp

    kepler_prediction_pick.h
    kepler_prediction_pick.cpp

    kepler_prediction_metrics.h
    kepler_prediction_metrics.cpp

    kepler_prediction_debug.h
    kepler_prediction_debug.cpp

  maneuver_nbody/
    README.md
    기존 maneuver 폴더 내용 거의 전부

  maneuver_kepler/
    README.md

    kepler_maneuver_types.h

    kepler_maneuver_plan.h
    kepler_maneuver_plan.cpp

    kepler_maneuver_commands.h
    kepler_maneuver_commands.cpp

    kepler_maneuver_controller.h
    kepler_maneuver_controller.cpp

    kepler_maneuver_basis.h
    kepler_maneuver_basis.cpp

    kepler_maneuver_panel.h
    kepler_maneuver_panel.cpp

    kepler_maneuver_gizmo.h
    kepler_maneuver_gizmo.cpp

    kepler_maneuver_execution.h
    kepler_maneuver_execution.cpp
```

이렇게 하면 폴더 깊이는 오히려 깔끔해져요.

나쁜 구조는 이런 거예요.

```txt
prediction/nbody/runtime/...
prediction/nbody/draw/...
maneuver/nbody/runtime/cache/...
```

좋은 구조는 이쪽이에요.

```txt
prediction_nbody/runtime/...
prediction_nbody/draw/...
prediction_kepler/kepler_prediction_system.cpp
maneuver_kepler/kepler_maneuver_gizmo.cpp
```

`prediction_nbody`와 `prediction_kepler`가 나란히 서면, 어느 코드가 어느 세계의 법을 따르는지 바로 보여요.

---

## 3. Kepler prediction은 어떻게 동작할지

Kepler prediction은 기존 prediction처럼 request, worker, poll, derived cache로 가지 않아요.

```txt
GameplayState::on_update()
  -> KeplerPredictionSystem::update()

KeplerPredictionSystem::update()
  -> 현재 우주선 상태 읽기
  -> primary body 결정
  -> base Kepler arc 계산
  -> maneuver plan 있으면 planned arc 계산
  -> arc를 polyline으로 샘플링
  -> 작은 KeplerPredictionState에 저장

GameplayState::on_draw_ui()
  -> KeplerPredictionSystem::draw()

KeplerPredictionSystem::draw()
  -> 저장된 polyline을 바로 line draw로 제출
```

Kepler prediction에는 이런 것들이 없어요.

```txt
없음:
  - OrbitPredictionService::request()
  - poll_completed()
  - worker_loop()
  - generation coalescing
  - derived worker
  - frame cache
  - streamed chunk assembly
  - publish stage
  - planned chunk cache
  - OrbitRenderCurve
```

대신 상태는 아주 작게 둬요.

```cpp
struct KeplerPredictionState
{
    bool valid{false};

    double build_time_s{0.0};
    double horizon_s{0.0};

    orbitsim::BodyId primary_body_id{orbitsim::kInvalidBodyId};

    std::vector<Game::KeplerArc> base_arcs;
    std::vector<Game::KeplerArc> planned_arcs;

    std::vector<Game::OrbitLineVertex> base_lines;
    std::vector<Game::OrbitLineVertex> planned_lines;

    Game::KeplerOrbitMetrics metrics{};
};
```

이건 “거대한 캐시 시스템”이 아니라, 그냥 현재 표시할 Kepler 결과물이에요.

---

## 4. Kepler에서 `OrbitRenderCurve`를 쓰지 않는 이유

현재 `OrbitRenderCurve`는 대략 이런 일을 해요.

```txt
TrajectorySegment 입력
  -> persistent binary tree LOD 구성
  -> 카메라 기준 segment selection
  -> render line subdivision
  -> pick line downsample
```

n체 예측 결과는 불규칙하고 길고 복잡한 Hermite segment 묶음이라 이런 구조가 의미 있어요. 하지만 Kepler 궤도는 수학적으로 바로 샘플링할 수 있어요.

Kepler에서는 이렇게 하면 돼요.

```txt
KeplerArc
  -> true anomaly 기준 샘플링
  -> 또는 screen error 기준 adaptive 샘플링
  -> polyline
  -> draw / pick
```

추천하는 Kepler tessellator 옵션은 이 정도예요.

```cpp
struct KeplerOrbitTessellationOptions
{
    double max_true_anomaly_step_rad{glm::radians(2.0)};
    double max_time_step_s{300.0};
    double min_time_step_s{1.0};

    std::size_t max_vertices_per_arc{2048};
    std::size_t max_vertices_total{8192};

    double screen_error_px{1.0};
    bool use_screen_error{true};

    bool include_apoapsis_marker{true};
    bool include_periapsis_marker{true};
    bool include_maneuver_nodes{true};
};
```

타원 궤도는 true anomaly 또는 eccentric anomaly 기준으로 균일하게 샘플링하면 좋고, 쌍곡선 궤도는 시간/거리 기반으로 제한하면 돼요. 화면 줌에 따라 꼭 줄이고 싶으면 persistent LOD tree가 아니라, 그 프레임의 camera context로 바로 adaptive tessellation하면 충분해요. Kepler sampling은 비싼 n체 적분이 아니라 가벼운 해석 계산이니까요.

pick도 새로 간단히 만들어요.

```txt
KeplerPredictionPick
  - draw에 사용한 polyline을 그대로 사용
  - screen/world line distance 판정
  - 가까운 arc, node, apsis 반환
```

즉 render와 pick이 같은 line vertices를 공유해요. 별도의 pick LOD cache도 필요 없어요.

---

## 5. Kepler에서 frame cache도 쓰지 않는 이유

기존 prediction은 inertial solver 결과를 display frame으로 바꾸고, BCI, Synodic, LVLH 같은 프레임을 지원해요. 그래서 `PredictionFrameCacheBuilder`, `PredictionFrameResolver`, `gameplay_prediction_derived_service`가 필요해요. 현재 README에서도 derived service가 solver 결과를 표시 프레임으로 변환하고 render curve와 metrics를 재구성한다고 되어 있어요. 

Kepler 첫 구현에서는 이걸 버려요.

Kepler 표시 좌표는 단순하게 가요.

```txt
기본:
  primary-centered inertial display

렌더:
  primary body world position + relative Kepler sample position

나중에 필요하면:
  body-centered inertial 정도만 Kepler 전용으로 추가
```

Synodic, LVLH, 복잡한 display frame은 n체 prediction 쪽 기능으로 남겨요. Kepler 쪽에 가져오면 또 커져요.

---

## 6. 천체 n체와 우주선 Kepler의 연결

천체는 계속 n체로 가야 하니, Kepler 쪽도 미래 시점의 primary 위치는 알아야 해요.

하지만 Kepler가 기존 prediction cache를 직접 알면 안 돼요. 아주 얇은 provider만 둬요.

```cpp
struct KeplerCelestialStateProvider
{
    std::function<bool(orbitsim::BodyId body_id,
                       double t_s,
                       orbitsim::State& out_state)> state_at;
};
```

`prediction_nbody` 또는 `orbit/nbody`는 천체 ephemeris에서 body state를 제공해요. `prediction_kepler`는 그 callback만 받아요.

즉 의존성은 이렇게요.

```txt
prediction_nbody
  -> n체 ephemeris 생성

prediction_kepler
  -> KeplerCelestialStateProvider callback으로 primary state 조회
  -> nbody cache 구조는 모름
```

첫 버전에서는 primary state lookup이 없으면 현재 primary state를 그대로 쓰는 fallback을 둬도 돼요. 이후 천체 ephemeris가 안정되면 `state_at(t)`를 사용해 primary 움직임까지 반영해요.

---

## 7. Kepler maneuver는 새로 만들기

기존 maneuver는 prediction data와 runtime cache에 많이 기대고 있어요. README에서도 node marker placement가 prediction data와 render interpolation에 의존한다고 되어 있어요.  Kepler에서는 이 의존성을 끊고 새로 만드는 게 맞아요.

Kepler maneuver의 핵심 흐름은 이거예요.

```txt
node list
  -> 선택/편집
  -> node time에서 Kepler arc sample
  -> primary-relative RTN basis 계산
  -> dv_rtn_mps를 inertial delta-v로 변환
  -> 새 arc 생성
  -> planned orbit 즉시 재계산
```

Kepler maneuver에는 기존의 이런 것들이 필요 없어요.

```txt
없음:
  - ManeuverPredictionBridge
  - ManeuverRuntimeCacheBuilder
  - prediction preview streaming
  - planned suffix refine
  - staged publish
  - prediction generation invalidation
```

기존 UI 코드는 참고하거나 복사해도 돼요. 하지만 include가 `prediction_nbody`로 향하면 안 돼요. Kepler maneuver는 `KeplerPredictionSystem`과만 이야기하게 해요.

```cpp
class KeplerManeuverSystem
{
public:
    void update(const KeplerManeuverUpdateContext& ctx);
    void draw_panel(const KeplerManeuverUiContext& ctx);
    void draw_gizmo(const KeplerManeuverGizmoContext& ctx);

    const KeplerManeuverPlan& plan() const;
    uint64_t revision() const;
};
```

Kepler prediction은 이 plan을 읽어서 planned arcs를 만들어요.

---

## 8. `orbitsim` Kepler 강화 계획

QuaternionEngine은 `third_party/orbitsim`을 서브모듈로 쓰고 있어요.  `orbitsim`에는 이미 `propagate_kepler_universal()`이 있고, universal variable 방식으로 position/velocity를 전파하며 `converged`, `iterations`를 반환해요. 

그러니 Kepler 계산은 엔진 내부에 많이 만들지 말고, `orbitsim`을 강화하는 게 좋아요.

추가할 API는 이 정도로 잡아요.

```txt
orbitsim/include/orbitsim/kepler.hpp
  - 기존 propagate_kepler_universal 유지
  - KeplerStatus 추가
  - KeplerDiagnostics 추가
  - KeplerPropagationOptions 확장
  - propagate_kepler_universal_safe 추가

orbitsim/include/orbitsim/kepler_trajectory.hpp
  - KeplerArc
  - KeplerArcSample
  - KeplerTrajectoryOptions
  - sample_kepler_arc_state
  - build_kepler_arc_samples
  - build_kepler_arc_polyline

orbitsim/include/orbitsim/kepler_maneuver.hpp
  - compute_rtn_basis
  - rtn_delta_v_to_inertial
  - apply_impulse_rtn
  - build_maneuvered_kepler_arcs
```

Kepler 안정성 강화 항목은 이렇게요.

```txt
수치 안정성:
  - elliptic, hyperbolic, near-parabolic 케이스 분리 진단
  - Newton 실패 시 fallback
  - 큰 |z|에서 Stumpff overflow 보호
  - dt < 0 역전파 안정화
  - 여러 주기 장시간 전파 안정화

진단:
  - status enum
  - residual
  - iteration count
  - final chi
  - alpha
  - fallback 사용 여부

테스트:
  - circular orbit 1 period roundtrip
  - eccentric orbit 1 period roundtrip
  - hyperbolic escape
  - near-parabolic
  - negative dt roundtrip
  - maneuver impulse split
```

---

## 9. GameplayState 통합 구조

최종적으로 `GameplayState`는 두 세계를 따로 소유하게 해요.

```cpp
class GameplayState
{
private:
    NBodyCelestialPredictionSystem _nbody_celestial_prediction;
    KeplerPredictionSystem _kepler_prediction;
    KeplerManeuverSystem _kepler_maneuver;
};
```

업데이트 흐름은 이렇게요.

```txt
on_update:
  - nbody celestial prediction update
  - kepler maneuver update
  - kepler prediction update

on_fixed_update:
  - kepler maneuver execution
  - 실제 우주선 상태 갱신 또는 burn 적용

on_draw_ui:
  - nbody celestial orbit draw
  - kepler spacecraft orbit draw
  - kepler maneuver panel/gizmo draw
```

기존 n체 우주선 prediction은 당장 삭제하지 말고 `prediction_nbody` 안에 legacy/debug로 남겨요.

```txt
prediction_nbody:
  - celestial nbody prediction
  - legacy nbody spacecraft prediction, debug only

prediction_kepler:
  - normal spacecraft prediction
```

이러면 Kepler 결과와 n체 결과를 비교할 수 있어요. 디버그 모드에서만 켜면 됩니다.

---

## 10. 단계별 작업 계획

### Phase 0. 정책 확정

먼저 코드 규칙을 정해요.

```txt
Kepler는 기존 prediction cache를 include하지 않는다.
Kepler는 OrbitRenderCurve를 include하지 않는다.
Kepler는 derived service를 사용하지 않는다.
Kepler는 worker thread를 사용하지 않는다.
Kepler는 streamed chunk를 만들지 않는다.
Kepler와 nbody 사이 공유는 callback/provider 수준으로 제한한다.
```

이걸 README에 박아두면 나중에 덩굴이 못 올라와요.

---

### Phase 1. 기존 코드 폴더 분리, 동작 변경 없음

먼저 기존 코드만 이동해요.

```txt
src/game/orbit/* 
  -> src/game/orbit/nbody/*

src/game/states/gameplay/prediction/*
  -> src/game/states/gameplay/prediction_nbody/*

src/game/states/gameplay/maneuver/*
  -> src/game/states/gameplay/maneuver_nbody/*
```

이 단계에서는 로직을 바꾸지 않아요. include path와 CMake만 바꿔요.

현재 `src/cmake/sources_game.cmake`는 소스 파일을 명시적으로 나열하고 있어서, 이동하면 반드시 수정해야 해요.  테스트 CMake도 orbit prediction service 파일들을 직접 모아서 사용하므로 같이 고쳐야 해요. 

초기에는 호환 alias를 둬도 돼요.

```cpp
// game/orbit/orbit_prediction_service.h
#pragma once
#include "game/orbit/nbody/nbody_prediction_service.h"

namespace Game
{
    using OrbitPredictionService = NBodyPredictionService;
}
```

하지만 이 alias는 임시예요. Kepler 연결 후 제거해요.

---

### Phase 2. `orbitsim` Kepler 강화

`orbitsim`에 Kepler trajectory와 maneuver API를 추가해요.

```txt
orbitsim:
  - KeplerArc
  - Kepler trajectory sampling
  - RTN impulse apply
  - diagnostics
  - tests
```

이 단계는 엔진과 별개로 먼저 안정화하는 게 좋아요.

---

### Phase 3. `game/orbit/kepler` 구현

엔진 쪽 Kepler 계산 모듈을 만들어요.

```txt
game/orbit/kepler:
  - primary resolver
  - orbit builder
  - maneuver solver
  - tessellator
  - metrics
```

이때도 gameplay UI와 아직 연결하지 않아도 돼요. 테스트 또는 debug draw로 먼저 검증해요.

---

### Phase 4. `prediction_kepler` 구현

새 prediction 시스템을 만들어요.

```txt
prediction_kepler:
  - KeplerPredictionState
  - KeplerPredictionSystem
  - subject state resolver
  - draw
  - pick
  - metrics
```

이 시스템은 매 프레임 또는 revision 변경 시 즉시 계산해요.

간단한 invalidation만 둬요.

```txt
rebuild 조건:
  - 우주선 위치/속도 변화
  - primary 변경
  - maneuver revision 변경
  - horizon 설정 변경
  - draw 설정 변경
```

기존 generation id는 쓰지 않아요. 필요하면 `uint64_t revision` 정도만 둬요.

---

### Phase 5. `maneuver_kepler` 구현

기존 maneuver를 공유하지 말고 새로 만들어요.

```txt
maneuver_kepler:
  - plan model
  - commands
  - controller
  - RTN basis
  - panel
  - gizmo
  - execution
```

UI 코드 일부는 복사해서 시작해도 돼요. 다만 `maneuver_nbody`나 `prediction_nbody` include는 금지해요.

---

### Phase 6. GameplayState 연결

기본 모드를 바꿔요.

```txt
천체 orbit:
  prediction_nbody 사용

우주선 orbit:
  prediction_kepler 사용

우주선 maneuver:
  maneuver_kepler 사용
```

기존 n체 우주선 prediction은 debug flag에서만 켜요.

```cpp
enum class SpacecraftOrbitPredictionMode
{
    Kepler,
    NBodyDebug,
};
```

---

### Phase 7. old nbody 우주선 prediction 축소

Kepler가 안정되면 `prediction_nbody` 안에서 우주선 planned route, streamed preview, chunk cache를 legacy/debug로 더 깊게 격리하거나 제거해요.

```txt
남길 것:
  - celestial nbody prediction
  - celestial ephemeris
  - nbody debug comparison

줄일 것:
  - spacecraft planned chunk streaming
  - maneuver preview streaming
  - suffix refine
  - baseline cache
```

단, 바로 삭제하지 말고 비교 도구로 잠시 남기는 편이 좋아요.

---

## 11. Kepler 렌더링 세부 설계

Kepler 렌더링은 이렇게 갑니다.

```cpp
struct OrbitLineVertex
{
    WorldVec3 position_world{};
    double t_s{0.0};
    uint32_t flags{0};
};

struct KeplerOrbitLineSet
{
    bool valid{false};
    std::vector<OrbitLineVertex> vertices{};
};
```

라인 생성은 `kepler_orbit_tessellator`가 담당해요.

```txt
입력:
  - KeplerArc list
  - primary state provider
  - camera context
  - tessellation options

출력:
  - OrbitLineVertex list
```

draw는 간단히 adjacent pair를 그려요.

```txt
for i in 0..N-2:
  draw_line(vertices[i], vertices[i + 1])
```

pick도 같은 vertices를 씁니다.

```txt
for each line segment:
  screen-space 또는 world-space distance 계산
  가장 가까운 hit 반환
```

성능이 부족하면 그때 아주 작은 cache만 추가해요.

```cpp
struct KeplerLineCache
{
    bool valid{false};
    uint64_t orbit_revision{0};
    uint64_t maneuver_revision{0};
    uint64_t camera_bucket{0};

    std::vector<OrbitLineVertex> base;
    std::vector<OrbitLineVertex> planned;
};
```

하지만 이건 `OrbitRenderCurve`가 아니에요. 그냥 polyline cache예요.

---

## 12. 무엇을 복사하고, 무엇을 버릴지

기존 코드에서 가져와도 되는 것:

```txt
복사 가능:
  - 색상 상수
  - 패널 UI 레이아웃 일부
  - gizmo hit-test 아이디어
  - RTN basis 계산 아이디어
  - OrbitPlotSystem line submit 패턴
  - 기본 orbital metrics 계산 아이디어
```

가져오지 말아야 할 것:

```txt
버릴 것:
  - PredictionTrackState
  - OrbitPredictionCache
  - PredictionSolverTrajectoryCache
  - PredictionDisplayFrameCache
  - PredictionAnalysisCache 전체 구조
  - StreamedChunkAssemblyBuilder
  - OrbitPredictionDerivedService
  - PredictionRuntimeController
  - PredictionRequestFactory
  - PredictionSolverResultApplier
  - ManeuverPredictionBridge
  - ManeuverRuntimeCacheBuilder
  - OrbitRenderCurve
```

Kepler는 새로 태어난 코드답게 얇게 가야 해요.

---

## 13. 최종 판단

최종안은 이거예요.

```txt
src/game/orbit/nbody
  기존 n체 orbit prediction, trajectory segment, render curve, streaming 보관

src/game/orbit/kepler
  새 Kepler 계산, maneuver arc, tessellation

src/game/states/gameplay/prediction_nbody
  기존 복잡한 prediction 보관

src/game/states/gameplay/prediction_kepler
  새 단순 우주선 orbit 표시

src/game/states/gameplay/maneuver_nbody
  기존 복잡한 maneuver 보관

src/game/states/gameplay/maneuver_kepler
  새 단순 Kepler maneuver
```

그리고 `OrbitRenderCurve`는:

```txt
n체에는 필요해요.
Kepler에는 필요 없어요.
Kepler에서 쓰지 않는 편이 최종 계획에 맞아요.
```

Kepler 쪽 렌더는 `KeplerOrbitTessellator`로 새로 만들어요. analytic arc를 직접 polyline으로 만들고, draw와 pick이 그 polyline을 공유하면 됩니다. 기존 render curve는 n체의 복잡한 궤도 실타래를 다루는 낡은 베틀로 남겨두고, Kepler는 새 자로 선을 긋는 쪽이 훨씬 산뜻해요.
