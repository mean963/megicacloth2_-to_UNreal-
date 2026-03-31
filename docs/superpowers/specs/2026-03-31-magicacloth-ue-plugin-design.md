# MagicaCloth UE Plugin - Design Specification

> MagicaCloth2(Unity) 아키텍처를 충실히 재현하는 Unreal Engine 범용 클로스 피직스 플러그인
> KawaiiPhysics 스타일의 설정 UX 적용

---

## 1. 목표

| 항목 | 내용 |
|------|------|
| 재현 대상 | Unity **MagicaCloth2** (PBD 기반 클로스 피직스) |
| 타겟 엔진 | **Unreal Engine 5.5+** |
| 사용 방식 | **AnimBP 노드** (FAnimNode_SkeletalControlBase) |
| 시뮬레이션 모드 | **MeshCloth 우선**, BoneCloth 지원 |
| UX 참조 | **KawaiiPhysics** 설정 패턴 |
| 최우선 요구사항 | **실시간 방송 환경에서 Frame Drop 없음** |
| 사용 사례 | 치마, 머리카락, 망토, 리본 등 범용 |
| 개발 방식 | 기존 코드 리팩토링 후, MagicaCloth2 아키텍처 충실 재현 |

---

## 2. 전체 아키텍처

MagicaCloth2의 Manager/Team 패턴을 UE의 WorldSubsystem으로 재현한다.

```
┌─────────────────────────────────────────────────────────┐
│              UMagicaClothSubsystem                       │
│            (UWorldSubsystem - 월드당 1개)                │
│                                                          │
│  ┌──────────────┐  ┌───────────────┐  ┌───────────────┐ │
│  │ TeamManager  │  │ VirtualMesh   │  │ Simulation    │ │
│  │              │  │ Manager       │  │ Manager       │ │
│  │ - Team 배치  │  │ - 메시 캐시   │  │ - PBD 실행    │ │
│  │ - 그룹 관리  │  │ - 토폴로지    │  │ - 워커 스레드 │ │
│  └──────────────┘  └───────────────┘  └───────────────┘ │
│                                                          │
│  ┌───────────────┐  ┌───────────────┐                   │
│  │ Collider      │  │ Wind          │                   │
│  │ Manager       │  │ Manager       │                   │
│  │ - Shape 풀    │  │ - WindZone    │                   │
│  │ - 충돌 검출   │  │ - Perlin 노이즈│                   │
│  └───────────────┘  └───────────────┘                   │
└─────────────────────────────────────────────────────────┘

┌──────────────────┐         ┌─────────────────────┐
│ UMagicaCloth     │────────▶│ FAnimNode_MagicaCloth│
│ Component        │         │ (AnimBP 노드)        │
│ (Actor에 부착)   │         │ - 결과 읽기/보간     │
│ - 설정/파라미터  │         │ - 본 오버라이드      │
│ - Team 등록      │         └─────────────────────┘
└──────────────────┘
```

### 핵심 설계 원칙

- **단일 SimulationManager**가 모든 Team을 배치 처리 (클로스 인스턴스별 스레드 X)
- 파티클/제약 데이터를 글로벌 배열에 연속 배치, Team별 DataChunk로 범위 지정
- Self-collision 없는 Team은 단일 Job, 있는 Team은 Split Job으로 분리

---

## 3. VirtualMesh 시스템

CPU 전용 메시 표현으로, 시뮬레이션과 렌더링 메시를 분리한다.

### 3.1 데이터 구조

```
FVirtualMesh
├── Vertices[]       ── Position, Normal, UV
├── Triangles[]      ── 삼각형 인덱스
├── Edges[]          ── 엣지 인접 정보
├── BoneWeights[]    ── 스키닝 웨이트
├── VertexAttributes[] ── Fixed/Move/Limit 플래그 (uint8)
├── DepthArray[]     ── 0.0(root) ~ 1.0(tip) 깊이값
└── Adjacency        ── 버텍스→버텍스, 버텍스→삼각형 맵
```

### 3.2 두 가지 모드

| | BoneCloth | MeshCloth |
|---|---|---|
| 소스 | Transform 계층 (본) | SkinnedMeshRenderer |
| 파티클 | 각 본 = 1 파티클 | 각 버텍스 = 1 파티클 |
| 토폴로지 | 부모-자식 체인 | 삼각형 메시 |
| 결과 반영 | 본 Transform 직접 쓰기 | 본에 역매핑 후 스키닝 |
| 페인트 | 수동 설정 | 텍스처 기반 (R=Fixed, G=Move, B=Limit) |

### 3.3 Depth 시스템

- 모든 버텍스/본에 0.0(root)~1.0(tip) 깊이값 부여
- Stiffness, Mass, Inertia, Wind 영향도를 깊이 기반 커브로 제어
- `FRuntimeFloatCurve`(UE 인라인 커브) 사용하여 에디터에서 직관적 편집

### 3.4 메시 리덕션 (선택적)

- 고해상도 메시를 저해상도 프록시로 변환하여 시뮬레이션
- 결과를 원본 메시에 역매핑
- 고폴리 치마 메시에서 성능 확보에 활용

---

## 4. PBD 솔버 + 제약 조건

### 4.1 시뮬레이션 스텝 (매 프레임)

```
1. UpdateFixedParticles()     ← 애니메이션 포즈에서 고정 파티클 위치 갱신
2. PredictPositions()         ← Verlet 적분: nextPos = pos + vel + accel * dt²
3. SolveConstraints() ×N회    ← SolverIterations (기본 5)
   ├── Distance (Vertical + Horizontal)
   ├── Bending (Dihedral Angle)
   ├── Tether (최대 거리 제한)
   ├── Angle (BoneCloth 전용)
   └── Inertia (월드/로컬 이동 제한)
4. SolveCollisions()          ← 콜라이더 충돌 해결
5. SolveSelfCollision()       ← 자기 충돌 (선택적)
6. UpdateVelocities()         ← vel = (newPos - oldPos) / dt
7. ClampVelocities()          ← MaxVelocity 제한
8. ApplyDamping()             ← 깊이 기반 속도 감쇠
```

### 4.2 제약 조건 상세

| 제약 | 역할 | 알고리즘 |
|------|------|----------|
| Distance (Vertical) | 부모-자식 간 거리 유지 | `corr = (dist - restLen) * stiffness * dir / (m1+m2)` |
| Distance (Horizontal) | 같은 깊이 이웃 간 거리 유지 | 동일 공식, 별도 stiffness 커브 |
| Bending | 삼각형 쌍의 이면각 보존 | DirectionDihedralAngle (초기 접힘 방향 유지) |
| Tether | root로부터 최대 이동 거리 제한 | `if (dist > maxDist) pos = root + dir * maxDist` |
| Inertia | 캐릭터 이동/회전 시 관성 제한 | 월드 시프트 + 로컬 감쇠 |
| Angle | 관절 각도 유지 (BoneCloth) | 기준 회전에서의 각도 오차 보정 |

### 4.3 데이터 패킹 (MagicaCloth2 방식)

- Distance 인덱스: `uint32 (count:10bit | startIndex:22bit)`
- Triangle 인덱스: `uint64 (v0:16 | v1:16 | v2:16 | v3:16)`
- VertexAttribute: `uint8` 플래그 (Fixed/Move/Collision 등)

---

## 5. 콜리전 시스템

### 5.1 3중 소싱 (KawaiiPhysics 호환)

| 소스 | 설명 |
|------|------|
| **Physics Asset (PhAT)** | SkeletalMesh에 설정된 PhAT 자동 추출. 가장 편리 |
| **인라인 설정** | AnimNode 디테일에서 직접 Sphere/Capsule/Box/Plane 추가 |
| **DataAsset** | 재사용 가능한 에셋. KawaiiPhysics LimitsDataAsset 호환 |

### 5.2 콜라이더 타입

| 타입 | 용도 | MagicaCloth2 | KawaiiPhysics |
|------|------|:---:|:---:|
| Sphere | 관절 부위 | O | O |
| Capsule | 팔/다리 | O | O |
| Box | 몸통/사각형 영역 | X | O |
| Plane | 바닥/벽 | X | O |

- **Inner/Outer 모드**: Outer(밖으로 밀기, 기본), Inner(안으로 가두기)
- **Point/Edge 충돌 모드**: MagicaCloth2 포팅. Edge 모드는 캡슐에서 더 정밀
- **마찰**: Dynamic Friction + Static Friction (MagicaCloth2 방식)

### 5.3 충돌 해결 과정

```
매 시뮬레이션 스텝:
1. 콜라이더 DrivingBone의 Transform 갱신
2. 파티클별 충돌 검사
   ├── Sphere: dist(particle, center) < radius → 표면으로 밀기
   └── Capsule: point-to-segment dist < radius → 표면으로 밀기
3. 마찰 적용 (Dynamic + Static)
4. 결과 반영 → 다음 제약 조건으로
```

### 5.4 Physics Asset 자동 추출

```cpp
// AnimNode 초기화 시 PhysicsAsset에서 콜라이더 자동 생성
PhysicsAsset->BodySetup[] 순회:
├── SphereElem → FSphericalLimit
├── SphylElem → FCapsuleLimit
└── TaperedCapsuleElem → FCapsuleLimit (변환)

// 본 필터로 특정 본의 콜라이더만 사용 가능
PhysicsAssetBoneFilter: [UpperLeg_L, UpperLeg_R, LowerLeg_L, LowerLeg_R]
```

---

## 6. Wind + Inertia 시스템

### 6.1 Wind

```
UMagicaWindZone (Actor) 또는 FInstancedStruct 확장

파라미터:
├── Influence (0~1)      전체 강도
├── Frequency            돌풍 속도
├── Turbulence           난류 (Perlin Noise 기반)
├── DepthWeight          깊이별 바람 영향
├── MovingWind           캐릭터 이동 반대 방향 바람
└── Synchronization      여러 클로스 간 위상 동기화

windForce = mainDirection * Influence
          + PerlinNoise3D(time * Frequency) * Turbulence
          + (-characterVelocity) * MovingWind
파티클별: force *= depthCurve(depth)
```

### 6.2 Inertia

```
1. World Inertia (이동)
   - 캐릭터 원점 이동량 감지 → 파티클에 글로벌 시프트
   - 속도 제한으로 폭발 방지

2. Local Inertia (회전)
   - 캐릭터 원점 회전량 감지 → 로컬 속도 감쇠

3. Teleport Detection
   - 이동량 > TeleportDistanceThreshold → 시뮬레이션 리셋
   - 회전량 > TeleportRotationThreshold → 시뮬레이션 리셋

4. Anchor (선택적)
   - 차량 탑승 시 관성 상쇄용 앵커 Transform
```

---

## 7. 스레딩 + 성능

### 7.1 스레딩 아키텍처

```
[Game Thread]                    [SimulationManager Worker Thread]
     │                                        │
     ├─ Tick: 애니메이션 포즈 수집             │
     ├─ Tick: 콜라이더 Transform 수집          │
     ├─ CriticalSection으로 입력 전달 ────────▶├─ 입력 수신
     │                                        ├─ Team별 배치 PBD (90Hz)
     │                                        │   ├─ Team0: 치마
     │                                        │   ├─ Team1: 머리카락
     │                                        │   └─ Team2: 망토
     │                                        ├─ DoubleBuffer에 Write
     ├─ AnimBP: DoubleBuffer.Read() ◀─────────┤
     ├─ AnimBP: 보간 적용                      │
     └─ AnimBP: 본 오버라이드 출력             │
```

### 7.2 Team 시스템

```cpp
struct FClothTeam
{
    int32 TeamId;
    FDataChunk ParticleChunk;      // 글로벌 배열 내 (StartIndex, Count)
    FDataChunk ConstraintChunk;    // 제약 조건 범위
    TArray<FColliderShape*> Colliders;
    FWindParams WindParams;
    FInertiaParams InertiaParams;
    FClothDoubleBuffer DoubleBuffer;  // Team별 독립 더블버퍼
    bool bHasSelfCollision;
};
```

### 7.3 성능 안전장치 (방송 환경)

| 장치 | 동작 |
|------|------|
| MaxStepsPerFrame | 프레임당 시뮬레이션 최대 3회 제한 |
| Adaptive Hz | CPU 과부하 시 90→60→30Hz 자동 감소 |
| Velocity Clamp | 파티클 속도 상한으로 폭발 방지 |
| Interpolation | SimHz ↔ RenderHz 차이를 보간으로 보정 |
| Lock-free DoubleBuffer | atomic swap으로 Game Thread 블로킹 없음 |

### 7.4 MeshCloth 결과 반영 파이프라인

```
시뮬레이션 버텍스 위치
    ↓ (역스키닝)
버텍스 → 본 가중치 역매핑
    ↓ (본 Transform 계산)
각 본의 최적 Transform (SVD / Polar Decomposition)
    ↓ (AnimBP 출력)
OutBoneTransforms[] → UE 스키닝 파이프라인
```

---

## 8. AnimNode 파라미터 구조 (KawaiiPhysics 스타일)

### 8.1 카테고리 구조

```cpp
FAnimNode_MagicaCloth : public FAnimNode_SkeletalControlBase
{
    // ═══ Bones ═══
    FBoneReference RootBone;
    TArray<FBoneReference> ExcludeBones;
    TArray<FMagicaRootBoneSetting> AdditionalRootBones;
    EBoneForwardAxis BoneForwardAxis;
    float DummyBoneLength;
    EMagicaClothType ClothType;  // BoneCloth / MeshCloth

    // ═══ Physics Settings ═══
    FMagicaPhysicsSettings PhysicsSettings;
    //   float Stiffness = 0.8f;
    //   float BendStiffness = 0.5f;
    //   float Damping = 0.1f;
    //   float Mass = 1.0f;
    //   float Radius = 3.0f;
    //   float MaxVelocity = 1000.0f;
    ESimulationSpace SimulationSpace;
    int32 TargetFramerate = 90;
    int32 SolverIterations = 5;
    int32 MaxStepsPerFrame = 3;
    float TeleportDistanceThreshold = 300.0f;
    float TeleportRotationThreshold = 10.0f;
    FVector SkelCompMoveScale = FVector(1,1,1);

    // ═══ Physics Settings (Advanced - 커브) ═══
    FRuntimeFloatCurve StiffnessCurve;
    FRuntimeFloatCurve BendStiffnessCurve;
    FRuntimeFloatCurve DampingCurve;
    FRuntimeFloatCurve MassCurve;
    FRuntimeFloatCurve RadiusCurve;

    // ═══ Limits (콜라이더) ═══
    TArray<FSphericalLimit> SphericalLimits;
    TArray<FCapsuleLimit> CapsuleLimits;
    TArray<FBoxLimit> BoxLimits;
    TArray<FPlanarLimit> PlanarLimits;
    UMagicaLimitsDataAsset* LimitsDataAsset;
    UPhysicsAsset* PhysicsAssetForLimits;
    TArray<FBoneReference> PhysicsAssetBoneFilter;

    // ═══ Constraints ═══
    TArray<FMagicaBoneConstraint> BoneConstraints;
    UMagicaConstraintsDataAsset* ConstraintsDataAsset;
    int32 ConstraintIterationCountBeforeCollision = 3;
    int32 ConstraintIterationCountAfterCollision = 1;

    // ═══ Inertia ═══
    float WorldInertia = 1.0f;
    float LocalInertia = 1.0f;
    FRuntimeFloatCurve InertiaCurve;

    // ═══ ExternalForce ═══
    FVector Gravity = FVector(0, 0, -980.f);
    bool bEnableWind;
    float WindScale = 1.0f;
    TArray<FInstancedStruct> ExternalForces;

    // ═══ Tag ═══
    FGameplayTag MagicaClothTag;
};
```

### 8.2 UPROPERTY 메타데이터 패턴

- `PinHiddenByDefault` — AnimGraph 핀 숨김, 디테일에서만 편집
- `AdvancedDisplay` — 커브 등 고급 설정은 접힌 상태
- `EditCondition` / `InlineEditConditionToggle` — 조건부 표시
- `TitleProperty` — 배열 아이템 표시 포맷
- `ClampMin` / `ClampMax` — 값 범위 제한

### 8.3 DataAsset 패턴

| DataAsset | 용도 |
|-----------|------|
| `UMagicaLimitsDataAsset` | 콜라이더 설정 재사용 |
| `UMagicaConstraintsDataAsset` | 본 제약 조건 재사용 |
| `UMagicaPhysicsPreset` | 전체 물리 파라미터 프리셋 (치마/머리/망토용) |

---

## 9. 에디터 통합

### 9.1 AnimGraphNode

```cpp
UAnimGraphNode_MagicaCloth : public UAnimGraphNode_SkeletalControlBase
{
    // 디테일 패널 커스텀
    void CustomizeDetails(IDetailLayoutBuilder&);
    void CustomizeDetailTools();          // Export 버튼
    void CustomizeDetailDebugVisualizations();  // 디버그 토글

    // 디버그 플래그
    bool bEnableDebugDrawBone;
    bool bEnableDebugDrawSphereLimit;
    bool bEnableDebugDrawCapsuleLimit;
    bool bEnableDebugDrawConstraint;
    bool bEnableDebugDrawWind;
    // ...
};
```

### 9.2 Edit Mode (뷰포트)

```cpp
FMagicaClothEditMode : public IAnimNodeEditMode
{
    // 렌더링
    void RenderModifyBones();       // 본 위치/속도
    void RenderSphericalLimits();   // Sphere 콜라이더
    void RenderCapsuleLimits();     // Capsule 콜라이더
    void RenderConstraints();       // 제약 조건 라인
    void RenderWind();              // 바람 방향 화살표

    // 기즈모
    FVector GetWidgetLocation();    // 선택된 콜라이더 위치
    bool DoTranslation();           // 이동
    bool DoRotation();              // 회전
    bool HandleClick();             // 클릭 선택
};
```

---

## 10. 플러그인 디렉토리 구조

```
MagicaClothUE/
├── Source/
│   ├── MagicaClothUE/              (Runtime)
│   │   ├── Core/
│   │   │   ├── MagicaClothSubsystem.h/.cpp
│   │   │   ├── TeamManager.h/.cpp
│   │   │   └── ClothTypes.h
│   │   ├── VirtualMesh/
│   │   │   ├── VirtualMesh.h/.cpp
│   │   │   ├── VirtualMeshBuilder.h/.cpp
│   │   │   └── VertexAttribute.h
│   │   ├── Simulation/
│   │   │   ├── SimulationManager.h/.cpp
│   │   │   ├── PBDSolver.h/.cpp
│   │   │   ├── ClothDoubleBuffer.h
│   │   │   └── Constraints/
│   │   │       ├── DistanceConstraint.h/.cpp
│   │   │       ├── BendingConstraint.h/.cpp
│   │   │       ├── TetherConstraint.h/.cpp
│   │   │       ├── InertiaConstraint.h/.cpp
│   │   │       ├── AngleConstraint.h/.cpp
│   │   │       └── SelfCollisionConstraint.h/.cpp
│   │   ├── Colliders/
│   │   │   ├── ColliderManager.h/.cpp
│   │   │   ├── MagicaColliderComponent.h/.cpp
│   │   │   ├── ColliderShapes.h
│   │   │   └── MagicaLimitsDataAsset.h/.cpp
│   │   ├── Wind/
│   │   │   ├── WindManager.h/.cpp
│   │   │   └── MagicaWindZone.h/.cpp
│   │   ├── AnimNode/
│   │   │   └── AnimNode_MagicaCloth.h/.cpp
│   │   ├── Component/
│   │   │   └── MagicaClothComponent.h/.cpp
│   │   └── DataAssets/
│   │       ├── MagicaConstraintsDataAsset.h/.cpp
│   │       └── MagicaPhysicsPreset.h/.cpp
│   │
│   └── MagicaClothUEEditor/        (Editor)
│       ├── AnimGraphNode_MagicaCloth.h/.cpp
│       ├── MagicaClothEditMode.h/.cpp
│       └── MagicaClothDetails.h/.cpp
│
└── MagicaClothUE.uplugin
```

---

## 11. KawaiiPhysics 호환성

| 기능 | 호환 방식 |
|------|-----------|
| LimitsDataAsset | KawaiiPhysics의 `UKawaiiPhysicsLimitsDataAsset` 직접 참조 가능 (KawaiiPhysics 플러그인이 프로젝트에 있을 때만. 없으면 자체 `UMagicaLimitsDataAsset` 사용) |
| PhysicsAsset 콜라이더 | 동일한 PhAT 추출 패턴 사용 |
| FRuntimeFloatCurve | 동일한 인라인 커브 시스템 |
| FInstancedStruct ExternalForces | 동일한 확장 패턴 |
| EditMode 기즈모 | 동일한 뷰포트 편집 패턴 |
| 디버그 토글 | 동일한 디테일 패널 토글 패턴 |

---

## 12. MagicaCloth2 대비 차이점

| 항목 | MagicaCloth2 | 우리 구현 |
|------|-------------|-----------|
| 플랫폼 | Unity DOTS/Jobs/Burst | UE FRunnable + 워커 스레드 |
| 콜라이더 소싱 | 자체 컴포넌트만 | PhAT + DataAsset + 인라인 3중 |
| 커브 시스템 | 에디터 전용 커브 | FRuntimeFloatCurve (런타임 수정 가능) |
| Box/Plane 콜라이더 | 미지원 | 지원 (KawaiiPhysics 호환) |
| 에디터 UX | Unity Inspector | KawaiiPhysics 스타일 AnimBP 노드 |
| 메시 리덕션 | ProxyMesh | 선택적 지원 (Phase 2) |
| Paint Map | Texture 기반 | Phase 2에서 지원 |

---

## 13. 개발 단계

### Phase 1: Core (필수)
1. UMagicaClothSubsystem + TeamManager
2. VirtualMesh (BoneCloth 모드)
3. PBD 솔버 리팩토링 (범용 파티클 기반)
4. 제약 조건 6종 (Distance V/H, Bending, Tether, Inertia, Angle)
5. SimulationManager (워커 스레드 + 더블버퍼)
6. 콜리전 3중 소싱 (PhAT + 인라인 + DataAsset)
7. AnimNode 리팩토링 (KawaiiPhysics 스타일 파라미터)
8. FRuntimeFloatCurve 깊이 커브 시스템

### Phase 2: MeshCloth + 고급
9. VirtualMesh (MeshCloth 모드)
10. 메시 → 본 역매핑 파이프라인
11. Paint Map 시스템
12. 메시 리덕션
13. Self-collision

### Phase 3: 에디터 + 폴리시
14. AnimGraphNode 디테일 커스텀
15. EditMode (뷰포트 기즈모)
16. Wind 시스템
17. DataAsset Export 도구
18. 디버그 시각화 토글
19. MagicaPhysicsPreset (프리셋 에셋)

### Phase 4: 최적화 + 안정화
20. 데이터 패킹 최적화
21. 방송 환경 안전장치 (Adaptive Hz, 속도 클램핑)
22. 멀티 캐릭터 배치 테스트
23. 프로파일링 + 최적화
