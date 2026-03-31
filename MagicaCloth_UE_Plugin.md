# Magica Cloth → Unreal Engine 플러그인 기획 정리

> Unity의 Magica Cloth 2와 동일한 천 물리 시뮬레이션을 Unreal Engine에서 재현하는 플러그인 개발 계획

---

## 1. 목표 정의

| 항목 | 내용 |
|------|------|
| 재현 대상 | Unity **Magica Cloth 2** (본 체인 기반 천 물리) |
| 사용 환경 | Unreal Engine **Animation Blueprint (AnimBP)** 노드 |
| 사용 방식 | C++ + Blueprint 양쪽 지원 |
| 최우선 요구사항 | **실시간 방송 중 Frame Drop 없음** |

---

## 2. Magica Cloth 2란?

- Unity DOTS 기반의 고성능 천 시뮬레이션 플러그인
- **Transform(본)** 과 **Mesh** 양쪽 모두 지원
- 머리카락, 치마, 망토, 리본 등 본 체인에 물리 적용
- 핵심 알고리즘: **Position Based Dynamics (PBD)**
- 물리 루프를 게임 프레임과 분리하여 안정적인 성능 제공 (기본값 **90Hz 고정**)
- 과부하 시 보간(Interpolation)으로 위치 보완

---

## 3. 왜 직접 만드는가?

| 옵션 | 장점 | 단점 |
|------|------|------|
| SPCRJointDynamics UE | 검증된 플러그인, AnimBP 노드 지원 | 알고리즘이 달라 Magica와 파라미터 체감 차이 |
| AnimVerlet | 오픈소스, Verlet 기반 | 커스터마이즈 한계 |
| **직접 개발** | Magica와 동일한 파라미터/동작 보장 | 개발 공수 큼 |

→ Unity → Unreal 이전 시 **동일한 파라미터로 동작 재현**이 목표이므로 직접 개발 선택

---

## 4. 성능 설계 원칙 (Frame Drop 방지)

### 핵심 구조: 물리와 렌더링의 완전 분리

```
[Game Thread]              [Physics Worker Thread]
     │                              │
     ├─ AnimBP 실행                 ├─ PBD 솔버 계산 (90Hz 고정)
     ├─ 렌더링                      ├─ 본 위치 업데이트
     └─ 결과 읽기 ←── 보간 ───── └─ 결과 버퍼에 Write
```

### 원칙 1: 전용 워커 스레드 (FRunnable)

```cpp
class FClothSimThread : public FRunnable
{
    virtual uint32 Run() override
    {
        const float SimHz = 90.f;
        const float SimDt = 1.f / SimHz;

        while (bRunning)
        {
            double Start = FPlatformTime::Seconds();

            StepSimulation(SimDt);       // PBD 솔버 실행
            WriteToDoubleBuffer();        // 결과 버퍼에 Write

            double Elapsed = FPlatformTime::Seconds() - Start;
            float SleepTime = SimDt - (float)Elapsed;
            if (SleepTime > 0.f)
                FPlatformProcess::Sleep(SleepTime);
        }
        return 0;
    }
};
```

### 원칙 2: 더블버퍼링으로 스레드 충돌 방지

```cpp
struct FClothDoubleBuffer
{
    TArray<FTransform> Buffer[2];
    std::atomic<int>   WriteIndex{0};

    void Write(const TArray<FTransform>& Data) {
        int next = 1 - WriteIndex.load();
        Buffer[next] = Data;
        WriteIndex.store(next); // atomic swap — 락 불필요
    }

    const TArray<FTransform>& Read() const {
        return Buffer[WriteIndex.load()];
    }
};
```

### 원칙 3: AnimBP 노드는 결과 읽기만 수행

```cpp
void FAnimNode_MagicaLike::EvaluateSkeletalControl_AnyThread(
    FComponentSpacePoseContext& Output,
    TArray<FBoneTransform>& OutBoneTransforms)
{
    // 물리 계산 절대 금지 — 결과만 읽음
    const auto& SimResult = ClothSim->DoubleBuffer.Read();
    float Alpha = ClothSim->GetInterpolationAlpha();

    for (int i = 0; i < BoneChain.Num(); i++)
    {
        FTransform Interped = FTransform::Identity;
        Interped.Blend(PrevResult[i], SimResult[i], Alpha);
        OutBoneTransforms.Add(FBoneTransform(BoneChain[i], Interped));
    }
}
```

---

## 5. 방송 환경 추가 안전장치

| 문제 상황 | 해결책 |
|----------|--------|
| 프레임 스파이크 | 시뮬레이션 최대 실행 횟수 캡 (기본 3회 제한) |
| 갑작스러운 움직임 | 속도 클램핑으로 파티클 폭발 방지 |
| CPU 과부하 | 시뮬레이션 주파수 동적 조절 (90Hz → 60Hz → 30Hz) |
| 카메라 컷 | 리셋 후 보간으로 순간 튀는 현상 방지 |

---

## 6. 구현해야 할 핵심 요소

### PBD 솔버

```
1. 파티클(본) 위치 예측
2. Constraint 만족 반복 계산
3. 속도 업데이트
```

### Constraint 종류

| Constraint | 역할 |
|-----------|------|
| Structural | 본 간 거리 유지 |
| Bend | 구부러짐 저항 |
| Self-Collision | 자기 충돌 |

### Collider

- Sphere Collider
- Capsule Collider
- Box Collider
- 캐릭터 본에 부착 가능

---

## 7. AnimBP 노드 구조 (C++)

```cpp
USTRUCT(BlueprintInternalUseOnly)
struct FAnimNode_MagicaClothLike : public FAnimNode_Base
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere)
    FBoneReference RootBone;        // 시뮬레이션 시작 본

    UPROPERTY(EditAnywhere)
    float Stiffness = 0.8f;         // Magica의 Stiffness

    UPROPERTY(EditAnywhere)
    float Damping = 0.1f;           // Magica의 Damping

    UPROPERTY(EditAnywhere)
    FVector Gravity = {0, 0, -980.f};

    // 내부 파티클 상태
    TArray<FVector> Positions;
    TArray<FVector> PrevPositions;

    virtual void EvaluateSkeletalControl_AnyThread(
        FComponentSpacePoseContext& Output,
        TArray<FBoneTransform>& OutBoneTransforms) override;
};
```

---

## 8. 플러그인 디렉토리 구조

```
MagicaClothUE/
├── Source/
│   ├── Simulation/
│   │   ├── FClothSimThread.h/.cpp      ← 전용 워커 스레드
│   │   ├── FPBDSolver.h/.cpp           ← PBD 솔버
│   │   ├── FClothConstraints.h/.cpp    ← Structural / Bend / Collision
│   │   └── FClothDoubleBuffer.h        ← 스레드 안전 더블버퍼
│   │
│   ├── AnimNode/
│   │   ├── AnimNode_MagicaCloth.h/.cpp ← FAnimNode_Base 구현
│   │   └── AnimGraphNode_MagicaCloth.h ← AnimBP 에디터 노드
│   │
│   └── Colliders/
│       ├── UMagicaSphereCollider.h
│       └── UMagicaCapsuleCollider.h
│
└── MagicaClothUE.uplugin
```

---

## 9. 개발 순서 (권장)

1. **PBD 솔버 구현** — Constraint 없이 중력만 적용하는 최소 버전
2. **더블버퍼 + 워커 스레드** — 멀티스레딩 구조 먼저 확립
3. **AnimBP 노드 등록** — 에디터에서 노드로 보이게 설정
4. **Constraint 추가** — Structural → Bend 순서로
5. **Collider 연동** — Sphere / Capsule
6. **파라미터 튜닝** — Magica Cloth 2 기본값에 맞게 보정
7. **방송 안전장치** — 속도 클램핑, 주파수 조절, 카메라 컷 대응

---

## 10. Unity ↔ Unreal 파라미터 매핑 (예정)

| Magica Cloth 2 파라미터 | Unreal 구현 |
|------------------------|------------|
| `Stiffness` | Structural Constraint 강도 |
| `Damping` | 속도 감쇠 계수 |
| `Gravity` | 파티클 중력 벡터 |
| `Mass` | 파티클 질량 |
| `Friction` | 충돌 시 속도 감쇠 |
| `UpdateMode` (Normal/Unscaled/Realtime) | 시뮬레이션 Hz 모드 |
| `MaxVelocity` | 속도 클램핑 상한 |

---

*최종 목표: 실시간 방송 환경에서 frame drop 없이, Magica Cloth 2와 동일한 파라미터 체감으로 동작하는 Unreal Engine AnimBP 플러그인*
