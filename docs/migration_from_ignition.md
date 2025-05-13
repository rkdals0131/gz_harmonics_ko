# Migration Guide

안녕하세요 Gazebo 커뮤니티 여러분!!

2022년 4월, ["Ignition"이라는 이름을 폐기하고 "Gazebo"를 선호하기로 발표했습니다.](https://community.gazebosim.org/t/a-new-era-for-gazebo/1356)
이 마이그레이션 가이드는 여러분의 패키지에서 필요한 변경 사항을 실행하는 데 도움이 될 것이며, 다행히도 Gazebo Classic에서의 이전만큼 번거롭지는 않을 것입니다!

## Overview

### 변경 사항

실제로 어떤 일이 일어나고 있나요? 요약하자면:

- `Ignition` 또는 `ign` 이름이 사용될 때마다, 대소문자를 유지하면서 Gazebo 대응 이름(`Gazebo` 또는 `gz`)이 대신 사용됩니다.
- `ign-gazebo` / `Ignition Gazebo`는 `gz-sim` / `Gazebo Sim`이 되었습니다.
- Ignition 로고가 Gazebo 로고로 교체되었습니다.

이러한 변경 사항은 다음에서 이루어졌습니다:

- 웹사이트
- GitHub 조직 및 저장소
- 문서
- UI
- 네임스페이스, 명령줄 도구, 공유 라이브러리, 디렉토리, API, 파일

이는 사용자의 마이그레이션 노력 대부분이 파일 이름, 디렉토리 및 소스 코드의 지능적인 찾기 및 바꾸기를 포함한다는 것을 의미합니다.
핵심 라이브러리에서 마이그레이션을 지원하기 위해 이루어진 변경 사항을 추적해야 하는 경우 [추적 GitHub 이슈](https://github.com/gazebo-tooling/release-tools/issues/698)를 참조할 수도 있습니다.

### Tick-tocks and Hard-tocks

이 섹션에서는 이루어진 다양한 변경 사항에 대한 개요만 제공합니다. tick-tock에 대한 자세한 목록은 각 개별 핵심 라이브러리 저장소의 마이그레이션 파일을 참조하세요:

- [gz-cmake](https://github.com/gazebosim/gz-cmake/blob/main/Migration.md)
- [gz-common](https://github.com/gazebosim/gz-common/blob/main/Migration.md)
- [gz-fuel-tools](https://github.com/gazebosim/gz-fuel-tools/blob/main/Migration.md)
- [gz-gui](https://github.com/gazebosim/gz-gui/blob/main/Migration.md)
- [gz-launch](https://github.com/gazebosim/gz-launch/blob/main/Migration.md)
- [gz-math](https://github.com/gazebosim/gz-math/blob/main/Migration.md)
- [gz-msgs](https://github.com/gazebosim/gz-msgs/blob/main/Migration.md)
- [gz-physics](https://github.com/gazebosim/gz-physics/blob/main/Migration.md)
- [gz-plugin](https://github.com/gazebosim/gz-plugin/blob/main/Migration.md)
- [gz-rendering](https://github.com/gazebosim/gz-rendering/blob/main/Migration.md)
- [gz-sensors](https://github.com/gazebosim/gz-sensors/blob/main/Migration.md)
- [gz-sim](https://github.com/gazebosim/gz-sim/blob/main/Migration.md)
- [gz-tools](https://github.com/gazebosim/gz-tools/blob/main/Migration.md)
- [gz-transport](https://github.com/gazebosim/gz-transport/blob/main/Migration.md)
- [gz-utils](https://github.com/gazebosim/gz-utils/blob/main/Migration.md)
- [sdformat](https://github.com/gazebosim/sdformat/blob/main/Migration.md)

또한 이 마이그레이션 가이드의 뒷부분에 있는 마이그레이션 포인터는 패키지를 Gazebo와 함께 사용할 준비를 하는 데 도움이 될 것입니다.

일반적으로 말해서, 스택에 작성된 명시적인 tick-tock 로직 덕분에 Harmonic을 사용하는 경우 **대부분의 것**에 대해 Ignition 대응 항목 또는 Gazebo 대응 항목을 계속 사용할 수 있어야 합니다.
Ignition 대응 항목을 사용하면 일반적으로 사용 중단 경고가 발생한다는 점에 유의하십시오.

#### Tick-tocks

다음에 대한 Tick-tock이 구현되었지만, 모든 항목이 사용 중단 경고를 발생시키는 것은 아닙니다.
이러한 tick-tock은 별칭으로 구현되거나, 대신 Gazebo 대응 항목을 대상으로 하는 어떤 종류의 리디렉션 메커니즘(예: 심볼릭 링크, 디렉토리 리타겟, 소스의 문자열 교체)을 가집니다.

또한 소스 코드에서 이러한 tick-tock 대부분은 사용 중단되었음을 알리는 관련 주석이나 `GZ_DEPRECATED()` 매크로 호출이 있습니다.

**Namespaces**

- Python 네임스페이스
    - 예: `ignition.math.XXX` → `gz.math.XXX`
- C++ 네임스페이스
    - 예: `ignition::gazebo::XXX` → `gz::sim::XXX`
- 메시지 네임스페이스 및 패키지
    - 예: `ignition.msgs.XXX` → `gz.msgs.XXX`, `ignition/msgs/header.proto` → `gz/msgs/header.proto`

**Source**

- 공개 헤더의 클래스 이름, 멤버, 함수 및 변수
    - 예: `IgnitionFormatter` → `GzFormatter`
- 공개 헤더
    - 예: `include/ignition` → `include/gz`
- 플러그인
    - 예: `ignition::gazebo::systems::LiftDrag` → `gz::sim::systems::LiftDrag`
- 공유 라이브러리
    - 예: `libignition-gazebo-buoyancy-engine-system.so` → `libgz-sim-buoyancy-engine-system.so`
    - `lib` 및 `.so` 접두사 및 접미사를 제거할 수 있습니다!
      - 예: `libignition-gazebo-buoyancy-engine-system.so` → `gz-sim-buoyancy-engine-system`
- 공개 헤더의 C++ 매크로
    - 예: `IGN_PARTITION` → `GZ_PARTITION`

**CMake and Packaging**

- CMake 매크로/함수
    - 예: `ign_find_package()` → `gz_find_package()`
- CMake 매크로/함수 인수
    - 예: `NO_IGNITION_PREFIX` → `NO_PROJECT_PREFIX`
- CMake 변수*
    - 예: `IgnOGRE2_FOUND` → `GzOGRE2_FOUND`
    - 모든 CMake 변수가 tick-tock 처리되지는 않지만, 다운스트림 라이브러리에서 사용되는 대부분은 처리됩니다.
- `gz_find_package()`로 찾은 CMake 패키지
    - 예: `gz_find_package(IgnCURL)` → `gz_find_package(GzCURL)`
- Debian 패키지
    - 예: `libignition-cmake3-dev` → `libgz-cmake3-dev`

**Misc.**

- 환경 변수 (이름 및 값)
    - 예: `IGN_GAZEBO_RESOURCE_PATH` → `GZ_SIM_RESOURCE_PATH`
- 명령줄
    - 예: `ign` → `gz`, `ign gazebo` → `gz sim`
- GUI QML
    - 예: `IgnSpinBox` → `GzSpinBox`
- 토픽* (일반적으로 테스트에서)
    - 예: `/ignition/XXX` → `/gz/XXX`
    - **참고:** `/gazebo`는 `/sim`으로 마이그레이션되지 **않습니다**.
- GitHub 조직 및 저장소
    - 예: `ignitionrobotics` → `gazebosim` , `ign-cmake` → `gz-cmake`
- GitHub 액션 및 워크플로우
    - 예: `ignition-tooling` → `gazebo-tooling`
- 웹사이트
    - 예: [ignitionrobotics.org](http://ignitionrobotics.org) → [gazebosim.org](http://gazebosim.org)
- SDF 및 launch 태그
    - 예: `<ignition-gui>` → `<gz-gui>`
- SDF 네임스페이스
    - 예: `ignition:type`


#### Hard-tocks

대신 hard-tock 처리된 몇 가지 예외가 있습니다. 즉, Gazebo 대응 항목을 **반드시** 사용해야 합니다.
Ignition 대응 항목을 사용하면 컴파일이나 다른 것이 중단될 가능성이 높습니다 (단순히 문서 변경이 아닌 경우).

**Namespaces**

- Ruby 네임스페이스
    - 예: `ignition/math` → `gz/math`

**Source**

- 설치 공간
    - 예: `install/share/ignition` → `install/share/gz`

**CMake and Packaging**

- `gz-cmake`의 대부분의 포함 가능한 CMake 파일
    - 예: `IgnUtils.cmake` → `GzUtils.cmake`
- Gazebo 라이브러리 CMake 프로젝트 이름
    - 예: `ignition-utils2` → `gz-utils2`
- 내부적으로 사용되는 CMake 변수
    - 예: `IGN_TRANSPORT_VER` → `GZ_TRANSPORT_VER`

**Misc.**

- Gazebo launch 파일 및 태그
    - 예: `sim.ign` → `sim.gzlaunch`, `<ign`  → `<gz`
- 설정 및 로그 경로 (보류 중)
    - 예: `~/.ignition/gui/log` → `~/.gz/gui/log`
    - 일부 설정 경로는 tick-tock 처리되었습니다 (예: `~/.ignition/gazebo/plugins`)
- Fuel URL (보류 중)
    - 예: https://fuel.ignitionrobotics.org
- Fuel 캐시 경로 (역시 보류 중)
    - 예: `~/.ignition/fuel` → `~/.gz/fuel`
- 문서 및 주석의 라이브러리 이름
    - 예: Ignition Gazebo → Gazebo Sim
- `gz-launch` Websocket 서버
    - 예: `ign.js` → `gz.js`

또한 핵심 Gazebo 라이브러리 내부에 있고 다운스트림 라이브러리에서 사용되지 않는 모든 것(예: 헤더 가드, 비공개 헤더 또는 소스, 테스트, 문서)은 hard-tock 처리됩니다.

### Untocks

주로 이전 버전과의 호환성 이유(예: Fortress 지원)로 마이그레이션되지 않은 항목은 극소수입니다.

- Harmonic 이전 릴리스를 대상으로 하는 Gazebo 라이브러리 버전의 브랜치 이름
    - 예: `ign-cmake2`
- 일부 링크
    - 예: [https://osrf-migration.github.io/ignition-gh-pages](https://osrf-migration.github.io/ignition-gh-pages)
- Fuel 사용자 에이전트 관련
    - 예: `X-Ign-Resource-Version`, `IgnitionFuelTools`

## Migration

### Overview

다음 마이그레이션 지침은 대부분의 일반적인 경우를 포괄해야 하는 패키지 마이그레이션 방법에 대한 지침 및 제안일 뿐입니다.

모든 Ignition 대응 항목(`IGN`, `Ign`, `Ignition`, `ign`, `ignition`)을 Gazebo 대응 항목(`GZ`, `Gz`, `gz`)으로 바꾸는 가장 중요한 목표를 명심하십시오.

#### Recommendations

크게 도움이 될 수 있는 것은 다음과 같습니다:

- `regex` 및 `sed`를 충분히 활용하십시오
    - 마이그레이션 가이드에서는 사용할 정규식 표현에 대한 제안을 제공하지만, **바꾸기를 실행하기 전에 항상 검토하십시오**
- 대소문자에 주의하십시오!!
- 편집기를 사용하여 변경 사항을 검토하고, 소스 제어를 사용하여 쉽게 롤백할 수 있도록 하십시오
- 컴파일 경고/오류에 주의하십시오. 이는 일반적으로 (항상 그런 것은 아니지만) 무엇을 바꿔야 하는지에 대한 제안을 제공합니다!
- 확실하지 않은 경우, [추적 이슈](https://github.com/gazebo-tooling/release-tools/issues/698)에 나열된 변경 사항으로 변경 사항을 추적하십시오

또한 Gazebo 스택을 소스에서 빌드하는 경우 깨끗하고 새로운 재빌드 및 설치를 수행해야 합니다.
`build` 및 `install` 디렉토리를 삭제하고 `--merge-install`로 빌드를 실행하십시오.

#### Gotchas

수행해야 할 마이그레이션 노력의 대부분은 주로 Ignition 관련 용어의 인스턴스를 찾아 Gazebo 관련 용어로 바꾸는 것입니다.

그러나 마이그레이션을 처리하는 스크립트를 만드는 것을 어렵게 만드는 (오류 또는 버그로 이어지는) 많은 예외적인 경우가 있으므로 마이그레이션 단계를 제안하기 전에 이러한 경우 중 일부를 아는 것이 좋습니다.

- 대소문자를 존중하지 않음
    - 예: `IGNITION_ADD_PLUGIN` → `gz_ADD_PLUGIN`
- 탐욕스러운 일치
    - 예: `Align` → `Algz`, `unsigned` → `unsgzed`, `signal` → `sgzal`
- "gazebo"를 "sim"으로 마이그레이션하지 않음
    - 예: `ign-gazebo` → `gz-gazebo` (이것은 `gz-sim`이어야 합니다)
- `ignition` 전에 `ign` 일치
    - 예: `ignition-cmake3` → `gzition-cmake3`
- 마이그레이션되어서는 안 되는 라이브러리 마이그레이션
    - 예: `ignition-cmake2` → `gz-cmake2`
- 문법
    - 예: "an Ignition library" → "**an** Gazebo library"

또한 다음 동작에 유의하십시오:

- 기존 설정 파일은 덮어쓰지 않으므로 이전 설정을 사용하거나 사용자 지정 위치에 설정 파일이 있는 경우, 설정 파일의 Ignition 대응 참조를 수동으로 마이그레이션하여 적절한 대응 항목을 가리키도록 해야 할 수 있습니다.
  - 예를 들어, `gz-sim` 파일을 대상으로 하는 이전 설정 파일은 `ignition::gazebo::systems::Physics` 플러그인을 사용할 수 있습니다.
    해당 참조가 아직 마이그레이션되지 않았기 때문에 사용 중단 경고가 발생할 수 있습니다.

- 설치 공간이 변경되었으므로 `ignition`을 포함하는 하드 코딩된 위치가 있는 경우 `gz`로 마이그레이션해야 할 수 있습니다.

### Migration Steps

각 개별 섹션 내의 단계를 순서대로 실행하는 것이 중요합니다! 단계는 일반적으로 구체적인 것에서 일반적인 것으로 진행됩니다.

#### Migrate Files and File References

1. 패키지 루트에서 `ign(ition)?[_|-]gazebo` (대소문자 구분 없음) 패턴과 일치하는 파일 및 디렉토리를 찾고, `ign` / `ignition`을 `gz`로, `gazebo`를 `sim`으로 적절하게 마이그레이션합니다.
2. 패키지 루트에서 `ign` / `ignition` (대소문자 구분 없음)이 있는 파일 및 디렉토리를 찾고, 대소문자를 일치시키면서 `gz`로 이동합니다.
3. 패키지 내에서 (1) 및 (2)의 마이그레이션된 파일 및 디렉토리에 대한 모든 내부 참조를 업데이트합니다.

#### Migrate CMake

`CMakeLists.txt` 파일 (및 소스 파일의 참조!)에서:

**Variables and macro/function calls**

```
Find: IGN(ITION)?_GAZEBO
Replace: GZ_SIM

Find: ign(ition)?_gazebo
Replace: gz_sim

Find: IGN(ITION)?_
Replace: GZ_

Find: ign(ition)?_
Replace: gz_
```

**Includes**

```
Find: include\(Ign
Replace: include(Gz

Find: include\(ign
Replace: include(gz

Find: gz_find_package\(ign-
Replace: gz_find_package(gz-

Find: gz_find_package\(Ign(ition)?
Replace: gz_find_package(Gz-
```

**Project Names**

```
Find: ignition-gazebo
Replace: gz-sim

Find: ignition-
Replace: gz-
```

**참고:** 때때로 CMake 인수가 소스 파일로 전달되므로 해당 인수도 적절하게 마이그레이션해야 합니다.

#### Migrate Macros and Environment Variables

- [환경 변수 마이그레이션의 유용한 목록](https://github.com/gazebo-tooling/release-tools/issues/734)
- [매크로 마이그레이션의 유용한 목록](https://github.com/gazebo-tooling/release-tools/issues/737) (토글된 드롭다운 블록 참조, 일부는 건너뜁니다!)

소스 매크로 및 환경 변수 마이그레이션

```
Find: IGN(ITION)?_GAZEBO
Replace: GZ_SIM

Find: ign(ition)?_gazebo
Replace: gz_sim

Find: IGN(ITION)?_
Replace: GZ_

Find: ign(ition)?_
Replace: gz_
```

환경 변수의 경우 매크로와 동일한 접근 방식을 사용할 수 있지만 환경 변수에 저장된 값(예: 경로)에 주의하십시오!

또한 로깅 매크로도 마이그레이션되었습니다! 사용 부분을 마이그레이션하십시오!

- `ignerr` -> `gzerr`
- `ignwarn` -> `gzwarn`
- `ignmsg` -> `gzmsg`
- `igndbg` -> `gzdbg`
- `ignlog` -> `gzlog`
- `ignLogInit` -> `gzLogInit`
- `ignLogClose` -> `gzLogClose`
- `ignLogDirectory` -> `gzLogDirectory`

#### Migrate SDF

`.sdf` 파일에서:

```
Find: <ignition
Replace: <gz

Find: </ignition
Replace: </gz

Find: ignition:
Replace: gz:
```

몇 가지 예:

- `<gz:odometer`
- `<gz-gui`

#### Migrate Plugins and Shared Libraries

플러그인 파인더는 파일 이름에서 `lib` 및 `.so`가 제거된 경우에도 플러그인을 찾을 수 있습니다.

`.sdf` 파일 및 소스 파일 (예: `.cc`)에서:

```
Find: (lib)?ign(ition)?-gazebo([^. ]*)\.so
Replace: gz-sim\3

Find: (lib)?ign(ition)?([^. ]*)\.so
Replace: gz\3

Find: ignition::gazebo
Replace: gz::sim

Find: ignition::
Replace: gz::
```

#### Migrate Bindings

Python 파일 (예: `.py`)에서

```
Find: ignition.gazebo
Replace: gz.sim

Find: ignition.
Replace: gz.
```

Ruby 파일 (예: `.i`, `.rb`)에서

```
Find: ign(ition)?/
Replace: gz/
```

#### Migrate Messages

메시지 정의에서

```
Find: ign(ition)?\.gazebo
Replace: gz.sim

Find: ign(ition)?/gazebo
Replace: gz/sim

Find: ign(ition)?\.
Replace: gz.

Find: ign(ition)?/
Replace: gz/
```

#### Migrate Headers and Sources

모든 곳에서 전면적인 검사 (이들을 검토하는 데 특별히 주의하십시오!)

**Headers**

```
Find: #include\s*([<"])ign(ition)?/gazebo
Replace: #include \1gz/sim

Find: #include\s*([<"])ign(ition)?/
Replace: #include \1gz/

// Note: You should be wary of the IGNITION GAZEBO case for the following
// and adjust accordingly
Find: #([^\s]*)\s+(.*)IGN(?:ITION)?_(.*)_(H+)_(.*)$
Replace: #$1 $2GZ_$3_$4_$5

Find: #endif\s*// GZ(.*)_H
Replace: #endif  // GZ$1_H
```

**Namespaces**

```
Find: namespace\s*ignition
Replace: namespace gz

Find: namespace\s*gazebo
Replace: namespace sim

Find: ignition::gazebo
Replace: gz::sim

Find: Ignition::Gazebo
Replace: Gz::Sim

Find: ignition::
Replace: gz::

Find: Ignition::
Replace: Gz::
```

**Everything Else**

수동으로 검사하고 싶을 수 있습니다:

- `Ign`
- `ignition`

또한 `gazebo`의 특정 인스턴스(일반적으로 API의 일부)는 대신 `sim`을 사용해야 한다는 점에 유의하십시오.

#### Migrate Your CLI Usage

이전에 사용했던 곳:

```
ign gazebo shapes.sdf
```

이제 다음을 사용해야 합니다:

```
gz sim shapes.sdf
```

`gazebo` 동사는 **사용 중단(deprecated)**되었습니다.

#### Helpful CLI Redirection

`Harmonic`과 `Fortress`의 병렬 설치를 지원하기 위해 `ign` CLI 실행 파일(및 `gazebo` 동사)은 `Harmonic`용으로 설치되지 않습니다.
이는 `Fortress`가 `Harmonic`과 함께 설치되지 않는 한 CLI 사용을 마이그레이션해야 함을 의미합니다.
`ign` 대신 `gz`를 사용하고 `gazebo` 대신 `sim`을 사용하십시오.

다음 스크립트를 `~/.bashrc` 파일에 추가하여 모든 `ign` 호출을 `gz`로 리디렉션할 수 있으므로 모든 스크립트(아직 `ign`을 사용 중인)를 마이그레이션할 필요는 없지만, 마이그레이션을 수행하는 것이 여전히 권장됩니다.

```shell
ign() {
  if which ign &> /dev/null; then
    $(which ign) "$@"
  else
    if which gz &> /dev/null; then
      echo "[DEPRECATED] ign is deprecated! Please use gz instead!"
      if [ "$1" = "gazebo" ]; then
        echo "[DEPRECATED] The gazebo verb is deprecated! Please use sim instead!"
        shift
        $(which gz) sim "$@"
      else
        $(which gz) "$@"
      fi
    else
      echo "[ERROR] It seems like you don't have Gazebo installed!"
      return 1
    fi
  fi
}
```

### Double-Check

남아있는 것이나 잘못 마이그레이션된 인스턴스가 있는지 확인하는 데 유용할 수 있습니다.

이들을 **대소문자 구분 없이** 일치시켜야 합니다.

**Errors**

- `gz-gazebo`
- `gzition`
- `an gz`

**Remaining Ign**

- `\.ign(ition)?`
- `ign(ition)?[-_]`

## Additional Packages

이 섹션에서는 다른 일부 Gazebo 관련 패키지에서 발생한 변경 사항을 자세히 설명합니다.

### ros_gz

`ros_ign`이 `ros_gz`로 이름이 변경되었습니다.
Harmonic 이전 버전의 Gazebo가 의존하지 않는 `ign` 또는 `ignition`에 대한 모든 내부 참조가 마이그레이션되었습니다.

사용자 지정 sim 버전 또는 sim 인수로 `ros_gz` 데모를 실행하려면 `gz_version` 및 `gz_args` 시작 매개변수를 사용하십시오.
`ign_version` 시작 매개변수를 사용하는 경우 `ign_args` 시작 매개변수도 명시적으로 설정해야 합니다.