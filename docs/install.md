# Gazebo Harmonic

Gazebo Harmonic은 Gazebo의 8번째 주요 릴리스입니다. 이것은 장기 지원 릴리스입니다.

## Binary installation instructions

바이너리 설치는 Gazebo를 설치하는 권장 방법입니다.

 * [Binary Installation on Ubuntu](install_ubuntu)
 * [Binary Installation on macOS](install_osx)
 * [Binary Installation on Windows](install_windows)

## Source Installation instructions

소스 설치는 Gazebo의 소스 코드를 변경하려는 (고급) 사용자에게 권장됩니다.

 * [Source Installation on Ubuntu](install_ubuntu_src)
 * [Source Installation on macOS](install_osx_src)
 * [Source Installation on Windows](install_windows_src)

## Harmonic Libraries

Harmonic 컬렉션은 다양한 Gazebo 라이브러리로 구성됩니다. 이 컬렉션은 모든 라이브러리가 서로 호환되며 함께 사용될 수 있도록 보장합니다.

이 라이브러리 버전 목록은 출시일까지 변경될 수 있습니다.

| Library name       | Version       |
| ------------------ |:-------------:|
|   gz-cmake         |       3.x     |
|   gz-common        |       5.x     |
|   gz-fuel-tools    |       9.x     |
|   gz-sim           |       8.x     |
|   gz-gui           |       8.x     |
|   gz-launch        |       7.x     |
|   gz-math          |       7.x     |
|   gz-msgs          |      10.x     |
|   gz-physics       |       7.x     |
|   gz-plugin        |       2.x     |
|   gz-rendering     |       8.x     |
|   gz-sensors       |       8.x     |
|   gz-tools         |       2.x     |
|   gz-transport     |      13.x     |
|   gz-utils         |       2.x     |
|   sdformat         |      14.x     |

## Supported platforms

다음은 **공식적으로** 지원되는 플랫폼입니다:

* Ubuntu Jammy (amd64에서)
* Ubuntu Noble (amd64에서)

**최선 지원**으로 지원되는 플랫폼에는 arm 아키텍처, Windows 및 macOS가 포함됩니다.
전체 상태는 [this ticket](https://github.com/gazebo-tooling/release-tools/issues/597)을 참조하십시오.