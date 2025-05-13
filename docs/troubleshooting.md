# Troubleshooting

## Gazebo libraries are not found
다음과 같은 오류 메시지가 표시되는 경우:

```bash
I cannot find any available 'gz' command:
	* Did you install any Gazebo library?
	* Did you set the GZ_CONFIG_PATH environment variable?
	    E.g.: export GZ_CONFIG_PATH=$HOME/local/share/gz
```

환경 변수를 설정해야 합니다:

```
# <path_to_install_dir>을 Gazebo 설치 디렉토리로 바꾸세요
GZ_CONFIG_PATH=<path_to_install_dir>/share/gz/
```

## macOS

### Maximum number of open files reached `ulimit` error
homebrew를 사용하여 설치할 때 다음과 같은 오류 메시지가 표시될 수 있습니다:

```bash
Error: The maximum number of open files on this system has been reached. Use 'ulimit -n' to increase this limit.
```

오류 메시지에서 제안한 대로 아래 명령을 실행하고 출력을 확인하세요. 기본값은 `256`으로 설정되어 너무 낮습니다.
```bash
ulimit -n
```

다음 명령을 실행하여 열린 파일 `ulimit`을 늘린 다음 homebrew 설치를 진행하세요:
```bash
ulimit -n 10240
```

### Unable to find `urdf_model.h` error
모든 종속성을 설치하고 빌드 프로세스를 시작한 후 다음과 같은 오류가 발생할 수 있습니다:

```bash
/Users/user/harmonic_ws/src/sdformat/src/parser_urdf.cc:30:10: fatal error: 'urdf_model/model.h' file not found
#include <urdf_model/model.h>
         ^~~~~~~~~~~~~~~~~~~~
1 error generated.
make[2]: *** [src/CMakeFiles/sdformat9.dir/parser_urdf.cc.o] Error 1
make[1]: *** [src/CMakeFiles/sdformat9.dir/all] Error 2
make: *** [all] Error 2
Failed   <<< sdformat9	[ Exited with code 2 ]
```

먼저 다음을 실행하여 `urdfdom` 및 `urdfdom_headers`가 설치되었는지 확인하세요:

```bash
brew install urdfdom urdfdom_headers
```

그래도 오류가 지속되면 다음을 실행하여 `urdfdom`의 내부 버전으로 컴파일하세요:

```bash
colcon build --cmake-args -DUSE_INTERNAL_URDF=ON --merge-install
```

이 명령은 `urdfdom`의 시스템 설치를 무시하고 대신 내부 버전을 사용합니다.

### Unable to load .dylib file
`gz sim -s` 명령을 실행할 때 아래와 같은 오류가 나타날 수 있습니다:

```bash
Error while loading the library [/Users/harmonic/harmonic_ws/install/lib//libgz-physics6-dartsim-plugin.6.dylib]: dlopen(/Users/harmonic/harmonic_ws/install/lib//libgz-physics6-dartsim-plugin.6.dylib, 5): Library not loaded: @rpath/libIrrXML.dylib
  Referenced from: /usr/local/opt/assimp/lib/libassimp.5.dylib
  Reason: image not found
[Err] [Physics.cc:275] Unable to load the /Users/harmonic/harmonic_ws/install/lib//libgz-physics6-dartsim-plugin.6.dylib library.
Escalating to SIGKILL on [Gazebo Sim Server]
```

이 문제는 OSX System Integrity Protection (SIP)과 관련이 있습니다. 해결 방법은 다른 ruby로 `gz`를 실행한 다음 해당 ruby가 로드되었는지 확인하는 것입니다.

```bash
brew install ruby

# ~/.bashrc에 다음을 추가합니다
export PATH=/usr/local/Cellar/ruby/2.6.5/bin:$PATH

# 터미널에서 ~/.bashrc를 소스합니다
. ~/.bashrc
```

### No rule to make target `'/usr/lib/libm.dylib', needed by 'lib/libgz-physics6-dartsim-plugin.6.1.0.dylib'. Stop.`
`brew outdated` 다음에 `brew upgrade`를 실행하면 일부 문제가 해결될 수 있습니다.

## Ubuntu

### Out of memory issues

Gazebo 컴파일 중, 특히 gz-physics 컴파일 중에 메모리 부족 문제가 발생할 수 있습니다. 메모리 부족 문제를 방지하려면 작업 수를 제한할 수 있습니다:

```bash
MAKEFLAGS="-j<Number of jobs> " colcon build --executor sequential
```

### Problems with dual Intel and Nvidia GPU systems

듀얼 Intel/Nvidia 시스템을 사용하는 경우 시뮬레이터가 Nvidia GPU 대신 Intel에서 실행될 수 있습니다. 버그는 다양하지만 그림자 문제, 잘못된 레이저 스캔 또는 기타 렌더링 관련 문제가 있을 수 있습니다.

#### prime-select command line tool

하이브리드 Intel/Nvidia 시스템은 명령줄 도구 prime-select를 사용하여 구성할 수 있습니다.
한 가지 옵션은 항상 Nvidia를 사용하는 것입니다:

    sudo prime-select nvidia
    # 사용자 세션 로그아웃 후 다시 로그인

다른 옵션은 OpenGL 애플리케이션에 대한 렌더 오프로드를 Nvidia를 사용하도록 구성하는 것입니다. 이는 X 화면과 모든 일반 애플리케이션은 Intel GPU에서 처리되지만 터미널에서 시작하는 모든 OpenGL 애플리케이션(Gazebo 포함)은 Nvidia GPU에서 렌더링된다는 것을 의미합니다.

    # 변경 사항을 영구적으로 적용하려면 .bashrc에 줄을 추가하세요
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    # 사용자 세션 로그아웃 후 다시 로그인

#### nvidia-settings GUI tool

nvidia-settings는 Nvidia 그래픽 카드 옵션 구성을 돕는 GUI 프로그램이며 하이브리드 Intel/Nvidia에 대한 일부 제어 기능을 포함합니다:

"PRIME Profiles" 섹션을 사용하여 "NVIDIA (Performance Mode)"를 선택하여 Nvidia 카드가 모든 GUI 애플리케이션을 제어하도록 선택할 수 있습니다.

"Application Profiles"는 애플리케이션별로 Nvidia GPU 사용을 제어할 수 있습니다.

### Unable to create the rendering window

"Unable to create the rendering window"와 같은 오류가 발생하는 경우 오래된 OpenGL 버전을 사용하고 있을 수 있습니다. Gazebo Sim은 기본적으로 Ogre 2 렌더링 엔진을 사용하며, 이는 3.3보다 높은 OpenGL 버전, 가급적 4.3 이상이 필요합니다.

이는 `~/.gz/rendering/ogre2.log`에서 Ogre 2 로그를 확인하여 확인할 수 있으며, 다음과 같은 오류가 있어야 합니다:

    "OGRE EXCEPTION(3:RenderingAPIException): OpenGL 3.3 is not supported. Please update your graphics card drivers."

다음을 실행하여 OpenGL 버전을 확인할 수도 있습니다:

    glxinfo | grep "OpenGL version"

Ogre 2 지원을 활성화하려면 컴퓨터의 OpenGL 버전을 업데이트해야 합니다. Ogre 로그에서 제안된 대로 그래픽 카드 드라이버를 업데이트해야 할 수 있습니다.

Ogre 2로 Gazebo를 실행할 때 여전히 OpenGL 문제가 발생하는 경우 특정 확장이 드라이버에서 지원되지 않거나 가상 머신 내에서 실행 중일 수 있습니다. 이 경우 DRI를 비활성화해 볼 수 있습니다:

    export LIBGL_DRI3_DISABLE=1

또는 소프트웨어 렌더링을 강제합니다

    export LIBGL_ALWAYS_SOFTWARE=1

MESA 드라이버를 사용하는 경우 OpenGL 버전을 재정의해 볼 수도 있습니다

    # GL 버전을 3.3으로 재정의
    export MESA_GL_VERSION_OVERRIDE=3.3

    # 또는 Core + Forward compatible 프로필을 3.3으로 선택
    export MESA_GL_VERSION_OVERRIDE=3.3FC

자세한 내용은 [MESA environment variable documentation](https://docs.mesa3d.org/envvars.html#envvar-MESA_GL_VERSION_OVERRIDE)을 참조하십시오.

osrfoundation 저장소의 Ogre 2 deb는 deb 패키징에 필요하고 Ogre 1.x와 함께 설치할 수 있도록 변경된 Ogre의 `v2-3` 브랜치 포크에서 빌드됩니다. 코드는 여기에서 찾을 수 있습니다:

https://github.com/osrf/ogre-2.3-release

하지만 문제 없이 Ogre 1을 사용할 수 있어야 합니다. Ogre 2 대신 Ogre 1로 실행하여 작동하는지 확인할 수 있습니다. 예:

    gz sim -v 3 shapes.sdf --render-engine ogre

로드되면 Ogre 1으로 Gazebo를 계속 사용할 수 있습니다. `--render-engine ogre` 옵션을 사용하면 됩니다.

### Wayland issues

Gazebo에서 Ogre와 Qt의 상호 작용 문제로 인해 wayland가 제대로 작동하지 않는 문제가 있습니다. 아래와 같은 오류 메시지가 표시될 수 있습니다:

```
Unable to create the rendering window: OGRE EXCEPTION(3:RenderingAPIException): currentGLContext was specified with no current GL context in GLXWindow::create at ./RenderSystems/GL3Plus/src/windowing/GLX/OgreGLXWindow.cpp (line 165)
```

해결 방법은 `QT_QPA_PLATFORM=xcb`를 설정하는 것입니다. 예:

```
QT_QPA_PLATFORM=xcb gz sim -v 4 shapes.sdf
```

시도해 볼 수 있는 또 다른 해결 방법은 `WAYLAND_DISPLAY` 환경 변수를 설정 해제하여 Gazebo가 XWayland로 시작되도록 하는 것입니다. 예:

```sh
env -u WAYLAND_DISPLAY gz sim -v 4 shapes.sdf
```

### EGL warnings

시작 시 Gazebo는 아래와 같은 EGL 경고 메시지를 출력합니다:

```
libEGL warning: DRI2: failed to create dri screen
```

이 메시지는 Ogre 2가 초기화 시 EGL 지원을 쿼리하기 위해 장치를 열거할 때 출력됩니다. 이 경고는 무시해도 괜찮습니다. Gazebo는 문제 없이 계속 작동해야 합니다.

### Network Configuration Issue
네트워크 구성이 잘못되면 Gazebo 창이 열린 후 응답하지 않을 수 있습니다. 이 문제는 ```gz sim -v 4 shapes.sdf```를 실행하고 다음 출력을 확인하여 진단할 수 있습니다:

```[GUI] [Dbg] [Gui.cc:343] GUI requesting list of world names. The server may be busy downloading resources. Please be patient.```

이 문제를 해결하려면 [ROS Enable Multicast](https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html#enable-multicast)의 단계를 따라 멀티캐스트를 활성화하세요.


## Windows

### VisualStudioVersion is not set, please run within a Visual Studio Command Prompt.
Gazebo를 컴파일하려고 할 때 프롬프트에 다음과 같은 오류가 표시될 수 있습니다:

    VisualStudioVersion is not set, please run within a Visual Studio Command Prompt.

이 경우 다음 명령 중 하나를 실행하세요:
 - CMD
```bash
    "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64
```

 - PowerShell:
```bash
pushd "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools"
cmd /c "VsDevCmd.bat&set" |
foreach {
  if ($_ -match "=") {
    $v = $_.split("="); set-item -force -path "ENV:\$($v[0])"  -value "$($v[1])"
  }
}
popd
```

### Many errors from setuptools when running colcon
패키지를 빌드하기 위해 `colcon`을 호출할 때 다음과 유사한 Python 오류 벽에 직면할 수 있습니다:

```
> colcon graph
Traceback (most recent call last):
  File "<string>", line 1, in <module>
ModuleNotFoundError: No module named 'setuptools.extern'
[10.385s] colcon.colcon_core.package_identification ERROR Exception in package identification extension 'python_setup_py' in 'conda\Lib\site-packages\adodbapi': Command '['D:\\programovani\\gz-ws\\conda\\python.exe', '-c', "import sys;from setuptools.extern.packaging.specifiers import SpecifierSet;from distutils.core import run_setup;dist = run_setup(    'setup.py', script_args=('--dry-run',), stop_after='config');skip_keys = ('cmdclass', 'distclass', 'ext_modules', 'metadata');data = {    key: value for key, value in dist.__dict__.items()     if (        not key.startswith('_') and         not callable(value) and         key not in skip_keys and         key not in dist.display_option_names    )};data['metadata'] = {    k: v for k, v in dist.metadata.__dict__.items()     if k not in ('license_files', 'provides_extras')};sys.stdout.buffer.write(repr(data).encode('utf-8'))"]' returned non-zero exit status 1.
```

메시지는 매우 암호화되어 있으며 근본 원인을 지적하지 않습니다. 근본 원인은 Gazebo 소스가 포함된 `src` 디렉토리와 동일한 디렉토리에 conda env 디렉토리를 만들었을 가능성이 있다는 것입니다(기본값이 아닌 대상에 env 디렉토리를 만들기 위해 `conda create --prefix ...`를 사용했을 가능성이 높습니다).

해결 방법은 conda env 디렉토리를 한 수준 위로 이동하는 것입니다.

예를 들어, 이것이 문제가 있는 폴더 구조입니다:

```
gz-ws\
  conda\  # The conda env
  src\    # The Gazebo sources
    gz-sim\
```

이 문제를 해결하려면 구조를 다음과 같이 변경하세요:

```
gz-ws\
  src\
    gz-sim\
conda\
```