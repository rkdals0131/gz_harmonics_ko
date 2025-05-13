# Binary Installation on Ubuntu

Harmonic 바이너리는 Ubuntu Jammy (22.04) 및 Ubuntu Noble (24.04)용으로 제공됩니다.
Harmonic 바이너리는 packages.osrfoundation.org 저장소에서 호스팅됩니다.
모든 것을 설치하려면 메타패키지 `gz-harmonic`을 설치할 수 있습니다.

<div class="warning">
경고: gazebo-classic (예: `gazebo11`) 사용자: 기본적으로 `gz-harmonic`은 `gazebo11`과 함께 설치할 수 없습니다. 마이그레이션을 용이하게 하려면 <a href="https://gazebosim.org/docs/harmonic/install_gz11_side_by_side">Installing Gazebo11 side by side with new Gazebo</a>에 자세히 설명된 지침을 사용하여 수행할 수 있습니다.
</div>

먼저 몇 가지 필요한 도구를 설치합니다:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

그런 다음 Gazebo Harmonic을 설치합니다:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

모든 라이브러리를 사용할 준비가 되었고 `gz sim` 앱을 실행할 준비가 되었습니다.

Gazebo 사용을 시작하려면 [Getting started](getstarted) 페이지로 돌아가십시오!

## Uninstalling binary install

Gazebo를 제거하거나 바이너리에서 라이브러리를 이미 설치한 후 소스 기반 설치로 전환해야 하는 경우 다음 명령을 실행하십시오:

```bash
sudo apt remove gz-harmonic && sudo apt autoremove
```

## Troubleshooting

[Troubleshooting](troubleshooting.md#ubuntu)를 참조하십시오.