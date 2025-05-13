#!/bin/bash

# 디렉토리 생성
mkdir -p docs/img
mkdir -p docs/images
mkdir -p docs/tutorials/actors
mkdir -p docs/tutorials/building_robot
mkdir -p docs/tutorials/fuel_insert
mkdir -p docs/tutorials/gui
mkdir -p docs/tutorials/moving_robot
mkdir -p docs/tutorials/ros2_integration
mkdir -p docs/tutorials/sdf_worlds
mkdir -p docs/tutorials/sensors
mkdir -p docs/tutorials/spawn_urdf

# 이미지 복사
cp -r img/* docs/img/
cp -r images/* docs/images/

# 튜토리얼 디렉토리 이미지 복사 (tutorials 디렉토리에 이미지가 있는 경우)
for dir in tutorials/*; do
  if [ -d "$dir" ]; then
    dir_name=$(basename "$dir")
    cp -r "$dir"/* "docs/tutorials/$dir_name/" 2>/dev/null || true
  fi
done

echo "이미지 복사 완료!" 