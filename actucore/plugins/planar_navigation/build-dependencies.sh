#!/usr/bin/env bash
# Dockerfile.jetson invokes this inside the standard image, never on the host.
set -eo pipefail
[[ "${BUILD_JOBS:-2}" =~ ^[1-9][0-9]*$ ]] || { echo 'invalid BUILD_JOBS' >&2; exit 2; }
test -f /opt/ros/humble/install/setup.bash
source /opt/ros/humble/install/setup.bash
lock_file="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/sources.lock"
workspace=/opt/actucore_navigation_ws
mkdir -p "${workspace}/src"
while read -r name url revision; do
    [[ -z "$name" || "$name" == \#* ]] && continue
    [[ "$name" =~ ^[a-z0-9_]+$ && "$url" == https://github.com/*.git && "$revision" =~ ^[0-9a-f]{40}$ ]] || exit 2
    git init "${workspace}/src/${name}"
    git -C "${workspace}/src/${name}" remote add origin "${GIT_MIRROR_PREFIX:-}${url}"
    fetched=false
    for attempt in 1 2 3; do
        if timeout 120 git -C "${workspace}/src/${name}" fetch --depth 1 origin "$revision"; then
            fetched=true
            break
        fi
    done
    "$fetched" || exit 1
    git -C "${workspace}/src/${name}" checkout --detach FETCH_HEAD
    test "$(git -C "${workspace}/src/${name}" rev-parse HEAD)" = "$revision"
done < "$lock_file"

# Older Humble headers use .h; forwarding aliases do not replace implementations.
while IFS= read -r header; do
    alias_header="${header%.h}.hpp"
    if [ ! -e "$alias_header" ]; then
        printf '#include "%s"\n' "$(basename "$header")" > "$alias_header"
    fi
done < <(find /opt/ros/humble/install/include -path '*tf2*' -name '*.h')

cd "$workspace"
packages=(behaviortree_cpp_v3 angles smclib bond bondcpp diagnostic_updater map_msgs laser_geometry
          nav2_common nav2_msgs nav2_util nav2_core nav2_voxel_grid nav2_costmap_2d
          nav2_behavior_tree nav2_bt_navigator nav2_controller nav2_planner nav2_navfn_planner
          nav2_lifecycle_manager nav2_amcl nav2_map_server nav2_regulated_pure_pursuit_controller
          nav_2d_msgs nav_2d_utils slam_toolbox)
missing=()
for package in "${packages[@]}"; do
    if ros2 pkg prefix "$package" >/dev/null 2>&1; then
        echo "[navigation] reuse base package: $package"
    else
        missing+=("$package")
    fi
done
if [ "${#missing[@]}" -eq 0 ]; then
    mkdir -p install
    printf 'source /opt/ros/humble/install/setup.bash\n' > install/setup.bash
else
    CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS:-2}" MAKEFLAGS="-j${BUILD_JOBS:-2}" \
      colcon build --merge-install --executor sequential --packages-select "${missing[@]}" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DBUILD_EXAMPLES=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_TOOLS=OFF
fi
source install/setup.bash
for package in "${packages[@]}"; do ros2 pkg prefix "$package" >/dev/null; done
cp "$lock_file" sources.lock
dpkg-query -W > dependency-versions.txt
rm -rf /opt/actucore_navigation_ws/src /opt/actucore_navigation_ws/build /opt/actucore_navigation_ws/log
