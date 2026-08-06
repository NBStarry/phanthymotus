#!/usr/bin/env bash
set -euo pipefail

g1_host="${G1_HOST:-g1-sh-wifi}"
container_name="phanthy-nav2-shadow"
require_n5_protocol="${REQUIRE_N5_PROTOCOL:-0}"
proposal_driver_node="${PROPOSAL_DRIVER_NODE:-}"
proposal_driver_standby="${PROPOSAL_DRIVER_STANDBY:-0}"
legacy_driver_input_upgrade_source_audit="${LEGACY_DRIVER_INPUT_UPGRADE_SOURCE_AUDIT:-0}"
if [[ "${require_n5_protocol}" != "0" && "${require_n5_protocol}" != "1" ]]; then
  echo "ERROR=REQUIRE_N5_PROTOCOL must be 0 or 1" >&2
  exit 2
fi
if [[ "${legacy_driver_input_upgrade_source_audit}" != "0" && \
      "${legacy_driver_input_upgrade_source_audit}" != "1" ]]; then
  echo "ERROR=LEGACY_DRIVER_INPUT_UPGRADE_SOURCE_AUDIT must be 0 or 1" >&2
  exit 2
fi
if [[ -n "${proposal_driver_node}" && \
      ! "${proposal_driver_node}" =~ ^[A-Za-z0-9_]+$ ]]; then
  echo "ERROR=PROPOSAL_DRIVER_NODE must be a ROS node basename" >&2
  exit 2
fi
if [[ "${proposal_driver_standby}" != "0" && \
      "${proposal_driver_standby}" != "1" ]]; then
  echo "ERROR=PROPOSAL_DRIVER_STANDBY must be 0 or 1" >&2
  exit 2
fi
if [[ "${proposal_driver_standby}" == "1" && \
      -z "${proposal_driver_node}" ]]; then
  echo "ERROR=PROPOSAL_DRIVER_STANDBY requires PROPOSAL_DRIVER_NODE" >&2
  exit 2
fi

ssh_opts=(
  -o ClearAllForwardings=yes
  -o BatchMode=yes
  -o ConnectTimeout=8
)

set +e
container_running="$(ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker container inspect --format '{{.State.Running}}' ${container_name}" 2>&1)"
inspect_rc=$?
set -e
if [[ "${inspect_rc}" == "255" ]]; then
  printf '%s\n' "${container_running}" >&2
  echo "ERROR=SSH target ${g1_host} is unreachable" >&2
  exit 1
fi
if [[ "${inspect_rc}" != "0" ]]; then
  printf '%s\n' "${container_running}" >&2
  echo "ERROR=remote container ${container_name} does not exist" >&2
  exit 1
fi
if [[ "${container_running}" != "true" ]]; then
  echo "ERROR=remote container ${container_name} is not running" >&2
  ssh "${ssh_opts[@]}" "${g1_host}" \
    "docker container inspect --format 'status={{.State.Status}} exit={{.State.ExitCode}} oom={{.State.OOMKilled}} started={{.State.StartedAt}} finished={{.State.FinishedAt}}' ${container_name}" || true
  ssh "${ssh_opts[@]}" "${g1_host}" \
    "docker logs --tail 40 ${container_name} 2>&1" || true
  exit 1
fi
echo "G1_NAV2_CONTAINER=running"

runtime_audit='
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  set -u
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

  topics="$(ros2 topic list --no-daemon --spin-time 8)"
  if grep -Fxq "/cmd_vel" <<<"${topics}"; then
    echo "ERROR=root /cmd_vel exists" >&2
    exit 1
  fi
  echo "G1_NAV2_ROOT_CMD_VEL=absent"

  required_topics=(
    /ubuntu/navigation/nav2/odom
    /ubuntu/navigation/nav2/scan
    /ubuntu/navigation/nav2/cmd_vel_raw
    /ubuntu/navigation/nav2/cmd_vel_shadow
    /ubuntu/navigation/nav2/command
    /ubuntu/navigation/nav2/status
  )
  for topic in "${required_topics[@]}"; do
    if ! grep -Fxq "${topic}" <<<"${topics}"; then
      echo "ERROR=required topic ${topic} is absent" >&2
      exit 1
    fi
    echo "G1_NAV2_TOPIC=${topic}"
  done

  endpoint_rows() {
    local info="$1"
    local node=""
    local node_namespace=""
    local endpoint_topic_type=""
    local endpoint_type=""
    local line=""
    while IFS= read -r line; do
      case "${line}" in
        "Node name: "*) node="${line#Node name: }" ;;
        "Node namespace: "*) node_namespace="${line#Node namespace: }" ;;
        "Topic type: "*) endpoint_topic_type="${line#Topic type: }" ;;
        "Endpoint type: "*)
          endpoint_type="${line#Endpoint type: }"
          printf "%s|%s|%s|%s\n" \
            "${endpoint_type}" "${endpoint_topic_type}" \
            "${node_namespace}" "${node}"
          ;;
      esac
    done <<<"${info}"
  }

  shadow_endpoints_ready=0
  for shadow_attempt in {1..10}; do
    shadow_info="$(ros2 topic info --verbose /ubuntu/navigation/nav2/cmd_vel_shadow)"
    shadow_rows="$(endpoint_rows "${shadow_info}")"
    shadow_twist_publishers=0
    shadow_internal_twist_subscribers=0
    shadow_foreign_twist_subscribers=""
    shadow_bus_observers=0
    shadow_unexpected_endpoints=""
    while IFS="|" read -r endpoint_type endpoint_topic_type \
        endpoint_namespace endpoint_node; do
      [[ -n "${endpoint_type}" ]] || continue
      if [[ "${endpoint_type}" == "PUBLISHER" ]]; then
        if [[ "${endpoint_topic_type}" == "geometry_msgs/msg/Twist" && \
              "${endpoint_node}" == "velocity_smoother" ]]; then
          ((shadow_twist_publishers += 1))
        else
          shadow_unexpected_endpoints+="publisher:${endpoint_topic_type}:${endpoint_namespace}/${endpoint_node},"
        fi
      elif [[ "${endpoint_type}" == "SUBSCRIPTION" ]]; then
        if [[ "${endpoint_topic_type}" == "geometry_msgs/msg/Twist" ]]; then
          if [[ "${endpoint_node}" == "g1_nav2_navigation_command" ]]; then
            ((shadow_internal_twist_subscribers += 1))
          else
            shadow_foreign_twist_subscribers+="${endpoint_namespace}/${endpoint_node},"
          fi
        elif [[ "${endpoint_topic_type}" == "std_msgs/msg/String" && \
                "${endpoint_node}" == "phanthy_bus_bridge" ]]; then
          ((shadow_bus_observers += 1))
        else
          shadow_unexpected_endpoints+="subscriber:${endpoint_topic_type}:${endpoint_namespace}/${endpoint_node},"
        fi
      fi
    done <<<"${shadow_rows}"

    if [[ "${shadow_twist_publishers}" == "1" && \
          "${shadow_internal_twist_subscribers}" -le 1 && \
          -z "${shadow_foreign_twist_subscribers}" && \
          -z "${shadow_unexpected_endpoints}" && \
          "${shadow_bus_observers}" -le 1 ]]; then
      shadow_endpoints_ready=1
      break
    fi
    echo "G1_NAV2_SHADOW_ENDPOINTS=waiting,attempt:${shadow_attempt},twist_publishers:${shadow_twist_publishers},internal_twist:${shadow_internal_twist_subscribers},bus_observers:${shadow_bus_observers}"
    sleep 2
  done

  printf "%s\n" "${shadow_info}"
  if [[ "${shadow_endpoints_ready}" != "1" ]]; then
    echo "ERROR=unsafe raw shadow endpoints: twist_publishers=${shadow_twist_publishers} internal_twist=${shadow_internal_twist_subscribers} bus_observers=${shadow_bus_observers} foreign_twist=${shadow_foreign_twist_subscribers:-none} unexpected=${shadow_unexpected_endpoints:-none}" >&2
    exit 1
  fi
  echo "G1_NAV2_SHADOW_TWIST_PUBLISHERS=${shadow_twist_publishers}"
  echo "G1_NAV2_SHADOW_BUS_OBSERVERS=${shadow_bus_observers}"

  if grep -Fxq "/ubuntu/navigation/nav2/velocity_proposal" <<<"${topics}"; then
    if [[ "${shadow_internal_twist_subscribers}" != "1" ]]; then
      echo "ERROR=raw shadow velocity must have exactly one typed internal proposal-wrapper subscriber" >&2
      exit 1
    fi
    echo "G1_NAV2_SHADOW_INTERNAL_TWIST_SUBSCRIBERS=${shadow_internal_twist_subscribers}"

    proposal_info="$(ros2 topic info --verbose /ubuntu/navigation/nav2/velocity_proposal)"
    printf "%s\n" "${proposal_info}"
    proposal_rows="$(endpoint_rows "${proposal_info}")"
    proposal_publishers=0
    proposal_bus_observers=0
    proposal_driver_subscribers=0
    proposal_unexpected_endpoints=""
    while IFS="|" read -r endpoint_type endpoint_topic_type \
        endpoint_namespace endpoint_node; do
      [[ -n "${endpoint_type}" ]] || continue
      if [[ "${endpoint_type}" == "PUBLISHER" && \
            "${endpoint_topic_type}" == "std_msgs/msg/String" && \
            "${endpoint_node}" == "g1_nav2_navigation_command" ]]; then
        ((proposal_publishers += 1))
      elif [[ "${endpoint_type}" == "SUBSCRIPTION" && \
              "${endpoint_topic_type}" == "std_msgs/msg/String" && \
              "${endpoint_node}" == "phanthy_bus_bridge" ]]; then
        ((proposal_bus_observers += 1))
      elif [[ "${endpoint_type}" == "SUBSCRIPTION" && \
              "${endpoint_topic_type}" == "std_msgs/msg/String" && \
              -n "${PROPOSAL_DRIVER_NODE:-}" && \
              "${endpoint_node}" == "${PROPOSAL_DRIVER_NODE}" ]]; then
        ((proposal_driver_subscribers += 1))
      else
        proposal_unexpected_endpoints+="${endpoint_type}:${endpoint_topic_type}:${endpoint_namespace}/${endpoint_node},"
      fi
    done <<<"${proposal_rows}"
    expected_driver_subscribers=0
    if [[ -n "${PROPOSAL_DRIVER_NODE:-}" && \
          "${PROPOSAL_DRIVER_STANDBY:-0}" != "1" ]]; then
      expected_driver_subscribers=1
    fi
    if [[ "${proposal_publishers}" != "1" || \
          "${proposal_bus_observers}" -gt 1 || \
          "${proposal_driver_subscribers}" != "${expected_driver_subscribers}" || \
          -n "${proposal_unexpected_endpoints}" ]]; then
      echo "ERROR=proposal endpoints are not ready: driver_node=${PROPOSAL_DRIVER_NODE:-none} driver_subscribers=${proposal_driver_subscribers} expected_driver_subscribers=${expected_driver_subscribers} unexpected=${proposal_unexpected_endpoints:-none}" >&2
      exit 1
    fi
    echo "G1_NAV2_TOPIC=/ubuntu/navigation/nav2/velocity_proposal"
    echo "G1_NAV2_PROPOSAL_BUS_OBSERVERS=${proposal_bus_observers}"
    echo "G1_NAV2_PROPOSAL_DRIVER_SUBSCRIBERS=${proposal_driver_subscribers}"
    if [[ "${PROPOSAL_DRIVER_STANDBY:-0}" == "1" ]]; then
      echo "G1_NAV2_PROPOSAL_DRIVER_STATE=standby"
    fi
    echo "G1_NAV2_N5_PROPOSAL_ISOLATED=PASS"
  else
    if [[ "${REQUIRE_N5_PROTOCOL}" == "1" ]]; then
      echo "ERROR=N5 velocity proposal topic is required but absent" >&2
      exit 1
    fi
    if [[ "${shadow_internal_twist_subscribers}" != "0" ]]; then
      echo "ERROR=legacy shadow output has typed Twist subscribers" >&2
      exit 1
    fi
    echo "G1_NAV2_SHADOW_INTERNAL_TWIST_SUBSCRIBERS=0"
    echo "G1_NAV2_N5_PROPOSAL=absent"
  fi

  if ! navigation_status="$(timeout 12 ros2 topic echo --once \
    --qos-reliability reliable --qos-durability transient_local --field data \
    /ubuntu/navigation/nav2/status 2>&1)"; then
    printf "%s\n" "${navigation_status}"
    echo "ERROR=no navigation2 status heartbeat" >&2
    exit 1
  fi
  echo "G1_NAV2_CARD_STATUS_BEGIN"
  printf "%s\n" "${navigation_status}"
  echo "G1_NAV2_CARD_STATUS_END"
  if ! grep -Fq "\"shadow_only\":true" <<<"${navigation_status}" || \
     ! grep -Fq "\"physical_execution\":false" <<<"${navigation_status}"; then
    echo "ERROR=navigation2 status is not shadow-only" >&2
    exit 1
  fi

  if ! odom_status="$(timeout 12 ros2 topic echo --once \
    --qos-reliability reliable --field data \
    /ubuntu/navigation/nav2/odom_status 2>&1)"; then
    printf "%s\n" "${odom_status}"
    echo "ERROR=no odom status sample" >&2
    exit 1
  fi
  echo "G1_NAV2_ODOM_STATUS_BEGIN"
  printf "%s\n" "${odom_status}"
  echo "G1_NAV2_ODOM_STATUS_END"
  if ! grep -Eq "\"state\"[[:space:]]*:[[:space:]]*\"ready\"" \
    <<<"${odom_status}"; then
    echo "ERROR=native odom is not ready; restore the G1 Driver input first" >&2
    exit 1
  fi

  if ! scan_header="$(timeout 12 ros2 topic echo --once \
    --qos-reliability best_effort --field header \
    /ubuntu/navigation/nav2/scan 2>&1)"; then
    if [[ "${LEGACY_DRIVER_INPUT_UPGRADE_SOURCE_AUDIT}" == "1" ]]; then
      echo "G1_NAV2_SCAN_STATUS=missing_legacy_input_allowed_for_card5_upgrade"
    else
      printf "%s\n" "${scan_header}"
      echo "ERROR=no lidar scan sample; restore the navigation sensor bridge first" >&2
      exit 1
    fi
  else
    echo "G1_NAV2_SCAN_HEADER_BEGIN"
    printf "%s\n" "${scan_header}"
    echo "G1_NAV2_SCAN_HEADER_END"
  fi

  if [[ "${LEGACY_DRIVER_INPUT_UPGRADE_SOURCE_AUDIT}" == "1" ]]; then
    echo "G1_NAV2_LEGACY_RUNTIME_STATUS=degraded_source_allowed_for_card5_upgrade"
    exit 0
  fi

  query_lifecycle_once() {
    local node="$1"
    local query_timeout="$2"
    set +e
    lifecycle_state="$(timeout "${query_timeout}" \
      ros2 lifecycle get "/${node}" 2>&1)"
    lifecycle_rc=$?
    set -e

    # ROS 2 CLI can print the service response and then overrun timeout while
    # tearing down DDS. An explicit active response is authoritative even when
    # timeout reports 124; an absent/non-active response remains a failure.
    [[ "${lifecycle_state}" == *"active [3]"* ]]
  }

  lifecycle_ready=0
  lifecycle_state="not queried"
  lifecycle_rc=0
  lifecycle_deadline=$((SECONDS + 60))
  while ((SECONDS < lifecycle_deadline)); do
    remaining=$((lifecycle_deadline - SECONDS))
    query_timeout=3
    if ((remaining < query_timeout)); then
      query_timeout=${remaining}
    fi
    if ((query_timeout < 1)); then
      break
    fi
    if query_lifecycle_once velocity_smoother "${query_timeout}"; then
      lifecycle_ready=1
      break
    fi
    sleep 1
  done
  if [[ "${lifecycle_ready}" != "1" ]]; then
    printf "G1_NAV2_NODE=velocity_smoother %s\n" "${lifecycle_state}"
    echo "ERROR=Nav2 lifecycle did not become active within 60 seconds" >&2
    exit 1
  fi

  for node in controller_server velocity_smoother planner_server bt_navigator; do
    lifecycle_ready=0
    lifecycle_state="not queried"
    lifecycle_rc=0
    for attempt in 1 2 3; do
      if query_lifecycle_once "${node}" 5; then
        lifecycle_ready=1
        break
      fi
      sleep 1
    done
    printf "G1_NAV2_NODE=%s %s\n" "${node}" "${lifecycle_state}"
    if [[ "${lifecycle_ready}" != "1" ]]; then
      echo "ERROR=/${node} lifecycle did not report active after 3 bounded queries (last_rc=${lifecycle_rc})" >&2
      exit 1
    fi
    if [[ "${lifecycle_rc}" != "0" ]]; then
      echo "G1_NAV2_NODE_QUERY_NOTE=/${node} active response accepted despite cli_rc=${lifecycle_rc}"
    fi
  done
  echo "G1_NAV2_LIFECYCLE_MANAGER=active"

  echo "G1_NAV2_MAP_METADATA_BEGIN"
  timeout 20 ros2 topic echo --once \
    --qos-reliability reliable --qos-durability transient_local \
    --field info /map nav_msgs/msg/OccupancyGrid
  echo "G1_NAV2_MAP_METADATA_END"
'
printf -v quoted_runtime_audit '%q' "${runtime_audit}"
if ! ssh "${ssh_opts[@]}" "${g1_host}" \
  "docker exec --env REQUIRE_N5_PROTOCOL=${require_n5_protocol} --env PROPOSAL_DRIVER_NODE=${proposal_driver_node} --env PROPOSAL_DRIVER_STANDBY=${proposal_driver_standby} --env LEGACY_DRIVER_INPUT_UPGRADE_SOURCE_AUDIT=${legacy_driver_input_upgrade_source_audit} ${container_name} /bin/bash -lc ${quoted_runtime_audit}"; then
  echo "ERROR=Nav2 shadow runtime audit failed" >&2
  exit 1
fi

echo "G1_NAV2_SHADOW_AUDIT=PASS"
if [[ "${proposal_driver_standby}" == "1" ]]; then
  echo "NOTE=read-only audit; the registered loco Driver is in trusted standby with no proposal subscription and no robot command was issued"
elif [[ -n "${proposal_driver_node}" ]]; then
  echo "NOTE=read-only audit; the expected loco proposal subscriber is present and no robot command was issued"
else
  echo "NOTE=read-only audit; no Driver executor is connected"
fi
