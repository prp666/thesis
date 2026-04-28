#!/usr/bin/env bash
set -euo pipefail

RESULT_DIR="${RESULT_DIR:-/home/prp/thesis/scenario_f_runtime_eval_results}"
WAIT_FOR_SYSTEM_SEC="${WAIT_FOR_SYSTEM_SEC:-15}"
HZ_DURATION_SEC="${HZ_DURATION_SEC:-10}"
RESOURCE_DURATION_SEC="${RESOURCE_DURATION_SEC:-60}"
RESOURCE_INTERVAL_SEC="${RESOURCE_INTERVAL_SEC:-1}"

EXPECTED_NODES=(
  "/fake_audio_source_node"
  "/sound_presence_detector_node"
  "/fake_image_source_node"
  "/vision_detector_node"
  "/decision_node"
)

TOPICS_TO_MEASURE=(
  "/sound/audio"
  "/sound/detected"
  "/sound/confidence"
  "/camera/image_raw"
  "/vision/detected"
  "/vision/confidence"
  "/decision/warning"
  "/decision/score"
  "/decision/state"
  "/decision/reason"
)

PROCESS_PATTERN='thesis_ws/install/.*/lib/.*/(fake_audio_source_node|sound_presence_detector_node|fake_image_source_node|vision_detector|decision_node)$'

mkdir -p "${RESULT_DIR}"

echo "[INFO] Scenario F observer started."
echo "[INFO] This script only observes an already running system."
echo "[INFO] Output directory: ${RESULT_DIR}"

set +u
source /opt/ros/humble/setup.bash
source /home/prp/thesis/thesis_ws/install/setup.bash
set -u

timestamp() {
  date +"%Y-%m-%d %H:%M:%S"
}

safe_name() {
  local name="$1"
  name="${name#/}"
  name="${name//\//_}"
  echo "${name}"
}

extract_average_rate() {
  local file="$1"
  awk '/average rate:/ {rate=$3} END {if (rate == "") print "NA"; else print rate}' "${file}"
}

collect_node_union() {
  local output_file="$1"
  local end_time=$((SECONDS + WAIT_FOR_SYSTEM_SEC))
  : > "${output_file}.tmp"
  while [[ "${SECONDS}" -lt "${end_time}" ]]; do
    ros2 node list >> "${output_file}.tmp" 2>/dev/null || true
    sleep 1
  done
  {
    echo "# collected_at: $(timestamp)"
    echo "# node discovery union over ${WAIT_FOR_SYSTEM_SEC}s"
    grep '^/' "${output_file}.tmp" | sort -u || true
  } > "${output_file}"
  rm -f "${output_file}.tmp"
}

echo "[INFO] F1: collecting node and topic availability..."
collect_node_union "${RESULT_DIR}/f1_node_list.txt"

{
  echo "# collected_at: $(timestamp)"
  ros2 topic list || true
} > "${RESULT_DIR}/f1_topic_list.txt"

{
  echo "# collected_at: $(timestamp)"
  ros2 topic list -t || true
} > "${RESULT_DIR}/f1_topic_list_with_types.txt"

{
  echo "node,available"
  node_list="$(grep '^/' "${RESULT_DIR}/f1_node_list.txt" || true)"
  for node in "${EXPECTED_NODES[@]}"; do
    if grep -Fxq "${node}" <<< "${node_list}"; then
      echo "${node},true"
    else
      echo "${node},false"
    fi
  done
} > "${RESULT_DIR}/f1_expected_node_availability.csv"

for node in "${EXPECTED_NODES[@]}"; do
  node_file="$(safe_name "${node}")"
  {
    echo "# collected_at: $(timestamp)"
    ros2 node info "${node}" || true
  } > "${RESULT_DIR}/f1_node_info_${node_file}.txt" 2>&1
done

for topic in "${TOPICS_TO_MEASURE[@]}"; do
  topic_file="$(safe_name "${topic}")"
  {
    echo "# collected_at: $(timestamp)"
    ros2 topic info "${topic}" -v || true
  } > "${RESULT_DIR}/f1_topic_info_${topic_file}.txt" 2>&1
done

echo "[INFO] F2: measuring topic publication frequencies..."
echo "topic,expected_hz_note,observed_average_hz,raw_file" > "${RESULT_DIR}/f2_topic_hz_summary.csv"

for topic in "${TOPICS_TO_MEASURE[@]}"; do
  topic_file="$(safe_name "${topic}")"
  raw_file="${RESULT_DIR}/f2_hz_${topic_file}.txt"

  echo "[INFO] Measuring ${topic} for ${HZ_DURATION_SEC}s..."
  {
    echo "# collected_at: $(timestamp)"
    echo "# command: timeout -s INT ${HZ_DURATION_SEC}s ros2 topic hz --window 10 --wall-time ${topic}"
    timeout -s INT "${HZ_DURATION_SEC}s" ros2 topic hz --window 10 --wall-time "${topic}" || true
  } > "${raw_file}" 2>&1

  observed="$(extract_average_rate "${raw_file}")"
  expected_note="project_specific"
  case "${topic}" in
    "/sound/audio"|"/sound/detected"|"/sound/confidence")
      expected_note="about_15.6_hz"
      ;;
    "/camera/image_raw")
      expected_note="about_5_hz"
      ;;
    "/vision/detected"|"/vision/confidence")
      expected_note="about_1.6_hz_if_process_every_3_frames"
      ;;
    "/decision/warning"|"/decision/score"|"/decision/state"|"/decision/reason")
      expected_note="about_5_hz"
      ;;
  esac

  echo "${topic},${expected_note},${observed},${raw_file}" >> "${RESULT_DIR}/f2_topic_hz_summary.csv"
done

echo "[INFO] F3: collecting runtime resource usage..."
{
  echo "# collected_at: $(timestamp)"
  ps -eo pid,ppid,stat,comm,%cpu,%mem,rss,args | grep -E "${PROCESS_PATTERN}" | grep -v grep || true
} > "${RESULT_DIR}/f3_process_snapshot.txt"

mapfile -t PIDS < <(pgrep -f "${PROCESS_PATTERN}" || true)
PID_CSV="$(IFS=,; echo "${PIDS[*]:-}")"

{
  echo "# collected_at: $(timestamp)"
  echo "# process_pattern: ${PROCESS_PATTERN}"
  echo "# pids: ${PID_CSV:-none}"
} > "${RESULT_DIR}/f3_resource_metadata.txt"

echo -e "timestamp\tpid\tcomm\tcpu_percent\tmem_percent\trss_kb\targs" > "${RESULT_DIR}/f3_resource_samples.tsv"

if [[ "${#PIDS[@]}" -eq 0 ]]; then
  echo "[WARN] No ROS project node processes matched pattern: ${PROCESS_PATTERN}"
else
  echo "[INFO] Sampling resources for PIDs: ${PID_CSV}"
  end_time=$((SECONDS + RESOURCE_DURATION_SEC))
  while [[ "${SECONDS}" -lt "${end_time}" ]]; do
    now="$(timestamp)"
    for pid in "${PIDS[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        ps -p "${pid}" -o pid=,comm=,%cpu=,%mem=,rss=,args= |
          awk -v ts="${now}" '{
            pid=$1; comm=$2; cpu=$3; mem=$4; rss=$5;
            sub("^[^ ]+[ ]+[^ ]+[ ]+[^ ]+[ ]+[^ ]+[ ]+[^ ]+[ ]+", "", $0);
            print ts "\t" pid "\t" comm "\t" cpu "\t" mem "\t" rss "\t" $0
          }' >> "${RESULT_DIR}/f3_resource_samples.tsv"
      fi
    done
    sleep "${RESOURCE_INTERVAL_SEC}"
  done
fi

if command -v pidstat >/dev/null 2>&1 && [[ "${#PIDS[@]}" -gt 0 ]]; then
  echo "[INFO] pidstat found; collecting pidstat summary..."
  pidstat -u -r -p "${PID_CSV}" 1 10 > "${RESULT_DIR}/f3_pidstat_10s.txt" || true
else
  echo "[INFO] pidstat not available or no PIDs found; skipped pidstat collection."
fi

{
  echo "# Scenario F collection finished at: $(timestamp)"
  echo "# Results directory: ${RESULT_DIR}"
  echo ""
  echo "F1 files:"
  echo "  f1_node_list.txt"
  echo "  f1_topic_list.txt"
  echo "  f1_topic_list_with_types.txt"
  echo "  f1_expected_node_availability.csv"
  echo ""
  echo "F2 files:"
  echo "  f2_topic_hz_summary.csv"
  echo "  f2_hz_*.txt"
  echo ""
  echo "F3 files:"
  echo "  f3_process_snapshot.txt"
  echo "  f3_resource_metadata.txt"
  echo "  f3_resource_samples.tsv"
  echo "  f3_pidstat_10s.txt, if pidstat is available"
} > "${RESULT_DIR}/scenario_f_README.txt"

echo "[INFO] Scenario F observer finished."
echo "[INFO] Results saved to: ${RESULT_DIR}"
