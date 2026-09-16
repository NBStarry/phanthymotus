#!/usr/bin/env python3
"""Run one public Perception card against simulation fixtures through Core.

Run inside the ROS-enabled Perception container. This proves ROS output, not
browser rendering. Explicit MCP IDs prevent accidentally testing another bundle.
"""
import argparse
import json
import ssl
import time
from collections import deque
from urllib.request import Request, urlopen

import rclpy
from audio_msgs.msg import AudioChunk
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("card", choices=["asr", "vop", "face_recognition", "tts", "ocr"])
    parser.add_argument("--driver-id", required=True)
    parser.add_argument("--perception-id", required=True)
    parser.add_argument("--expected-object", default="bus",
                        help="VOP target in the selected input scene (bus fixture or chair room)")
    parser.add_argument("--expected-ocr-text", default="SIM ROOM",
                        help="Required text from the rendered room sign")
    parser.add_argument("--producer-first", action="store_true",
                        help="Start the simulation sensor before its consumer, as canvas startup does")
    parser.add_argument("--tts-input-topic", default="/acceptance/sim",
                        help="Match the TTS card's canvas input topic when checking its waveform")
    parser.add_argument("--preview-delay", type=int, choices=range(0, 31), default=0,
                        help="Seconds to open the browser audio preview before TTS speak (0-30)")
    args = parser.parse_args()
    if args.card == "ocr" and not args.expected_ocr_text.strip():
        parser.error("--expected-ocr-text must not be empty")
    if args.producer_first and args.card == "tts":
        parser.error("--producer-first requires a sensor-backed card")
    # Loopback-only, self-signed development Core. Never send credentials here.
    core = "https://127.0.0.1:15678"
    context = ssl._create_unverified_context()

    def call(mcp_id, tool, action, **arguments):
        body = json.dumps({"tool": tool, "arguments": {"action": action, **arguments}}).encode()
        req = Request(f"{core}/api/mcp/{mcp_id}/call", body,
                      {"Content-Type": "application/json"})
        with urlopen(req, context=context, timeout=45) as response:
            result = json.load(response)
        if result.get("code") != 200:
            raise RuntimeError(f"{tool}.{action}: {result}")
        data = json.loads(result["data"][0]["text"])
        if data.get("state") == "error" or data.get("error") or data.get("error_code"):
            raise RuntimeError(f"{tool}.{action}: {data}")
        print(f"{tool}.{action}: {data.get('state', data.get('status', 'ok'))}", flush=True)
        return data

    sensor = "mic" if args.card == "asr" else "camera_rgb"
    topic = "/phanthymotus_sim_g1/" + ("mic/audio" if args.card == "asr" else "camera/rgb")
    if args.card == "tts":
        topic = args.tts_input_topic
    suffix = {"asr": "/asr", "vop": "/objects", "face_recognition": "/face", "tts": "/tts", "ocr": "/ocr"}[args.card]
    instance = "sim_accept_" + args.card
    rclpy.init()
    node = rclpy.create_node("sim_public_card_acceptance")
    records = deque(maxlen=100)
    pcm = bytearray()
    audio_ended = False

    def receive(msg):
        nonlocal audio_ended
        if args.card == "tts":
            data = bytes(msg.data)
            if data == b"\x01\x00\xff\xff\x01\x00\xff\xff":
                audio_ended = True
            else:
                pcm.extend(data)
        else:
            records.append(json.loads(msg.data))

    sub = node.create_subscription(AudioChunk if args.card == "tts" else String,
                                   topic + suffix, receive, qos_profile_sensor_data)
    cleanup_errors = []
    cleanup = []
    try:
        if args.card != "tts":
            state = call(args.driver_id, sensor, "info")
            if not state.get("simulation") or state.get("state") != "idle":
                raise RuntimeError("Require an idle simulation sensor; do not stop another user's stream")
        existing = call(args.perception_id, args.card, "info", instance_id=instance)
        if existing.get("state") not in ("idle", "ready"):
            raise RuntimeError(f"Require an idle card before acceptance: {existing.get('state')}")
        if args.producer_first:
            cleanup.append((args.driver_id, sensor, {}))
            call(args.driver_id, sensor, "start")
        cleanup.append((args.perception_id, args.card, {"instance_id": instance, "input_topic": topic}))
        state = call(args.perception_id, args.card, "start", input_topic=topic, instance_id=instance)
        if state.get("state") not in ("running", "loading"):
            raise RuntimeError(f"start did not start the card: {state}")
        if args.card != "tts" and not args.producer_first:
            cleanup.append((args.driver_id, sensor, {}))
            call(args.driver_id, sensor, "start")
        # VOP start lacks topic_out; info supplies the original producer contract.
        call(args.perception_id, args.card, "info", input_topic=topic, instance_id=instance)
        # Upstream 125db64's call-time registration swallows an asyncio scope
        # error. Its existing capability-refresh endpoint registers live topics.
        req = Request(f"{core}/api/mcp/{args.perception_id}/ping", data=b"", method="POST")
        with urlopen(req, context=context, timeout=30) as response:
            refresh = json.load(response)
        if refresh.get("code") != 200:
            raise RuntimeError("Core capability refresh failed")
        if args.card == "tts":
            print("TTS_PREVIEW_READY", topic + suffix, flush=True)
            ready_at = time.monotonic() + args.preview_delay
            while time.monotonic() < ready_at:
                rclpy.spin_once(node, timeout_sec=.2)
            call(args.perception_id, "tts", "speak", text="你好，这是仿真语音输出测试。", instance_id=instance)
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.5)
            if (args.card == "asr" and any("机器人" in r.get("text", "") for r in records)
                    or args.card == "vop" and any(any(o.get("name") == args.expected_object for o in r.get("objects", [])) for r in records)
                    or args.card == "face_recognition" and any(r.get("count", 0) > 0 and r.get("faces") for r in records)
                    or args.card == "ocr" and any(not r.get("error") and r.get("items") and args.expected_ocr_text.replace(" ", "") in r.get("text", "").replace(" ", "") for r in records)
                    or args.card == "tts" and audio_ended and len(pcm) >= 32000 and any(pcm)):
                break
        else:
            raise RuntimeError(f"No expected {args.card} output within 40 seconds")
        print("PUBLIC_CARD_ROS_PASS", args.card,
              json.dumps(records[-1] if records else {"audio_bytes": len(pcm), "eof": audio_ended}, ensure_ascii=False), flush=True)
    finally:
        for mcp_id, tool, extra in reversed(cleanup):
            try:
                call(mcp_id, tool, "stop", **extra)
            except Exception as error:
                cleanup_errors.append(str(error))
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()
        if cleanup_errors:
            raise RuntimeError(f"Cleanup failed: {cleanup_errors}")


if __name__ == "__main__":
    main()
