import copy
import json
import math
from pathlib import Path
import unittest

from g1_nav2.execution_protocol import (
    ExecutorGate,
    GateError,
    GateState,
    ProtocolError,
    RuntimeHealth,
    Velocity,
    VelocityProposal,
    build_velocity_proposal,
)


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance_ms(self, milliseconds):
        self.now += milliseconds / 1000.0


def healthy(**overrides):
    values = {
        "main_control_ready": True,
        "estop_clear": True,
        "odom_age_ms": 20.0,
        "scan_age_ms": 30.0,
        "nav2_status_age_ms": 40.0,
    }
    values.update(overrides)
    return RuntimeHealth(**values)


def proposal(
    *,
    nav_id="nav-1",
    sequence=1,
    ttl_ms=250,
    status="navigating",
    velocity=Velocity(0.1, 0.0, 0.1),
):
    return build_velocity_proposal(
        nav_id=nav_id,
        sequence=sequence,
        ttl_ms=ttl_ms,
        navigation_status=status,
        velocity=velocity,
        issued_at_unix_ms=123456789,
    )


class VelocityProposalTest(unittest.TestCase):
    def test_checked_in_schema_and_example_match_the_runtime_contract(self):
        protocol_dir = Path(__file__).resolve().parents[1] / "protocol"
        schema = json.loads(
            (protocol_dir / "velocity-proposal-v1.schema.json").read_text()
        )
        example = json.loads(
            (protocol_dir / "velocity-proposal-v1.example.json").read_text()
        )

        parsed = VelocityProposal.from_payload(example)
        properties = schema["properties"]
        self.assertEqual(
            properties["schema"]["const"],
            "phanthy.navigation.velocity_proposal.v1",
        )
        self.assertEqual(properties["frame"]["const"], "base_link")
        self.assertIn("planning", properties["nav_status"]["enum"])
        self.assertEqual(properties["ttl_ms"]["maximum"], 250)
        self.assertEqual(properties["velocity"]["properties"]["x"]["maximum"], 0.15)
        self.assertEqual(properties["velocity"]["properties"]["x"]["minimum"], -0.05)
        self.assertEqual(properties["velocity"]["properties"]["y"]["maximum"], 0.12)
        self.assertEqual(properties["velocity"]["properties"]["yaw"]["maximum"], 0.35)
        self.assertEqual(set(schema["required"]), set(example))
        self.assertEqual(parsed.nav_id, "nav-example-1")

    def test_round_trip_preserves_identity_ttl_and_shadow_flags(self):
        payload = proposal()
        parsed = VelocityProposal.from_payload(payload)

        self.assertEqual(parsed.nav_id, "nav-1")
        self.assertEqual(parsed.sequence, 1)
        self.assertEqual(parsed.ttl_ms, 250)
        self.assertEqual(parsed.velocity, Velocity(0.1, 0.0, 0.1))
        self.assertTrue(payload["shadow_only"])
        self.assertFalse(payload["physical_execution"])
        self.assertEqual(payload["frame"], "base_link")
        self.assertEqual(payload["nav_status"], "navigating")

    def test_rejects_schema_flags_ttl_non_finite_and_speed_violations(self):
        base = proposal()
        cases = []
        for path, value, code in (
            (("schema",), "wrong", "schema_mismatch"),
            (("frame",), "odom", "frame_mismatch"),
            (("shadow_only",), False, "unsafe_flag"),
            (("physical_execution",), True, "unsafe_flag"),
            (("ttl_ms",), 251, "ttl_limit"),
            (("velocity", "x"), float("nan"), "invalid_number"),
            (("velocity", "x"), 0.16, "velocity_limit"),
            (("velocity", "y"), 0.13, "velocity_limit"),
            (("velocity", "yaw"), 0.36, "velocity_limit"),
        ):
            payload = copy.deepcopy(base)
            target = payload
            for key in path[:-1]:
                target = target[key]
            target[path[-1]] = value
            cases.append((payload, code))

        planar = copy.deepcopy(base)
        planar["velocity"] = {"x": 0.15, "y": 0.12, "yaw": 0.0}
        cases.append((planar, "velocity_limit"))

        for payload, code in cases:
            with self.subTest(code=code, payload=payload):
                with self.assertRaises(ProtocolError) as caught:
                    VelocityProposal.from_payload(payload)
                self.assertEqual(caught.exception.code, code)

    def test_non_motion_state_must_carry_zero(self):
        payload = proposal()
        payload["nav_status"] = "paused"

        with self.assertRaises(ProtocolError) as caught:
            VelocityProposal.from_payload(payload)

        self.assertEqual(caught.exception.code, "unsafe_navigation_state")

    def test_rejects_driver_unsupported_status_alias(self):
        payload = proposal()
        payload["navigation_status"] = payload.pop("nav_status")

        with self.assertRaises(ProtocolError) as caught:
            VelocityProposal.from_payload(payload)

        self.assertEqual(
            caught.exception.code, "unsupported_navigation_status_field"
        )


class ExecutorGateTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.gate = ExecutorGate(clock=self.clock)

    def arm(self, *, lease_duration_ms=3000):
        return self.gate.arm(
            owner_id="owner-console",
            lease_id="lease-1",
            nav_id="nav-1",
            lease_duration_ms=lease_duration_ms,
            health=healthy(),
        )

    def test_restart_is_unarmed_and_cannot_execute_a_proposal(self):
        decision = self.gate.ingest(proposal(), health=healthy())

        self.assertEqual(self.gate.state, GateState.UNARMED)
        self.assertFalse(decision.accepted)
        self.assertTrue(decision.must_send_zero)
        self.assertEqual(decision.velocity, Velocity.zero())

    def test_arm_requires_live_health_and_bounded_lease(self):
        with self.assertRaises(GateError) as unhealthy:
            self.gate.arm(
                owner_id="owner-console",
                lease_id="lease-1",
                nav_id="nav-1",
                lease_duration_ms=3000,
                health=healthy(scan_age_ms=501),
            )
        self.assertEqual(unhealthy.exception.code, "runtime_not_ready")

        with self.assertRaises(GateError) as lease:
            self.gate.arm(
                owner_id="owner-console",
                lease_id="lease-1",
                nav_id="nav-1",
                lease_duration_ms=5001,
                health=healthy(),
            )
        self.assertEqual(lease.exception.code, "invalid_lease")
        self.assertEqual(self.gate.state, GateState.UNARMED)

    def test_valid_sequence_executes_and_zero_returns_to_armed_idle(self):
        armed = self.arm()
        moving = self.gate.ingest(proposal(), health=healthy())
        idle = self.gate.ingest(
            proposal(sequence=2, velocity=Velocity.zero()), health=healthy()
        )

        self.assertEqual(armed.state, GateState.ARMED_IDLE)
        self.assertEqual(moving.state, GateState.EXECUTING)
        self.assertEqual(moving.velocity, Velocity(0.1, 0.0, 0.1))
        self.assertFalse(moving.must_send_zero)
        self.assertEqual(idle.state, GateState.ARMED_IDLE)
        self.assertTrue(idle.must_send_zero)

    def test_wrong_nav_or_replayed_sequence_stops_and_requires_ack(self):
        for bad_payload, reason in (
            (proposal(nav_id="nav-2"), "nav_id_mismatch"),
            (proposal(sequence=1), "sequence_replay"),
        ):
            with self.subTest(reason=reason):
                self.gate = ExecutorGate(clock=self.clock)
                self.arm()
                if reason == "sequence_replay":
                    self.gate.ingest(proposal(sequence=1), health=healthy())
                decision = self.gate.ingest(bad_payload, health=healthy())
                self.assertEqual(decision.state, GateState.STOPPING)
                self.assertEqual(decision.reason, reason)
                self.assertTrue(decision.must_send_zero)
                stopped = self.gate.acknowledge_stopped()
                self.assertEqual(stopped.state, GateState.UNARMED)

    def test_receive_time_ttl_and_lease_heartbeat_are_fail_closed(self):
        self.arm()
        self.gate.ingest(proposal(ttl_ms=200), health=healthy())
        self.clock.advance_ms(201)
        expired = self.gate.poll(health=healthy())

        self.assertEqual(expired.reason, "proposal_ttl_expired")
        self.assertEqual(expired.state, GateState.STOPPING)

        self.gate.acknowledge_stopped()
        self.arm(lease_duration_ms=3000)
        self.clock.advance_ms(1001)
        heartbeat = self.gate.poll(health=healthy())
        self.assertEqual(heartbeat.reason, "heartbeat_timeout")

        self.gate.acknowledge_stopped()
        self.arm(lease_duration_ms=500)
        self.clock.advance_ms(500)
        lease = self.gate.poll(health=healthy())
        self.assertEqual(lease.reason, "lease_expired")

    def test_renewal_extends_control_heartbeat_but_not_a_stale_proposal(self):
        self.arm()
        self.gate.ingest(proposal(ttl_ms=250), health=healthy())
        self.clock.advance_ms(700)
        renewed = self.gate.renew(
            owner_id="owner-console",
            lease_id="lease-1",
            lease_duration_ms=3000,
            health=healthy(),
        )
        expired = self.gate.poll(health=healthy())

        self.assertTrue(renewed.accepted)
        self.assertEqual(expired.reason, "proposal_ttl_expired")

    def test_runtime_health_loss_forces_zero(self):
        self.arm()
        self.gate.ingest(proposal(), health=healthy())
        decision = self.gate.poll(
            health=healthy(main_control_ready=False)
        )

        self.assertEqual(decision.state, GateState.STOPPING)
        self.assertEqual(decision.reason, "health:main_control_not_ready")
        self.assertEqual(decision.velocity, Velocity.zero())

    def test_terminal_navigation_disarms_after_physical_stop_ack(self):
        self.arm()
        terminal = self.gate.ingest(
            proposal(
                sequence=1,
                status="arrived",
                velocity=Velocity.zero(),
            ),
            health=healthy(),
        )
        acknowledged = self.gate.acknowledge_stopped()

        self.assertEqual(terminal.state, GateState.STOPPING)
        self.assertEqual(terminal.reason, "navigation_arrived")
        self.assertEqual(acknowledged.state, GateState.UNARMED)
        self.assertIsNone(self.gate.snapshot()["lease_id"])

    def test_estop_is_latched_until_explicit_reset(self):
        self.arm()
        self.gate.ingest(proposal(), health=healthy())
        stopping = self.gate.estop(reason="remote_estop")
        latched = self.gate.acknowledge_stopped()

        self.assertEqual(stopping.state, GateState.STOPPING)
        self.assertEqual(latched.state, GateState.ESTOPPED)
        with self.assertRaises(GateError):
            self.arm()

        reset = self.gate.reset_estop(
            owner_id="owner-console", health=healthy()
        )
        self.assertEqual(reset.state, GateState.UNARMED)

    def test_hardware_estop_health_signal_is_also_latched(self):
        self.arm()
        self.gate.ingest(proposal(), health=healthy())
        stopping = self.gate.poll(health=healthy(estop_clear=False))
        latched = self.gate.acknowledge_stopped()

        self.assertEqual(stopping.reason, "health:estop_not_clear")
        self.assertEqual(latched.state, GateState.ESTOPPED)

    def test_non_finite_runtime_age_is_not_ready(self):
        self.arm()
        decision = self.gate.poll(health=healthy(odom_age_ms=math.inf))

        self.assertEqual(decision.reason, "health:odom_stale")
        self.assertEqual(decision.state, GateState.STOPPING)


if __name__ == "__main__":
    unittest.main()
