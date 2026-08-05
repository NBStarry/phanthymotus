import os
import unittest
from unittest import mock

import mcp_probe


class McpProbeStartupTest(unittest.TestCase):
    def test_initialize_retries_connection_refused_within_bound(self):
        expected = {"serverInfo": {"name": "perception-bundle"}}
        with mock.patch.dict(os.environ, {"MCP_STARTUP_TIMEOUT": "1"}), mock.patch(
            "mcp_probe.rpc", side_effect=[OSError("connection refused"), expected]
        ) as rpc, mock.patch("mcp_probe.time.sleep"):
            result = mcp_probe.initialize()

        self.assertEqual(result, expected)
        self.assertEqual(rpc.call_count, 2)

    def test_initialize_rejects_unbounded_timeout(self):
        with mock.patch.dict(os.environ, {"MCP_STARTUP_TIMEOUT": "61"}):
            with self.assertRaisesRegex(ValueError, "within"):
                mcp_probe.initialize()


if __name__ == "__main__":
    unittest.main()
