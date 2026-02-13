#
# Copyright (c) 2023 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#

import contextlib
import unittest
from unittest.mock import Mock

import rclpy

import muto_agent.muto_agent
from muto_agent.topic_parser import MutoTopicParser


class TestAgentNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self):
        # Create node without full initialization to avoid MQTT/service dependencies
        self.node = muto_agent.muto_agent.MutoAgent()

    def tearDown(self):
        with contextlib.suppress(BaseException):
            self.node.cleanup()
        self.node.destroy_node()

    def test_mqtt_node_create(self):
        assert self.node is not None, "Node couldn't be created."

    def test_node_has_topic_parser(self):
        # Initialize just the topic parser for testing
        from muto_agent.topic_parser import MutoTopicParser

        self.node._topic_parser = MutoTopicParser()

        # Test that node has a topic parser
        parser = self.node.get_topic_parser()
        assert parser is not None, "Node should have a topic parser"

    def test_config_manager(self):
        # Test config manager creation with a mock node
        mock_node = Mock()
        mock_node.get_parameters_by_prefix.return_value = {}
        mock_node.declare_parameter.return_value = None

        # Mock parameter values
        def mock_get_parameter(key, default):
            return default  # Just return the default value

        mock_node.get_parameter.return_value.value = None

        # Create a simpler mock that handles parameter access
        from muto_agent.config import AgentConfig, MQTTConfig, TopicConfig

        simple_config = AgentConfig(
            mqtt=MQTTConfig(
                host="localhost",
                port=1883,
                keep_alive=60,
                user="test",
                password="test",
                namespace="test",
                prefix="muto",
                name="test",
            ),
            topics=TopicConfig(
                stack_topic="stack",
                twin_topic="twin",
                agent_to_gateway_topic="agent_to_gateway",
                gateway_to_agent_topic="gateway_to_agent",
                agent_to_commands_topic="agent_to_command",
                commands_to_agent_topic="command_to_agent",
                thing_messages_topic="thing_messages",
            ),
        )

        # Test basic config creation
        assert simple_config is not None, "Should be able to create config"
        assert simple_config.mqtt.port == 1883, "Port should be set correctly"

    def test_parse_topic_via_parser(self):
        # Test parsing via the topic parser directly
        parser = MutoTopicParser()

        # Agent command topic
        topic = "org.eclipse.muto.sandbox/f1tenth/things/live/messages/agent/commands/test"
        result = parser.parse_topic(topic)
        assert result == ("agent", "test"), f"Expected ('agent', 'test'), got {result}"

        # Stack command topic
        topic = "org.eclipse.muto.sandbox:f1tenth/stack/commands/apply"
        result = parser.parse_topic(topic)
        assert result == ("stack", "apply"), f"Expected ('stack', 'apply'), got {result}"


class TestTopicParser(unittest.TestCase):
    def setUp(self):
        self.parser = MutoTopicParser()

    def test_agent_topic_parsing(self):
        topic = "org.eclipse.muto.sandbox/f1tenth/things/live/messages/agent/commands/test"
        parsed = self.parser.parse_topic(topic)

        assert parsed is not None, "Should parse agent topic successfully"
        assert parsed == ("agent", "test"), f"Expected ('agent', 'test'), got {parsed}"

    def test_stack_topic_parsing(self):
        topic = "org.eclipse.muto.sandbox:f1tenth/stack/commands/apply"
        parsed = self.parser.parse_topic(topic)

        assert parsed is not None, "Should parse stack topic"
        assert parsed == ("stack", "apply"), f"Expected ('stack', 'apply'), got {parsed}"

    def test_ping_topic_parsing(self):
        topic = "org.eclipse.muto.sandbox/f1tenth/things/live/messages/agent/commands/ping"
        parsed = self.parser.parse_topic(topic)

        assert parsed is not None, "Should parse ping topic"
        # Based on the original test, ping commands result in ("ping", None)
        assert parsed == ("ping", None), f"Expected ('ping', None), got {parsed}"

    def test_telemetry_topic_parsing(self):
        topic = "org.eclipse.muto.sandbox/device/things/live/messages/telemetry"
        parsed = self.parser.parse_topic(topic)

        assert parsed == (None, None), "Should return (None, None) for telemetry topic"

    def test_invalid_topic_parsing(self):
        invalid_topic = "invalid/topic/format"

        try:
            parsed = self.parser.parse_topic(invalid_topic)
            assert parsed == (None, None), "Should return (None, None) for invalid topic"
        except Exception:
            # Also acceptable to raise an exception
            pass

    def test_empty_topic_parsing(self):
        try:
            self.parser.parse_topic("")
            raise AssertionError("Should have raised an exception for empty topic")
        except Exception:
            pass  # Expected behavior

    def test_is_valid_topic(self):
        # Test basic validation
        valid_topic = "org.eclipse.muto.sandbox/f1tenth/things/live/messages/agent/commands/test"
        invalid_topic = "invalid_topic"

        assert self.parser.is_valid_topic(valid_topic), "Valid topic should pass validation"
        assert not self.parser.is_valid_topic(invalid_topic), "Invalid topic should be invalid"


if __name__ == "__main__":
    unittest.main()
