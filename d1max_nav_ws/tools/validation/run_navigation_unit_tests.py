#!/usr/bin/env python3
"""Run passive navigation tests with ROS graph creation forbidden.

Not the bag/closed-loop runner: integration replay needs a separately reviewed
loopback-only Zenoh configuration and domain. No such launch is hidden here.
"""
import sys
from unittest.mock import patch
import pytest
import rclpy
from rclpy.node import Node


def forbidden(*args, **kwargs):
    raise AssertionError('ROS graph creation forbidden in file-only navigation regression')


if __name__ == '__main__':
    with patch.object(rclpy,'init',forbidden), patch.object(rclpy,'create_node',forbidden), \
            patch.object(Node,'__init__',forbidden):
        raise SystemExit(pytest.main(sys.argv[1:]))
