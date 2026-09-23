"""Run GUI logic tests without requiring a live serial adapter."""

import sys
import types
import unittest
from pathlib import Path


try:
    import serial  # noqa: F401
except ImportError:
    serial = types.ModuleType("serial")
    serial.SerialException = OSError
    serial.tools = types.ModuleType("serial.tools")
    serial.tools.list_ports = types.ModuleType("serial.tools.list_ports")
    serial.tools.list_ports.comports = lambda: []
    sys.modules.update(
        {
            "serial": serial,
            "serial.tools": serial.tools,
            "serial.tools.list_ports": serial.tools.list_ports,
        }
    )


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.discover(str(Path(__file__).parent), "test_*.py")
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(0 if result.wasSuccessful() else 1)
