from __future__ import annotations

import pytest

from roarm_mcp.arm import Arm
from roarm_mcp.sim import SimTransport


@pytest.fixture
def arm() -> Arm:
    return Arm(SimTransport(realtime=False))
