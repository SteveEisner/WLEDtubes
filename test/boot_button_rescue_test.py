#!/usr/bin/env python3
"""Source contracts for boot rescue admission and button diagnostics."""
from pathlib import Path

source = (Path(__file__).parents[1] / "wled00" / "wled.cpp").read_text()
assert "checkRescuePin" not in source
assert "checkRescueSerial" in source
assert "WLED_DISABLE_STUCK_BUTTON_DIAGNOSTICS" in source
assert "diagnoseBootButtons();" in source
assert "buttons.size()" in source
assert "STUCK_BUTTON" in source
assert "WLED button diagnostics: healthy" in source
assert "WLED button diagnostics: disabled" in source
print("boot rescue admission and stuck-button diagnostics: PASS")
