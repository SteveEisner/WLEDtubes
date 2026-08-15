#!/usr/bin/env python3
"""Source contracts for boot rescue admission and button diagnostics."""
from pathlib import Path

source = (Path(__file__).parents[1] / "wled00" / "wled.cpp").read_text()
assert "checkRescuePin" not in source
assert "checkRescueSerial" in source
assert "WLED_DISABLE_STUCK_BUTTON_DIAGNOSTICS" in source
assert source.count("diagnoseBootButtons();") >= 2
assert "stuckButtonInterval = 10000UL" in source
assert "activeSince[b] = 0" in source
assert "activeInitialized[b] = false" in source
assert "if (!activeInitialized[b])" in source
assert "diagnosticPin[b] != pin" in source
assert "diagnosticType[b] != type" in source
assert "BTN_TYPE_PUSH && button.type != BTN_TYPE_PUSH_ACT_HIGH" in source
assert "active at boot" not in source
assert "sustained active" in source
assert "WLED button diagnostics: STUCK_BUTTON" in source
assert "WLED button diagnostics: AVAILABLE/INACTIVE" in source
assert "HEALTHY/AVAILABLE" not in source
assert "WLED button diagnostics: disabled" in source
assert 'DEBUG_PRINTF_P(PSTR("heap %u\\n")' in source
assert 'PSTR("heap %u\\\\n")' not in source
print("boot rescue admission and sustained stuck-button diagnostics: PASS")
