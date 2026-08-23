#include <array>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "device_report_protocol.h"

namespace {

void expect(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}

// A probe accepts one normalized MAC or the explicit all-zero wildcard, so
// malformed serial input cannot become an unintended fleet request.
void normalized_mac_or_wildcard_selects_expected_devices() {
    uint8_t requestedMac[6] = {0};
    expect(parseDeviceReportMac("5443b2b542f4", requestedMac), "normalized MAC was rejected");
    expect(!parseDeviceReportMac("54:43:b2:b5:42:f4", requestedMac), "separator-filled MAC was accepted");
    expect(!parseDeviceReportMac("5443b2b542fg", requestedMac), "non-hexadecimal MAC was accepted");

    DeviceReportMessage probe;
    memcpy(probe.mac, requestedMac, sizeof(probe.mac));
    expect(deviceReportTargetsMac(probe, requestedMac), "probe did not select its exact MAC");

    uint8_t otherMac[6] = {0x54, 0x43, 0xb2, 0xb5, 0x42, 0xf5};
    expect(!deviceReportTargetsMac(probe, otherMac), "probe selected a neighboring MAC");

    memset(probe.mac, 0, sizeof(probe.mac));
    expect(deviceReportMacIsWildcard(probe.mac), "zero MAC was not recognized as the wildcard");
    expect(deviceReportTargetsMac(probe, requestedMac), "wildcard omitted the requested device");
    expect(deviceReportTargetsMac(probe, otherMac), "wildcard omitted a neighboring device");
}

// Targeted update selection uses the complete printed Device ID and rejects
// zero or partial values before they can enter the mesh.
void update_selection_targets_one_device_id() {
    DeviceId requestedId = 0;
    expect(parseDeviceReportId("1A2B", requestedId), "hexadecimal Device ID was rejected");
    expect(requestedId == 0x1A2B, "hexadecimal Device ID changed during parsing");
    expect(!parseDeviceReportId("1A2", requestedId), "partial Device ID was accepted");
    expect(!parseDeviceReportId("0000", requestedId), "zero Device ID was accepted");
    expect(!parseDeviceReportId("1X2B", requestedId), "non-hexadecimal Device ID was accepted");

    DeviceReportMessage request;
    request.kind = DeviceUpdateSelect;
    request.nodeId = 0x1A2B;
    expect(deviceReportTargetsNode(request, 0x1A2B), "target device ignored its update selection");
    expect(!deviceReportTargetsNode(request, 0x1A2C), "neighbor accepted another device's selection");
}

// The two-byte Action prefix is the compatibility contract that lets deployed
// firmware ignore the extension while relaying all report fields unchanged.
void report_preserves_the_deployed_action_prefix() {
    DeviceReportMessage report;
    report.kind = DeviceReportReply;
    report.nonce = 0x89ABCDEF;

    const auto* wireBytes = reinterpret_cast<const uint8_t*>(&report);
    expect(wireBytes[0] == static_cast<uint8_t>(DEVICE_REPORT_ACTION_KEY), "action key moved off byte zero");
    expect(wireBytes[1] == DeviceReportReply, "report kind moved off the action argument byte");
    expect(isDeviceReportMessage(report), "valid report failed protocol validation");

    report.magic = 0;
    expect(!isDeviceReportMessage(report), "invalid protocol magic was accepted");
}

} // namespace

int main() {
    const std::array<std::pair<const char*, void (*)()>, 3> tests = {{
        {"normalized MAC or wildcard selects expected devices", normalized_mac_or_wildcard_selects_expected_devices},
        {"update selection targets one Device ID", update_selection_targets_one_device_id},
        {"report preserves deployed action prefix", report_preserves_the_deployed_action_prefix},
    }};

    for (const auto& test : tests) {
        try {
            test.second();
            std::cout << "PASS: " << test.first << '\n';
        } catch (const std::exception& error) {
            std::cerr << "FAIL: " << test.first << ": " << error.what() << '\n';
            return 1;
        }
    }
    return 0;
}
