#include <array>
#include <cstdint>
#include <cstddef>
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

// A probe accepts exactly one normalized MAC, so malformed serial input cannot
// turn into a broadcast match against an unintended device.
void normalized_mac_selects_only_the_requested_device() {
    uint8_t requestedMac[6] = {0};
    expect(parseDeviceReportMac("5443b2b542f4", requestedMac), "normalized MAC was rejected");
    expect(!parseDeviceReportMac("54:43:b2:b5:42:f4", requestedMac), "separator-filled MAC was accepted");
    expect(!parseDeviceReportMac("5443b2b542fg", requestedMac), "non-hexadecimal MAC was accepted");

    DeviceReportMessage probe;
    memcpy(probe.mac, requestedMac, sizeof(probe.mac));
    expect(deviceReportTargetsMac(probe, requestedMac), "probe did not select its exact MAC");

    uint8_t otherMac[6] = {0x54, 0x43, 0xb2, 0xb5, 0x42, 0xf5};
    expect(!deviceReportTargetsMac(probe, otherMac), "probe selected a neighboring MAC");
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

// Lock the additive report extension to its deployed byte layout. This keeps
// mixed-version relays and host tooling compatible when fields are appended.
void report_layout_and_taxonomy_remain_stable() {
    expect(sizeof(DeviceReportMessage) == 38, "device report wire size changed");
    expect(offsetof(DeviceReportMessage, actionKey) == 0, "action key offset changed");
    expect(offsetof(DeviceReportMessage, kind) == 1, "action argument offset changed");
    expect(offsetof(DeviceReportMessage, magic) == 2, "report magic offset changed");
    expect(offsetof(DeviceReportMessage, hardwareFamily) == 5, "hardware family offset changed");
    expect(offsetof(DeviceReportMessage, tubesVersion) == 6, "Tubes version offset changed");
    expect(offsetof(DeviceReportMessage, releaseHash) == 30, "release hash offset changed");
    expect(offsetof(DeviceReportMessage, uptimeSeconds) == 34, "uptime offset changed");
    expect(DEVICE_REPORT_PROTOCOL_VERSION == 1, "report protocol version changed");
    expect(TubeHardwareDig2Go == 1, "Dig2Go family ID changed");
    expect(TubeHardwareWaveshareS3 == 6, "Waveshare S3 family ID changed");
}

} // namespace

int main() {
    const std::array<std::pair<const char*, void (*)()>, 3> tests = {{
        {"normalized MAC selects only requested device", normalized_mac_selects_only_the_requested_device},
        {"report preserves deployed action prefix", report_preserves_the_deployed_action_prefix},
        {"report layout and taxonomy remain stable", report_layout_and_taxonomy_remain_stable},
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
