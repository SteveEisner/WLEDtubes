#pragma once

#include <stdint.h>
#include <string.h>

constexpr char DEVICE_REPORT_ACTION_KEY = 'z';
constexpr uint16_t DEVICE_REPORT_MAGIC = 0x5452;
constexpr uint8_t DEVICE_REPORT_PROTOCOL_VERSION = 1;

enum DeviceReportMeshFlag : uint8_t {
    DeviceReportMeshStarted = 1 << 0,
    DeviceReportMeshFollowing = 1 << 1,
    DeviceReportMasterBehavior = 1 << 2,
};

enum DeviceReportKind : uint8_t {
    DeviceReportProbe = 1,
    DeviceReportReply = 2,
};

enum TubeHardwareFamily : uint8_t {
    TubeHardwareUnknown = 0,
    TubeHardwareDig2Go = 1,
    TubeHardwareMatrixM1 = 2,
    TubeHardwareAthomC3 = 3,
    TubeHardwareGledopto = 4,
    TubeHardwareHomeLight = 5,
    TubeHardwareWaveshareS3 = 6,
};

enum TubeFirmwareVariant : uint8_t {
    TubeVariantStandard = 0,
    TubeVariantChristmas = 1,
    TubeVariantGolden = 2,
};

enum TubeCompatibilityClass : uint8_t {
    TubeCompatibilityUnknown = 0,
    TubeCompatibilityLegacy = 1,
    TubeCompatibilityCurrent = 2,
    TubeCompatibilityNext = 3,
};

// A bounded, read-only copy of canonical local Tubes report state for board adapters.
// Compatibility remains Unknown until a version-policy adapter proves otherwise.
struct TubesReadOnlySnapshot {
    uint8_t hardwareFamily = TubeHardwareUnknown;
    uint8_t reportProtocolVersion = DEVICE_REPORT_PROTOCOL_VERSION;
    uint8_t compatibilityClass = TubeCompatibilityUnknown;
    uint16_t tubesRelease = 0;
    char wledVersion[16] = {0};
    uint8_t controllerRole = 0;
    uint8_t meshFlags = 0;
    uint16_t nodeId = 0;
    uint16_t uplinkId = 0;
};

// Board adapters consume a copy rather than reaching into the Tubes controller.
bool tubesCopyReadOnlySnapshot(TubesReadOnlySnapshot& snapshot);

#ifndef TUBES_HARDWARE_FAMILY
#define TUBES_HARDWARE_FAMILY TubeHardwareUnknown
#endif

#ifndef TUBES_FIRMWARE_VARIANT
#define TUBES_FIRMWARE_VARIANT TubeVariantStandard
#endif

#pragma pack(push, 1)
struct DeviceReportMessage {
    // The first two bytes preserve the deployed `Action` layout. Older firmware
    // treats `z` as an unknown action, then safely relays the untouched payload.
    char actionKey = DEVICE_REPORT_ACTION_KEY;
    uint8_t kind = DeviceReportProbe;

    uint16_t magic = DEVICE_REPORT_MAGIC;
    uint8_t protocolVersion = DEVICE_REPORT_PROTOCOL_VERSION;
    uint8_t hardwareFamily = TubeHardwareUnknown;
    uint16_t tubesVersion = 0;
    uint32_t nonce = 0;
    uint8_t mac[6] = {0};
    uint16_t ledCount = 0;
    uint8_t busCount = 0;
    uint8_t ledPin = 0xFF;
    uint8_t ledType = 0;
    uint8_t firmwareVariant = TubeVariantStandard;
    uint8_t controllerRole = 0;
    uint8_t meshFlags = 0;
    uint16_t nodeId = 0;
    uint16_t uplinkId = 0;
    uint32_t releaseHash = 0;
    uint32_t uptimeSeconds = 0;
};
#pragma pack(pop)

static_assert(sizeof(DeviceReportMessage) == 38, "Device report wire size changed");
static_assert(sizeof(DeviceReportMessage) <= 64, "Device report exceeds the mesh payload");

inline bool isDeviceReportMessage(const DeviceReportMessage& message) {
    return message.actionKey == DEVICE_REPORT_ACTION_KEY
        && message.magic == DEVICE_REPORT_MAGIC
        && message.protocolVersion == DEVICE_REPORT_PROTOCOL_VERSION
        && (message.kind == DeviceReportProbe || message.kind == DeviceReportReply);
}

inline bool deviceReportTargetsMac(
    const DeviceReportMessage& message,
    const uint8_t deviceMac[6]
) {
    return message.kind == DeviceReportProbe
        && memcmp(message.mac, deviceMac, sizeof(message.mac)) == 0;
}

inline int deviceReportHexNibble(char value) {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
}

inline bool parseDeviceReportMac(const char* text, uint8_t mac[6]) {
    if (!text || strlen(text) != 12)
        return false;

    for (uint8_t index = 0; index < 6; index++) {
        int high = deviceReportHexNibble(text[index * 2]);
        int low = deviceReportHexNibble(text[index * 2 + 1]);
        if (high < 0 || low < 0)
            return false;
        mac[index] = uint8_t((high << 4) | low);
    }
    return true;
}
