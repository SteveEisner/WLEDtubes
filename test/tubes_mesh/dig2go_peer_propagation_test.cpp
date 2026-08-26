#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#define EXPECT(condition) do { \
  if (!(condition)) { \
    std::cerr << "EXPECT failed at line " << __LINE__ << ": " #condition "\n"; \
    std::exit(1); \
  } \
} while (false)

static std::string readSource(const char* path) {
  std::ifstream source(path);
  EXPECT(source.good());
  std::stringstream buffer;
  buffer << source.rdbuf();
  return buffer.str();
}

static void propagationSelectionIsSeparateFromOtaSelection() {
  const std::string controller = readSource("usermods/Tubes/controller.h");
  EXPECT(controller.find("TUBE_COMMAND('Q', PropagationSelectOperation, MeshScope)")
      != std::string::npos);
  const auto begin = controller.find("bool startSelectedPropagation()");
  const auto end = controller.find("bool isSelected() const", begin);
  EXPECT(begin != std::string::npos && end != std::string::npos);
  const std::string trigger = controller.substr(begin, end - begin);
  EXPECT(trigger.find("makeModernPropagationServeCommand") != std::string::npos);
  EXPECT(trigger.find("updater.ready") == std::string::npos);
  EXPECT(trigger.find("select()") == std::string::npos);
}

static void barePowerSaveCommandRemainsIntact() {
  const std::string controller = readSource("usermods/Tubes/controller.h");
  EXPECT(controller.find("key == 'P' && strchr(command + 1, ',')")
      != std::string::npos);
  EXPECT(controller.find("else if (key == 'P')") != std::string::npos);
}

static void laptopFleetToolCannotStartPropagation() {
  const std::string tool = readSource("usermods/Tubes/fleet_pull_update.py");
  EXPECT(tool.find("--propagate") == std::string::npos);
  EXPECT(tool.find("args.propagate") == std::string::npos);
  EXPECT(tool.find("f\"Y{release},{advertise}") != std::string::npos);
}

static void productionBuildHasNoBenchBootTriggers() {
  const std::string config = readSource("platformio_tubes.ini");
  const auto begin = config.find("[env:esp32_quinled_dig2go_tubes_p2p]");
  const auto end = config.find("\n[env:", begin + 1);
  EXPECT(begin != std::string::npos && end != std::string::npos);
  const std::string environment = config.substr(begin, end - begin);
  EXPECT(environment.find("TUBES_ENABLE_DIG2GO_PEER_PROPAGATION=1") != std::string::npos);
  EXPECT(environment.find("TUBES_DIG2GO_LEGACY_PULL_HOST=1") != std::string::npos);
  EXPECT(environment.find("TUBES_DIG2GO_DYNAMIC_ENROLLMENT=1") != std::string::npos);
  EXPECT(environment.find("AUTO_TRIGGER") == std::string::npos);
  EXPECT(environment.find("PRIME_MAC") == std::string::npos);
  EXPECT(environment.find("BOOT_FALLBACK_TEST") == std::string::npos);
}

static void oneTurnAdvertisesToLegacyAndCurrentPeers() {
  const std::string tubes = readSource("usermods/Tubes/Tubes.h");
  const auto begin = tubes.find("case LegacyPullRendezvousSendWake");
  const auto end = tubes.find("case LegacyPullRendezvousStationArrived", begin);
  EXPECT(begin != std::string::npos && end != std::string::npos);
  const std::string wake = tubes.substr(begin, end - begin);
  EXPECT(wake.find("sendFleetPullUpdateOffer") != std::string::npos);
  EXPECT(wake.find("sendLegacyPullUpdateOffer") != std::string::npos);
}

static void propagationRetiresAfterTransferWithoutRebootAck() {
  const std::string tubes = readSource("usermods/Tubes/Tubes.h");
  EXPECT(tubes.find("legacyPullBodyServed && !legacyHostRetired")
      != std::string::npos);
  EXPECT(tubes.find("transfer_complete_no_ack") != std::string::npos);
  EXPECT(tubes.find("requestDig2GoHealthReport") == std::string::npos);
}

static void modernIdentityIsAuthorizedBeforeReceiverAdmission() {
  const std::string host = readSource("usermods/Tubes/legacy_pull_host.h");
  const auto observe = host.find("void observe()");
  const auto observeEnd = host.find("bool bodyComplete()", observe);
  const auto authorize = host.find("if (modern && !authorizeModernRequest");
  const auto admission = host.find("const int slot = admitRequestStation", authorize);
  const auto admit = host.find("int admitRequestStation(");
  const auto admitEnd = host.find("static bool parseUnsignedParam", admit);
  EXPECT(observe != std::string::npos && observeEnd != std::string::npos);
  const std::string associationOnly = host.substr(observe, observeEnd - observe);
  EXPECT(associationOnly.find("LegacyPullTelemetry::admit(") == std::string::npos);
  EXPECT(associationOnly.find("setEnrolledMac") == std::string::npos);
  EXPECT(associationOnly.find("stationSeen() = true") == std::string::npos);
  EXPECT(authorize != std::string::npos && admission != std::string::npos);
  EXPECT(authorize < admission);
  EXPECT(admit != std::string::npos && admitEnd != std::string::npos);
  const std::string eligibleReceiver = host.substr(admit, admitEnd - admit);
  const auto seenAt = eligibleReceiver.find("stationSeenAt() = millis()");
  const auto admitted = eligibleReceiver.find("LegacyPullTelemetry::admit");
  EXPECT(seenAt != std::string::npos && admitted != std::string::npos);
  EXPECT(seenAt < admitted);
  EXPECT(host.find("[serve](AsyncWebServerRequest* request) { serve(request, false); }")
      != std::string::npos);
  EXPECT(host.find("[serve](AsyncWebServerRequest* request) { serve(request, true); }")
      != std::string::npos);
}

static void failedPullKeepsRestoringUntilMeshIsStarted() {
  const std::string controller = readSource("usermods/Tubes/controller.h");
  const auto begin = controller.find(
      "if (fleetPropagationTransportSuspended && updater.status == Failed)");
  const auto end = controller.find("// WLED state changes", begin);
  EXPECT(begin != std::string::npos && end != std::string::npos);
  const std::string recovery = controller.substr(begin, end - begin);
  EXPECT(recovery.find("fleetPropagationRestoreStarted = restoreMeshRadioAfterDig2Go()")
      != std::string::npos);
  EXPECT(recovery.find("else if (meshRadioStartedAfterDig2Go())")
      != std::string::npos);
  const auto meshStarted = recovery.find("else if (meshRadioStartedAfterDig2Go())");
  const auto clearSuspended = recovery.find("fleetPropagationTransportSuspended = false");
  EXPECT(meshStarted < clearSuspended);
}

int main() {
  propagationSelectionIsSeparateFromOtaSelection();
  barePowerSaveCommandRemainsIntact();
  laptopFleetToolCannotStartPropagation();
  productionBuildHasNoBenchBootTriggers();
  oneTurnAdvertisesToLegacyAndCurrentPeers();
  propagationRetiresAfterTransferWithoutRebootAck();
  modernIdentityIsAuthorizedBeforeReceiverAdmission();
  failedPullKeepsRestoringUntilMeshIsStarted();
  std::cout << "dig2go_peer_propagation_test: ok\n";
  return 0;
}
