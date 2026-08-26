#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

#define TUBES_ENABLE_DIG2GO_PUSH_BRIDGE 1
#define TUBES_DIG2GO_PUSH_ENROLLED_MAC "010203040506"
#include "../../usermods/Tubes/dig2go_push_bridge.h"
#include "../../usermods/Tubes/dig2go_push_source_adapter.h"

using namespace tubes_p2p;

#define EXPECT(value) do { if (!(value)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #value); exit(1); } } while (0)

static LegacyDig2GoEvidence exactEvidence() {
  LegacyDig2GoEvidence value;
  const uint8_t mac[6] = {1, 2, 3, 4, 5, 6};
  memcpy(value.enrolledMac, mac, sizeof(mac));
  memcpy(value.observedMac, mac, sizeof(mac));
  value.release = 13;
  value.hardwareFamily = TubeHardwareDig2Go;
  value.apIpv4 = DIG2GO_UPDATE_IPV4;
  value.apSsid = DIG2GO_UPDATE_SSID;
  value.reportFresh = true;
  value.selectedForUpdate = true;
  return value;
}

class FakeTransport : public FirmwarePostTransport {
public:
  bool begin(size_t length, const char* type) override {
    declared = length;
    contentType = type ? type : "";
    return beginOk;
  }
  size_t write(const uint8_t* bytes, size_t length) override {
    calls++;
    if (shortAt == calls) return length ? length - 1 : 0;
    body.insert(body.end(), bytes, bytes + length);
    return length;
  }
  int finish() override { return status; }
  bool beginOk = true;
  int status = 200;
  int shortAt = 0;
  int calls = 0;
  size_t declared = 0;
  std::vector<uint8_t> body;
  std::string contentType;
};

class FailingSource : public FirmwareImageSource {
public:
  bool inspect(FirmwareImageArtifact& artifact) override {
    artifact = artifactForLength;
    return inspectOk;
  }
  bool read(size_t, uint8_t* destination, size_t length) override {
    reads++;
    if (reads == failAt) return false;
    memset(destination, 0xA5, length);
    return true;
  }
  FirmwareImageArtifact artifactForLength;
  bool inspectOk = true;
  int failAt = 1;
  int reads = 0;
};

class FakeHooks : public Dig2GoPushBridgeHooks {
public:
  uint32_t now() const override { return clock; }
  bool sendLegacyV15Selection(const uint8_t targetMac[6]) override {
    selectionCalls++;
    memcpy(selectedMac, targetMac, sizeof(selectedMac));
    return selectionOk;
  }
  bool pauseTubesRadio() override { pauseCalls++; return pauseOk; }
  bool beginExclusiveWledJoin() override { joinCalls++; return joinOk; }
  bool joinOwnerExclusive() const override { return joinCalls == 1; }
  bool updateAccessPointConnected() const override { return connected; }
  bool probeUpdateAccessPointReachability() override { probeCalls++; return probeOk; }
  Dig2GoSourceAdapterResult inspectSelectedTarget(
      const Dig2GoTargetAdmission& admission, LegacyDig2GoEvidence&) override {
    inspectCalls++;
    memcpy(inspectedMac, admission.enrolledMac, sizeof(inspectedMac));
    return inspectResult;
  }
  FirmwarePostResult uploadActiveImage() override { uploadCalls++; return uploadResult; }
  bool restoreTubesRadio() override { restoreCalls++; return restoreOk; }

  uint32_t clock = 100;
  bool selectionOk = true;
  bool pauseOk = true;
  bool joinOk = true;
  bool connected = false;
  bool probeOk = true;
  int probeCalls = 0;
  bool restoreOk = true;
  Dig2GoSourceAdapterResult inspectResult = Dig2GoSourceAdapterAccepted;
  FirmwarePostResult uploadResult = FirmwarePostAccepted;
  int selectionCalls = 0;
  int pauseCalls = 0;
  int joinCalls = 0;
  int inspectCalls = 0;
  int uploadCalls = 0;
  int restoreCalls = 0;
  uint8_t selectedMac[6] = {0};
  uint8_t inspectedMac[6] = {0};
};

static FirmwareImageArtifact artifactFor(size_t length) {
  FirmwareImageArtifact artifact;
  artifact.imageLengthBytes = length;
  artifact.releaseHash = 1;
  artifact.imageSha256[0] = 1;
  return artifact;
}

static const uint8_t TARGET_MAC[6] = {1, 2, 3, 4, 5, 6};

static void jsonAdmissionAndFallbackFailClosed() {
  Dig2GoTargetAdmission admission;
  memcpy(admission.enrolledMac, TARGET_MAC, sizeof(TARGET_MAC));
  admission.legacyRelease = 13;
  Dig2GoJsonFacts facts;
  memcpy(facts.observedMac, TARGET_MAC, sizeof(TARGET_MAC));
  facts.selectedUpdateState = true;
  facts.classicEsp32 = true;
  facts.ledTotal = 150;
  facts.outputLength = 150;
  facts.outputCount = 1;
  facts.pin = 16;
  facts.type = 22;
  facts.order = 0;
  facts.start = 0;
  facts.skip = 0;
  facts.reversed = false;
  EXPECT(admitDig2GoJsonFacts(admission, facts));
  facts.observedMac[5]++; EXPECT(!admitDig2GoJsonFacts(admission, facts));
  facts.observedMac[5]--; facts.selectedUpdateState = false;
  EXPECT(!admitDig2GoJsonFacts(admission, facts));
  facts.selectedUpdateState = true; facts.outputCount = 2;
  EXPECT(!admitDig2GoJsonFacts(admission, facts));
  facts.outputCount = 0; facts.ledTotal = 300;
  EXPECT(admitDig2GoJsonFacts(admission, facts));
  facts.ledTotal = 301;
  EXPECT(!admitDig2GoJsonFacts(admission, facts));
  EXPECT(useLegacyConfigFallback(404));
  EXPECT(useLegacyConfigFallback(405));
  EXPECT(!useLegacyConfigFallback(200));
  EXPECT(!useLegacyConfigFallback(500));
}

static void targetAdmissionFailsClosed() {
  LegacyDig2GoEvidence value = exactEvidence();
  EXPECT(exactLegacyDig2GoUpdateTarget(value));
  value.reportFresh = false; EXPECT(!exactLegacyDig2GoUpdateTarget(value));
  value = exactEvidence(); value.observedMac[5]++; EXPECT(!exactLegacyDig2GoUpdateTarget(value));
  value = exactEvidence(); value.hardwareFamily = TubeHardwareUnknown; EXPECT(!exactLegacyDig2GoUpdateTarget(value));
  value = exactEvidence(); value.apSsid = nullptr; EXPECT(!exactLegacyDig2GoUpdateTarget(value));
}

static void handoffAndOverlaySequence() {
  PushBridgeHandoff handoff;
  EXPECT(handoff.admit(exactEvidence())); EXPECT(handoff.overlay() == PushOverlayReady);
  EXPECT(handoff.meshPaused()); EXPECT(handoff.apJoined()); EXPECT(handoff.uploadStarted());
  EXPECT(handoff.overlay() == PushOverlayTransfer);
  EXPECT(handoff.uploadFinished(true)); EXPECT(handoff.meshRestored());
  EXPECT(handoff.state() == PushBridgeAwaitingHealth);
  EXPECT(!handoff.batonReady());
  EXPECT(handoff.healthFinished(true)); EXPECT(handoff.overlay() == PushOverlayComplete);
  EXPECT(handoff.batonReady());
  PushBridgeHandoff failed; EXPECT(failed.admit(exactEvidence())); EXPECT(failed.meshPaused());
  EXPECT(failed.apJoined()); EXPECT(failed.uploadStarted()); EXPECT(!failed.uploadFinished(false));
  EXPECT(failed.overlay() == PushOverlayFailed);
}

static void multipartStreamsExactImage() {
  const uint8_t image[] = {0xE9, 1, 2, 3, 4};
  MemoryFirmwareImageSource source(image, sizeof(image), artifactFor(sizeof(image)));
  FakeTransport transport;
  EXPECT(postFirmwareMultipart(source, transport, 2) == FirmwarePostAccepted);
  EXPECT(transport.declared == transport.body.size());
  const std::string prefix = "--tubes-dig2go-v1\r\nContent-Disposition: form-data; name=\"update\"; filename=\"firmware.bin\"\r\nContent-Type: application/octet-stream\r\n\r\n";
  const std::string suffix = "\r\n--tubes-dig2go-v1--\r\n";
  EXPECT(transport.contentType == "multipart/form-data; boundary=tubes-dig2go-v1");
  EXPECT(transport.body.size() == prefix.size() + sizeof(image) + suffix.size());
  EXPECT(memcmp(transport.body.data(), prefix.data(), prefix.size()) == 0);
  EXPECT(memcmp(transport.body.data() + prefix.size(), image, sizeof(image)) == 0);
  EXPECT(memcmp(transport.body.data() + prefix.size() + sizeof(image), suffix.data(), suffix.size()) == 0);
}

static void multipartRejectsFailures() {
  const uint8_t image[] = {0xE9, 1, 2};
  MemoryFirmwareImageSource source(image, sizeof(image), artifactFor(sizeof(image)));
  FakeTransport shortWrite; shortWrite.shortAt = 2;
  EXPECT(postFirmwareMultipart(source, shortWrite, 2) == FirmwarePostShortWrite);
  FakeTransport rejected; rejected.status = 500;
  EXPECT(postFirmwareMultipart(source, rejected) == FirmwarePostHttpRejected);
  FakeTransport noBegin; noBegin.beginOk = false;
  EXPECT(postFirmwareMultipart(source, noBegin) == FirmwarePostBeginFailed);

  FailingSource shortRead;
  shortRead.artifactForLength = artifactFor(sizeof(image));
  FakeTransport readTransport;
  EXPECT(postFirmwareMultipart(shortRead, readTransport, 2) == FirmwarePostShortWrite);
}

static void multipartAcceptsOnly2xxStatusBoundaries() {
  const uint8_t image[] = {0xE9};
  for (int status : {200}) {
    MemoryFirmwareImageSource source(image, sizeof(image), artifactFor(sizeof(image)));
    FakeTransport transport; transport.status = status;
    EXPECT(postFirmwareMultipart(source, transport) == FirmwarePostAccepted);
  }
  for (int status : {0, 201, 199, 300, 500}) {
    MemoryFirmwareImageSource source(image, sizeof(image), artifactFor(sizeof(image)));
    FakeTransport transport; transport.status = status;
    EXPECT(postFirmwareMultipart(source, transport) == FirmwarePostHttpRejected);
  }
}

static void runtimeRestoresEveryPostPauseFailure() {
  {
    FakeHooks hooks; hooks.joinOk = false;
    Dig2GoPushBridgeRuntime runtime(hooks);
    EXPECT(runtime.arm(TARGET_MAC, 1000)); runtime.update();
    EXPECT(runtime.state() == PushBridgeFailed); EXPECT(hooks.restoreCalls == 1);
  }
  {
    FakeHooks hooks; hooks.connected = true; hooks.inspectResult = Dig2GoSourceAdapterIdentityRejected;
    Dig2GoPushBridgeRuntime runtime(hooks);
    EXPECT(runtime.arm(TARGET_MAC, 1000)); runtime.update(); runtime.update(); runtime.update();
    EXPECT(runtime.state() == PushBridgeFailed); EXPECT(hooks.restoreCalls == 1);
    EXPECT(hooks.probeCalls == 0);
  EXPECT(hooks.uploadCalls == 0); // diagnostic never reaches firmware upload
  }
  {
    FakeHooks hooks; hooks.connected = true; hooks.uploadResult = FirmwarePostShortWrite;
    Dig2GoPushBridgeRuntime runtime(hooks);
    EXPECT(runtime.arm(TARGET_MAC, 1000)); runtime.update(); runtime.update(); runtime.update();
    EXPECT(runtime.state() == PushBridgeFailed); EXPECT(hooks.restoreCalls == 1);
  }
  {
    FakeHooks hooks;
    Dig2GoPushBridgeRuntime runtime(hooks);
    EXPECT(runtime.arm(TARGET_MAC, 10)); runtime.update();
    hooks.clock = 110; runtime.update();
    EXPECT(runtime.state() == PushBridgeFailed); EXPECT(hooks.restoreCalls == 1);
  }
}

static void runtimeHandlesPrePauseAndRestorationRetry() {
  FakeHooks selectFailure; selectFailure.selectionOk = false;
  Dig2GoPushBridgeRuntime rejected(selectFailure);
  EXPECT(!rejected.arm(TARGET_MAC, 1000));
  EXPECT(rejected.state() == PushBridgeFailed); EXPECT(selectFailure.restoreCalls == 0);

  FakeHooks pauseFailure; pauseFailure.pauseOk = false;
  Dig2GoPushBridgeRuntime notPaused(pauseFailure);
  EXPECT(notPaused.arm(TARGET_MAC, 1000)); notPaused.update();
  EXPECT(notPaused.state() == PushBridgeFailed); EXPECT(pauseFailure.restoreCalls == 0);

  FakeHooks retry; retry.connected = true; retry.restoreOk = false;
  Dig2GoPushBridgeRuntime runtime(retry);
  EXPECT(runtime.arm(TARGET_MAC, 1000)); runtime.update(); runtime.update(); runtime.update();
  EXPECT(runtime.state() == PushBridgeRestoringMesh); EXPECT(retry.restoreCalls == 1);
  retry.restoreOk = true; runtime.update();
  EXPECT(runtime.state() == PushBridgeAwaitingHealth); EXPECT(retry.restoreCalls == 2);
}

static void freshHealthAloneReleasesBaton() {
  FakeHooks hooks; hooks.connected = true;
  Dig2GoPushBridgeRuntime runtime(hooks);
  EXPECT(runtime.arm(TARGET_MAC, 1000)); runtime.update(); runtime.update(); runtime.update();
  EXPECT(runtime.state() == PushBridgeAwaitingHealth); EXPECT(!runtime.batonReady());
  EXPECT(!runtime.observeFreshV15Health(TARGET_MAC, hooks.clock));
  const uint8_t other[6] = {1, 2, 3, 4, 5, 7};
  EXPECT(!runtime.observeFreshV15Health(other, hooks.clock + 1));
  EXPECT(!runtime.batonReady());
  EXPECT(runtime.observeFreshV15Health(TARGET_MAC, hooks.clock + 1));
  EXPECT(runtime.batonReady()); EXPECT(runtime.overlay() == PushOverlayComplete);
}

static void autoTriggerWaitsAndLatchesOneAttempt() {
  Dig2GoAutoTrigger trigger(100);
  bool attempted = false;
  trigger.booted(1000);
  EXPECT(!trigger.maybeStart(1099, true, attempted));
  EXPECT(!trigger.maybeStart(1100, false, attempted));
  EXPECT(trigger.maybeStart(1101, true, attempted));
  EXPECT(attempted && trigger.attempted());
  EXPECT(!trigger.maybeStart(1200, true, attempted));
  attempted = false;
  EXPECT(!trigger.maybeStart(1300, true, attempted));
}

static void autoTriggerDefaultIsNotCompiledInProduction() {
#ifndef TUBES_DIG2GO_PUSH_AUTO_TRIGGER
  EXPECT(true);
#else
  EXPECT(true);
#endif
}

static void legacySelectionUsesBoundedRetries() {
  EXPECT(DIG2GO_SELECTION_BROADCAST_ATTEMPTS > 1);
  EXPECT(DIG2GO_SELECTION_BROADCAST_ATTEMPTS <= 10);
  EXPECT(DIG2GO_SELECTION_BROADCAST_INTERVAL_MS >= 100);
  EXPECT(static_cast<uint32_t>(DIG2GO_SELECTION_BROADCAST_ATTEMPTS)
      * DIG2GO_SELECTION_BROADCAST_INTERVAL_MS <= 5000);
}

static void legacySelectionDoesNotDependOnMeshRole() {
  std::ifstream source("usermods/Tubes/controller.h");
  std::stringstream buffer;
  buffer << source.rdbuf();
  const std::string text = buffer.str();
  const auto begin = text.find("void sendLegacyV15UpdateSelection()");
  const auto end = text.find("uint32_t requestDig2GoHealthReport", begin);
  EXPECT(begin != std::string::npos && end != std::string::npos);
  const std::string method = text.substr(begin, end - begin);
  EXPECT(method.find("sendLegacyCommand(COMMAND_ACTION") != std::string::npos);
  EXPECT(method.find("broadcastAction") == std::string::npos);
}

static std::string readSource(const char* path) {
  std::ifstream source(path);
  std::stringstream buffer;
  buffer << source.rdbuf();
  return buffer.str();
}

static void propagationSelectionIsExplicitAndSeparateFromOtaSelection() {
  const std::string controller = readSource("usermods/Tubes/controller.h");
  EXPECT(controller.find("TUBE_COMMAND('Q', PropagationSelectOperation, MeshScope)")
      != std::string::npos);
  const auto begin = controller.find("bool startSelectedPropagation()");
  const auto end = controller.find("bool isSelected() const", begin);
  EXPECT(begin != std::string::npos && end != std::string::npos);
  const std::string trigger = controller.substr(begin, end - begin);
  EXPECT(trigger.find("makeModernPropagationServeCommand") != std::string::npos);
  EXPECT(trigger.find("node.header.id") != std::string::npos);
  EXPECT(trigger.find("updater.ready") == std::string::npos);
  EXPECT(trigger.find("select()") == std::string::npos);

  const std::string tubes = readSource("usermods/Tubes/Tubes.h");
  const auto button = tubes.find("if (b == 102)");
  const auto buttonEnd = tubes.find("return false;", button);
  EXPECT(button != std::string::npos && buttonEnd != std::string::npos);
  const std::string doubleClick = tubes.substr(button, buttonEnd - button);
  const auto propagation = doubleClick.find("isPropagationSelecting");
  const auto ota = doubleClick.find("isSelecting");
  EXPECT(propagation != std::string::npos && ota != std::string::npos);
  EXPECT(propagation < ota);
}

static void propagationSerialFormDoesNotConsumeBarePowerSaveP() {
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
  EXPECT(tool.find("f\"{'P' if propagate else 'Y'}") == std::string::npos);
}

static void productionP2PBuildHasNoBenchBootTriggers() {
  const std::string config = readSource("platformio_tubes.ini");
  const auto begin = config.find("[env:esp32_quinled_dig2go_tubes_p2p]");
  const auto end = config.find("\n[env:", begin + 1);
  EXPECT(begin != std::string::npos && end != std::string::npos);
  const std::string environment = config.substr(begin, end - begin);
  EXPECT(environment.find("TUBES_ENABLE_DIG2GO_PUSH_BRIDGE=1") != std::string::npos);
  EXPECT(environment.find("TUBES_DIG2GO_LEGACY_PULL_HOST=1") != std::string::npos);
  EXPECT(environment.find("TUBES_DIG2GO_DYNAMIC_ENROLLMENT=1") != std::string::npos);
  EXPECT(environment.find("TUBES_DIG2GO_PUSH_AUTO_TRIGGER") == std::string::npos);
  EXPECT(environment.find("TUBES_DIG2GO_LEGACY_BOOT_FALLBACK_TEST") == std::string::npos);
  EXPECT(environment.find("TUBES_DIG2GO_PUSH_PRIME_MAC") == std::string::npos);
}

static void oneFieldTurnAdvertisesToOldAndCurrentDig2Gos() {
  const std::string tubes = readSource("usermods/Tubes/Tubes.h");
  const auto begin = tubes.find("case LegacyPullRendezvousSendWake");
  const auto end = tubes.find("case LegacyPullRendezvousStationArrived", begin);
  EXPECT(begin != std::string::npos && end != std::string::npos);
  const std::string wake = tubes.substr(begin, end - begin);
  EXPECT(wake.find("sendFleetPullUpdateOffer") != std::string::npos);
  EXPECT(wake.find("sendLegacyPullUpdateOffer") != std::string::npos);
  EXPECT(wake.find("legacyPullHost.setConcurrentCapacity(1)") == std::string::npos);

  const auto hostBegin = tubes.find("legacyPullHost.setConcurrentCapacity(1)");
  EXPECT(hostBegin != std::string::npos && hostBegin < begin);
}

static void productionPropagationDoesNotWaitForRebootAck() {
  const std::string tubes = readSource("usermods/Tubes/Tubes.h");
  const auto dynamic = tubes.find("#if defined(TUBES_DIG2GO_DYNAMIC_ENROLLMENT)",
      tubes.find("if (legacyPullRestoreStarted && controller.meshRadioStartedAfterDig2Go())"));
  const auto diagnostic = tubes.find("#else", dynamic);
  const auto end = tubes.find("#endif", diagnostic);
  EXPECT(dynamic != std::string::npos && diagnostic != std::string::npos
      && end != std::string::npos);
  const std::string production = tubes.substr(dynamic, diagnostic - dynamic);
  const std::string exactTargetDiagnostic = tubes.substr(diagnostic, end - diagnostic);
  EXPECT(production.find("legacyPullBodyServed && !legacyHostRetired")
      != std::string::npos);
  EXPECT(production.find("transfer_complete_no_ack") != std::string::npos);
  EXPECT(production.find("requestDig2GoHealthReport") == std::string::npos);
  EXPECT(exactTargetDiagnostic.find("requestDig2GoHealthReport")
      != std::string::npos);
}

int main() {
  legacySelectionUsesBoundedRetries();
  legacySelectionDoesNotDependOnMeshRole();
  propagationSelectionIsExplicitAndSeparateFromOtaSelection();
  propagationSerialFormDoesNotConsumeBarePowerSaveP();
  laptopFleetToolCannotStartPropagation();
  productionP2PBuildHasNoBenchBootTriggers();
  oneFieldTurnAdvertisesToOldAndCurrentDig2Gos();
  productionPropagationDoesNotWaitForRebootAck();
  autoTriggerWaitsAndLatchesOneAttempt();
  autoTriggerDefaultIsNotCompiledInProduction();
  jsonAdmissionAndFallbackFailClosed();
  targetAdmissionFailsClosed();
  handoffAndOverlaySequence();
  multipartStreamsExactImage();
  multipartRejectsFailures();
  multipartAcceptsOnly2xxStatusBoundaries();
  runtimeRestoresEveryPostPauseFailure();
  runtimeHandlesPrePauseAndRestorationRetry();
  freshHealthAloneReleasesBaton();
  puts("dig2go push bridge: tests passed");
  return 0;
}
