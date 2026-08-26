#include <cstdio>
#include <cstring>
#define TUBES_ENABLE_DIG2GO_PUSH_BRIDGE 1
#define TUBES_DIG2GO_INSPECTION_ONLY_TEST 1
#define TUBES_DIG2GO_PUSH_ENROLLED_MAC "5443B2B54C38"
#include "../../usermods/Tubes/dig2go_push_source_adapter.h"
using namespace tubes_p2p;
#define EXPECT(x) do { if (!(x)) { std::fprintf(stderr, "failed: %s\n", #x); return 1; } } while (0)

struct Hooks : Dig2GoPushBridgeHooks {
  uint32_t now() const override { return clock; }
  bool sendLegacyV15Selection(const uint8_t*) override { return true; }
  bool pauseTubesRadio() override { return true; }
  bool beginExclusiveWledJoin() override { return true; }
  bool updateAccessPointConnected() const override { return true; }
  bool probeUpdateAccessPointReachability() override { return true; }
  Dig2GoSourceAdapterResult inspectSelectedTarget(
      const Dig2GoTargetAdmission&, LegacyDig2GoEvidence&) override {
    inspections++;
    if (httpFailures-- > 0) return Dig2GoSourceAdapterHttpFailed;
    return inspectionResult;
  }
  FirmwarePostResult uploadActiveImage() override { uploads++; return FirmwarePostAccepted; }
  bool restoreTubesRadio() override { restores++; return true; }
  Dig2GoSourceAdapterResult inspectionResult = Dig2GoSourceAdapterAccepted;
  mutable uint32_t clock = 100;
  int httpFailures = 0;
  int inspections = 0;
  int uploads = 0;
  int restores = 0;
};

int main() {
  const uint8_t mac[6] = {0x54,0x43,0xB2,0xB5,0x4C,0x38};
  Hooks pass;
  Dig2GoPushBridgeRuntime runtime(pass);
  EXPECT(runtime.arm(mac, 30000)); runtime.update(); runtime.update(); runtime.update();
  EXPECT(runtime.state() == PushBridgeHealthy);
  EXPECT(pass.inspections == 1 && pass.uploads == 0 && pass.restores == 1);

  Hooks reject;
  reject.inspectionResult = Dig2GoSourceAdapterIdentityRejected;
  Dig2GoPushBridgeRuntime failed(reject);
  EXPECT(failed.arm(mac, 30000)); failed.update(); failed.update(); failed.update();
  EXPECT(failed.state() == PushBridgeFailed);
  EXPECT(reject.inspections == 1 && reject.uploads == 0 && reject.restores == 1);

  Hooks delayed;
  delayed.httpFailures = 2;
  Dig2GoPushBridgeRuntime retry(delayed);
  EXPECT(retry.arm(mac, 30000)); retry.update(); retry.update(); retry.update();
  EXPECT(retry.state() == PushBridgeApJoined && delayed.inspections == 1);
  delayed.clock += 1000; retry.update();
  EXPECT(retry.state() == PushBridgeApJoined && delayed.inspections == 2);
  delayed.clock += 1000; retry.update(); retry.update();
  EXPECT(retry.state() == PushBridgeHealthy);
  EXPECT(delayed.inspections == 3 && delayed.uploads == 0 && delayed.restores == 1);
  std::puts("inspection-only diagnostic: passed");
  return 0;
}
