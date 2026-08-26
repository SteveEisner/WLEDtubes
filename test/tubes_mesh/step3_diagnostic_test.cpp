#include <cstdio>
#include <cstring>
#define TUBES_ENABLE_DIG2GO_PUSH_BRIDGE 1
#define TUBES_DIG2GO_READINESS_DELAY_TEST 1
#define TUBES_DIG2GO_PUSH_ENROLLED_MAC "5443B2B54C38"
#include "../../usermods/Tubes/dig2go_push_source_adapter.h"
using namespace tubes_p2p;
#define EXPECT(x) do { if (!(x)) { std::fprintf(stderr, "failed: %s\n", #x); return 1; } } while (0)
struct Hooks : Dig2GoPushBridgeHooks {
 uint32_t t=0; bool connected=true, local=true, gateway=true, restore=true; int pauses=0, joins=0, uploads=0, probes=0, restores=0;
 uint32_t now() const override{return t;}
 bool sendLegacyV15Selection(const uint8_t*) override{return true;}
 bool pauseTubesRadio() override{++pauses; return true;}
 bool beginExclusiveWledJoin() override{++joins; return true;}
 bool updateAccessPointConnected() const override{return connected;}
 bool updateAccessPointHasLocalIp() const override{return local;}
 bool updateAccessPointHasGateway() const override{return gateway;}
 bool probeUpdateAccessPointReachability() override{++probes; return true;}
 Dig2GoSourceAdapterResult inspectSelectedTarget(const Dig2GoTargetAdmission&,LegacyDig2GoEvidence&) override{ return Dig2GoSourceAdapterAccepted; }
 FirmwarePostResult uploadActiveImage() override{++uploads; return FirmwarePostAccepted;}
 bool restoreTubesRadio() override{++restores; return restore;}
};
int main(){
 uint8_t mac[6]={0x54,0x43,0xB2,0xB5,0x4C,0x38};
 Hooks h; Dig2GoPushBridgeRuntime r(h); EXPECT(r.arm(mac,10000)); EXPECT(r.state()==PushBridgeReadinessDelay); r.update(); EXPECT(h.pauses==0&&h.joins==0); h.t=4999; r.update(); EXPECT(h.pauses==0&&h.joins==0); h.t=5000; r.update(); EXPECT(h.pauses==1&&h.joins==1); r.update(); EXPECT(r.state()==PushBridgeHealthy); EXPECT(r.overlay()==PushOverlayComplete); EXPECT(h.uploads==0&&h.probes==0&&h.restores==1);
 Hooks retry; retry.restore=false; Dig2GoPushBridgeRuntime pending(retry); EXPECT(pending.arm(mac,10000)); retry.t=5000; pending.update(); pending.update(); EXPECT(pending.state()==PushBridgeRestoringMesh); EXPECT(retry.restores==1); retry.restore=true; pending.update(); EXPECT(pending.state()==PushBridgeHealthy); EXPECT(retry.restores==2);
 Hooks fail; fail.local=false; Dig2GoPushBridgeRuntime f(fail); EXPECT(f.arm(mac,10000)); fail.t=5000; f.update(); EXPECT(f.state()==PushBridgeMeshPaused); EXPECT(fail.joins==1&&fail.uploads==0); fail.t=10000; f.update(); EXPECT(f.state()==PushBridgeFailed); EXPECT(f.overlay()==PushOverlayFailed);
 Hooks timeout; Dig2GoPushBridgeRuntime t(timeout); EXPECT(t.arm(mac,4000)); timeout.t=4000; t.update(); EXPECT(t.state()==PushBridgeFailed); EXPECT(timeout.pauses==0&&timeout.joins==0);
 std::puts("readiness-delay join test: passed"); return 0;
}
