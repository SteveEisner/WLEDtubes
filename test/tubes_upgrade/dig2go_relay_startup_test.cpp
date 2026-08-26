#include <cassert>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <sstream>
#include <string>
#include "../../wled00/relay_startup_policy.h"

int main() {
  for (bool present : {false, true}) {
    for (bool bootOn : {false, true}) {
      for (bool polarity : {false, true}) {
        const auto d = dig2goRelayStartup(present, bootOn, bootOn ? 64 : 0, polarity);
        assert(d.relayPresent == present);
        assert(d.relayOn == (bootOn && 64 > 0));
        assert(d.outputLevel == (polarity ? d.relayOn : !d.relayOn));
        assert(d.offMode == !d.relayOn);
      }
    }
  }
  const auto retained = dig2goRelayStartup(true, true, 0, true);
  assert(retained.relayPresent && !retained.relayOn && !retained.outputLevel);

  std::ifstream source("wled00/wled.cpp");
  std::stringstream buffer;
  buffer << source.rdbuf();
  const std::string text = buffer.str();
  const auto begin = text.find("void WLED::beginStrip()");
  const auto end = text.find("void WLED::initAP", begin);
  assert(begin != std::string::npos && end != std::string::npos);
  const std::string lifecycle = text.substr(begin, end - begin);
  assert(lifecycle.find("#if defined(TUBES_DIG2GO_RELAY_STARTUP_POLICY)") != std::string::npos);
  assert(lifecycle.find("TUBES_HARDWARE_FAMILY == TubeHardwareDig2Go") == std::string::npos);
  assert(lifecycle.find("dig2goRelayStartup") != std::string::npos);
  assert(lifecycle.find("#else") != std::string::npos);
  assert(lifecycle.find("digitalWrite(rlyPin, rlyMde ? bri > 0 : bri == 0)") != std::string::npos);
  assert(lifecycle.find("offMode = bri == 0") != std::string::npos);

  std::ifstream environments("platformio_tubes.ini");
  std::stringstream environmentBuffer;
  environmentBuffer << environments.rdbuf();
  const std::string environmentText = environmentBuffer.str();
  const auto dig2goBegin = environmentText.find("[env:esp32_quinled_dig2go_tubes]");
  const auto dig2goEnd = environmentText.find("\n[env:", dig2goBegin + 1);
  assert(dig2goBegin != std::string::npos && dig2goEnd != std::string::npos);
  const std::string dig2goEnvironment = environmentText.substr(
      dig2goBegin, dig2goEnd - dig2goBegin);
  assert(dig2goEnvironment.find("-D TUBES_DIG2GO_RELAY_STARTUP_POLICY=1")
      != std::string::npos);
  assert(environmentText.find("TUBES_DIG2GO_RELAY_STARTUP_POLICY=1")
      == dig2goEnvironment.find("TUBES_DIG2GO_RELAY_STARTUP_POLICY=1") + dig2goBegin);
}
