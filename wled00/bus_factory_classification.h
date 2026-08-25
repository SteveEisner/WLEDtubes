#pragma once

#include <cstdint>

enum class BusFactoryKind : uint8_t {
  TubesNull,
  VirtualNetwork,
  Other
};

constexpr BusFactoryKind classifyBusFactoryType(uint8_t type, uint8_t tubesNullType,
                                                uint8_t virtualMin, uint8_t virtualMax) {
  // The null framebuffer must win before the generic virtual/network range.
  return type == tubesNullType ? BusFactoryKind::TubesNull
       : (type >= virtualMin && type <= virtualMax ? BusFactoryKind::VirtualNetwork
                                                    : BusFactoryKind::Other);
}
