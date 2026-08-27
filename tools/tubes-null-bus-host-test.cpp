#include <cassert>
#include <cstdint>

#include "../wled00/bus_factory_classification.h"

constexpr uint8_t TYPE_TUBES_NULL = 96;
constexpr uint8_t TYPE_VIRTUAL_MIN = 80;
constexpr uint8_t TYPE_VIRTUAL_MAX = 95;

struct BusConfigModel {
  uint8_t type;
  unsigned count;
  unsigned pin;
};

static BusConfigModel parseOnePinDefaults(uint8_t type, unsigned pin, unsigned count) {
  return {type, count, pin};
}

int main() {
  static_assert(classifyBusFactoryType(TYPE_TUBES_NULL, TYPE_TUBES_NULL,
                                       TYPE_VIRTUAL_MIN, TYPE_VIRTUAL_MAX) ==
                BusFactoryKind::TubesNull, "null classification");
  assert(classifyBusFactoryType(TYPE_TUBES_NULL, TYPE_TUBES_NULL,
                                TYPE_VIRTUAL_MIN, TYPE_VIRTUAL_MAX) ==
         BusFactoryKind::TubesNull);

  // DATA_PINS=255 follows the ordinary one-pin loader path. The target-only
  // sentinel is consumed by BusTubesNull and never reaches a digital driver.
  const BusConfigModel config = parseOnePinDefaults(TYPE_TUBES_NULL, 255, 60);
  assert(config.type == TYPE_TUBES_NULL);
  assert(config.count == 60);
  assert(config.pin == 255);

  // The factory classification selects BusTubesNull, not BusDigital or a
  // network bus, while preserving the normal single BusConfig shape.
  const auto kind = classifyBusFactoryType(config.type, TYPE_TUBES_NULL,
                                           TYPE_VIRTUAL_MIN, TYPE_VIRTUAL_MAX);
  assert(kind == BusFactoryKind::TubesNull);
  return 0;
}
