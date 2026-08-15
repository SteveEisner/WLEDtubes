#include "Tubes.h"

static TubesUsermod tubes;
REGISTER_USERMOD(tubes);

bool tubesCopyReadOnlySnapshot(TubesReadOnlySnapshot& snapshot) {
  tubes.copyReadOnlySnapshot(snapshot);
  return true;
}
