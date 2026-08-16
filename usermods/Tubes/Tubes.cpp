#include "Tubes.h"

static TubesUsermod tubes;
REGISTER_USERMOD(tubes);

bool tubesCopyReadOnlySnapshot(TubesReadOnlySnapshot& snapshot) {
  tubes.copyReadOnlySnapshot(snapshot);
  return true;
}

#ifdef TUBES_READ_ONLY_FIELD_SHELL
bool tubesS3ForcePrevious() {
  return tubes.s3ForcePrevious();
}

bool tubesS3ForceNext() {
  return tubes.s3ForceNext();
}

bool tubesS3SetBroadcastEnabled(bool enabled) {
  return tubes.setS3BroadcastEnabled(enabled);
}

bool tubesS3BroadcastEnabled() {
  return tubes.isS3BroadcastEnabled();
}
#endif
