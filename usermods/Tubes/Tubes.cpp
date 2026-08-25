#include "Tubes.h"

static TubesUsermod tubes;
REGISTER_USERMOD(tubes);

bool tubesS3ReadStatus(TubesS3FieldStatus &status) {
  tubes.readS3FieldStatus(status);
  return true;
}
