#include "Tubes.h"

static TubesUsermod tubes;
REGISTER_USERMOD(tubes);

bool tubesS3ReadStatus(TubesS3FieldStatus &status) {
  tubes.readS3FieldStatus(status);
  return true;
}

bool tubesS3ReadPeer(size_t index, TubesS3PeerStatus &peer) {
  return tubes.readS3Peer(index, peer);
}

bool tubesS3ForceNext() {
  tubes.s3ForceNext();
  return true;
}
