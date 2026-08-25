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
  return tubes.s3ForceNext();
}

bool tubesS3BroadcastFleetOffer(const FleetUpdateOffer &offer) {
  return tubes.s3BroadcastFleetOffer(offer);
}

bool tubesS3RequestDeviceReport(const uint8_t mac[6], uint32_t nonce) {
  return tubes.s3RequestDeviceReport(mac, nonce);
}
