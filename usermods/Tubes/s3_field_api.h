#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr size_t TUBES_S3_PREVIEW_PIXELS = 60;
constexpr size_t TUBES_S3_PATTERN_NAME_LENGTH = 24;

struct TubesS3ChannelStatus {
  bool active = false;
  uint16_t localChannelId = 0;
  uint16_t ownerChannelId = 0;
  uint16_t ownerControlId = 0;
  uint32_t sourceSession = 0;
  uint16_t sequence = 0;
  uint32_t leaseRemainingMs = 0;
};

struct TubesS3PeerStatus {
  uint16_t nodeId = 0;
  uint16_t uplinkId = 0;
  uint32_t lastSeenMs = 0;
  uint32_t samples = 0;
  int8_t latestRssi = 0;
  uint8_t protocolGeneration = 0;
  bool rssiKnown = false;
};

struct TubesS3FieldStatus {
  bool isMaster = false;
  bool isFollowing = false;
  bool radioReady = false;
  bool powerSave = false;
  uint8_t role = 0;
  uint8_t radioChannel = 0;
  uint8_t patternId = 0;
  uint8_t paletteId = 0;
  uint16_t bpm = 0;
  uint8_t beat = 0;
  uint16_t currentPatternPhrase = 0;
  uint16_t nextPatternPhrase = 0;
  uint16_t localNodeId = 0;
  uint16_t uplinkId = 0;
  uint8_t currentSyncMode = 0;
  uint8_t nextPatternId = 0;
  uint8_t nextSyncMode = 0;
  uint16_t currentPalettePhrase = 0;
  uint16_t nextPalettePhrase = 0;
  uint8_t nextPaletteId = 0;
  size_t peerCount = 0;
  TubesS3ChannelStatus beatChannel;
  TubesS3ChannelStatus patternChannel;
  TubesS3ChannelStatus paletteChannel;

  char patternName[TUBES_S3_PATTERN_NAME_LENGTH] = {};
  uint32_t preview[TUBES_S3_PREVIEW_PIXELS] = {};
};

bool tubesS3ReadStatus(TubesS3FieldStatus &status);
bool tubesS3ReadPeer(size_t index, TubesS3PeerStatus &peer);
bool tubesS3ForceNext();
