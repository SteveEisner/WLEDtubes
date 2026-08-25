#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr size_t TUBES_S3_PREVIEW_PIXELS = 60;
constexpr size_t TUBES_S3_PATTERN_NAME_LENGTH = 24;

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

  char patternName[TUBES_S3_PATTERN_NAME_LENGTH] = {};
  uint32_t preview[TUBES_S3_PREVIEW_PIXELS] = {};
};

bool tubesS3ReadStatus(TubesS3FieldStatus &status);
