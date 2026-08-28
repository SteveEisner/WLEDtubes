const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const ui = fs.readFileSync('usermods/WaveshareS3TubesRemote/WaveshareS3TubesRemote.cpp','utf8');
const tubes = fs.readFileSync('usermods/Tubes/Tubes.h','utf8');
const bridge = fs.readFileSync('usermods/Tubes/Tubes.cpp','utf8');
const controller = fs.readFileSync('usermods/Tubes/controller.h','utf8');

test('Patterns remains on canonical completed WLED framebuffer', () => {
  assert.match(ui, /::strip\.getPixelColor\(i\)/);
  assert.doesNotMatch(ui, /BusManager::getBus\([^)]*\)->getPixelColor/);
});
test('Field OS offers direct pattern and gradient scheduling without legacy controls', () => {
  assert.match(ui, /tubesS3SelectPattern/);
  assert.match(ui, /tubesS3ScheduleGradient/);
  assert.match(bridge, /return tubes\.s3ForceNext\(\)/);
  assert.match(tubes, /return controller\.force_next_if_authoritative\(\)/);
  assert.doesNotMatch(ui, /Previous|SetMasterAuthority|SetAnchorAuthority|Update\.begin|esp_ota_begin|F\("Channels"\)/);
  assert.doesNotMatch(tubes, /force_next_pattern\(/);
  assert.match(ui, /COULD NOT SET PATTERN/);
});
test('Mesh is bounded, fresh, and read-only', () => {
  const mesh = ui.match(/void drawMeshContent\(\) \{([\s\S]*?)\n  \}\n\n  static uint32_t blendColor/)[1];
  assert.match(ui, /TubesS3PeerStatus sorted\[4\]/);
  assert.match(ui, /now - candidate\.lastSeenMs > 60000/);
  assert.match(ui, /class MeshView final : public FieldView/);
  assert.doesNotMatch(mesh, /THIS S3|THIS DEVICE|NEARBY DEVICES/);
  assert.match(ui, /drawDeviceCard\(106 \+ i \* 70/);
  assert.match(ui, /Rect\{120, 408, 240, 56\}[\s\S]*FieldViewId::Update/);
});

test('Colors queues every edit and regenerates two rows of anchored presets', () => {
  assert.match(ui, /FieldViewId::Colors/);
  assert.match(ui, /class ColorsView final : public FieldView/);
  assert.match(ui, /uint32_t gradientStops\[3\]/);
  assert.match(ui, /uint32_t gradientPresets\[8\]\[3\]/);
  assert.match(ui, /void regenerateGradientPresets\(\)/);
  assert.match(ui, /gradientPresets\[preset\]\[stop\] = anchor/);
  assert.match(ui, /for \(uint8_t preset = 0; preset < 8; preset\+\+\)/);
  assert.match(ui, /const int16_t y = 312 \+ \(preset \/ 4\) \* 64/);
  assert.match(ui, /owner\.regenerateGradientPresets\(\);\s*owner\.queueGradient\(\)/);
  assert.match(ui, /void applyGradientPreset[\s\S]*queueGradient\(\)/);
  assert.doesNotMatch(ui, /EDIT NEXT GRADIENT|GRADIENT QUEUED|F\("Set next"\)/);
  assert.doesNotMatch(ui, /FieldViewId::Channels|class ChannelsView|LIVE CHANNEL AUTHORITY/);
  assert.match(ui, /hueFromPaletteX\(x\)/);
  assert.match(ui, /for \(int16_t x = 24; x < 456; x \+= hueSliceWidth\)/);
  assert.doesNotMatch(ui, /gradientHue/);
});

test('Beats controls listening, downbeat, and a bounded overlay grid', () => {
  assert.match(ui, /class BeatsView final : public FieldView/);
  assert.match(ui, /tubesS3SetTempoListening\(!status\.tempoListening\)/);
  assert.match(ui, /tubesS3TapDownbeat\(\)/);
  assert.match(ui, /tubesS3SelectBeatOverlay\(row \* 3 \+ column\)/);
  assert.match(ui, /for \(uint8_t choice = 0; choice < 9; choice\+\+\)/);
  assert.match(bridge, /return tubes\.s3SetTempoListening\(enabled\)/);
  assert.match(bridge, /return tubes\.s3TapDownbeat\(\)/);
  assert.match(bridge, /return tubes\.s3SelectBeatOverlay\(choice\)/);
  assert.match(controller, /sound\.setTempoTracking\(enabled\)/);
  assert.match(controller, /beats\.start_phrase\(\);\s*update_beat\(\);\s*publishApplicationChannel\(BeatChannel\);\s*send_update\(\)/);
  assert.match(controller, /choice > S3_BEAT_OVERLAY_COUNT/);
  assert.match(controller, /gSoundPrograms\[choice - 1\]/);
  assert.doesNotMatch(controller, /AUTO_TEMPO pinned on/);
});
