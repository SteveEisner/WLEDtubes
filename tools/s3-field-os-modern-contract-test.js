const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const ui = fs.readFileSync('usermods/WaveshareS3CompileCanary/WaveshareS3CompileCanary.cpp','utf8');
const tubes = fs.readFileSync('usermods/Tubes/Tubes.h','utf8');
const bridge = fs.readFileSync('usermods/Tubes/Tubes.cpp','utf8');

test('Conductor remains on canonical completed WLED framebuffer', () => {
  assert.match(ui, /::strip\.getPixelColor\(i\)/);
  assert.doesNotMatch(ui, /BusManager::getBus\([^)]*\)->getPixelColor/);
});
test('Field OS offers Next but no Previous or authority mutation', () => {
  assert.match(ui, /F\("Next"\)/);
  assert.match(bridge, /tubes\.s3ForceNext\(\)/);
  assert.match(tubes, /controller\.force_next\(true\)/);
  assert.doesNotMatch(ui, /Previous|SetMasterAuthority|SetAnchorAuthority|Update\.begin|esp_ota_begin/);
  assert.doesNotMatch(tubes, /force_next_pattern/);
});
test('Surveyor is bounded, fresh, read-only, and exposes modern channel winners', () => {
  assert.match(ui, /TubesS3PeerStatus sorted\[7\]/);
  assert.match(ui, /now - candidate\.lastSeenMs > 60000/);
  assert.match(ui, /status\.beatChannel/);
  assert.match(ui, /status\.patternChannel/);
  assert.match(ui, /status\.paletteChannel/);
});
