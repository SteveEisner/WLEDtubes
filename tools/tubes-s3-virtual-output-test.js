const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

const root = path.resolve(__dirname, '..');
const read = file => fs.readFileSync(path.join(root, file), 'utf8');

test('Waveshare S3 alone enables a 60-pixel null logical output', () => {
  const ini = read('platformio_tubes.ini');
  const s3 = ini.match(/\[env:waveshare_s3_tubes_remote\]([\s\S]*?)(?=\n\[|$)/)[1];
  assert.match(s3, /-D TUBES_NULL_OUTPUT/);
  assert.match(s3, /-D PIXEL_COUNTS=60/);
  assert.match(s3, /TUBES_HARDWARE_FAMILY=TubeHardwareWaveshareS3/);
  assert.doesNotMatch(s3, /(?:LEDPIN|DATA_PINS|TYPE_NET_)/);
  const dig2go = ini.match(/\[env:esp32_quinled_dig2go_tubes\]([\s\S]*?)(?=\n\[|$)/)[1];
  assert.doesNotMatch(dig2go, /TUBES_NULL_OUTPUT/);
});

test('S3 field shell is local and keeps Updater read-only', () => {
  const source = read('usermods/WaveshareS3CompileCanary/WaveshareS3CompileCanary.cpp');
  assert.match(source, /FieldScreen : uint8_t \{ Home, Conductor, Surveyor, Updater \}/);
  assert.match(source, /No peer erase, write, reboot/);
  assert.doesNotMatch(source, /Update\.begin|Update\.write|esp_ota_begin|ESP\.restart/);
  const report = read('usermods/Tubes/device_report_protocol.h');
  assert.match(report, /TubeHardwareWaveshareS3 = 6/);
});

test('null output is an internal framebuffer bus with no transport or GPIO path', () => {
  const header = read('wled00/bus_manager.h');
  const source = read('wled00/bus_manager.cpp');
  assert.match(header, /class BusTubesNull[\s\S]*getPixelColor/);
  assert.match(header, /uint32_t \*_data/);
  const body = source.match(/#ifdef TUBES_NULL_OUTPUT([\s\S]*?)#endif/)[1];
  assert.match(body, /BusTubesNull::setPixelColor/);
  assert.match(body, /BusTubesNull::getPixelColor/);
  assert.doesNotMatch(body, /(?:UDP|RMT|I2S|PinManager|pinMode|digitalWrite|Network)/);
  assert.match(source, /TUBES_NULL_OUTPUT[\s\S]*make_unique<BusTubesNull>/);
});

test('Tubes S3 setup requests only the null bus and keeps WLED effects on the real strip engine', () => {
  const tubes = read('usermods/Tubes/Tubes.h');
  const pattern = read('usermods/Tubes/pattern.h');
  assert.match(tubes, /#ifdef TUBES_NULL_OUTPUT[\s\S]*TYPE_TUBES_NULL[\s\S]*PIXEL_COUNTS/);
  assert.match(pattern, /void draw_wled_fx\(VirtualStrip \*strip\)/);
  assert.match(pattern, /static const uint8_t numInternalPatterns = 24/);
  assert.match(pattern, /\{FX_MODE_[A-Z0-9_]+, draw_wled_fx/);
  const controller = read('usermods/Tubes/controller.h');
  assert.match(controller, /strip\.getPixelColor\(i\)/);
});
