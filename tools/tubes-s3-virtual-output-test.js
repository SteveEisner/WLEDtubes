const test = require('node:test');
const {spawnSync} = require('node:child_process');
const path = require('node:path');
const assert = require('node:assert/strict');
const fs = require('node:fs');

const root = path.resolve(__dirname, '..');
const read = file => fs.readFileSync(path.join(root, file), 'utf8');

test('Waveshare S3 alone enables a 60-pixel null logical output', () => {
  const ini = read('platformio_tubes.ini');
  const s3 = ini.match(/\[env:waveshare_s3_tubes_remote\]([\s\S]*?)(?=\n\[|$)/)[1];
  assert.match(s3, /-D TUBES_NULL_OUTPUT/);
  assert.match(s3, /-D LED_TYPES=TYPE_TUBES_NULL/);
  assert.match(s3, /-D PIXEL_COUNTS=60/);
  assert.doesNotMatch(s3, /-D MASTER(?:\s|$)/);
  assert.match(s3, /-D DATA_PINS=255/); // sentinel pin; null-bus factory ignores it
  assert.doesNotMatch(s3, /(?:LEDPIN|TYPE_NET_)/);
  const dig2go = ini.match(/\[env:esp32_quinled_dig2go_tubes\]([\s\S]*?)(?=\n\[|$)/)[1];
  assert.doesNotMatch(dig2go, /TUBES_NULL_OUTPUT/);
});

test('null output is a geometry-only sink with no transport, GPIO, or private buffer', () => {
  const header = read('wled00/bus_manager.h');
  const source = read('wled00/bus_manager.cpp');
  const declaration = header.match(/class BusTubesNull[\s\S]*?#endif/)[0];
  assert.match(declaration, /getPixelColor/);
  assert.doesNotMatch(declaration, /_data|new \(std::nothrow\)/);
  const body = source.match(/#ifdef TUBES_NULL_OUTPUT([\s\S]*?)#endif/)[1];
  assert.match(body, /BusTubesNull::BusTubesNull/);
  assert.match(declaration, /setPixelColor\(unsigned pix, uint32_t c\) override \{\}/);
  assert.match(declaration, /getPixelColor\(unsigned pix\) const override \{ return 0; \}/);
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

test('S3 repairs a stale one-pixel segment before the Tubes renderer runs', () => {
  const tubes = read('usermods/Tubes/Tubes.h');
  assert.match(tubes, /needsSegments = seg\.length\(\) != strip\.getLengthTotal\(\)/);
  assert.match(tubes, /makeAutoSegments\(true\)/);
});

test('compiled host proof preserves a nonuniform 60-pixel null-bus frame', () => {
  const result = spawnSync('c++', ['-std=c++17', '-Wall', '-Wextra',
    'tools/tubes-null-bus-host-test.cpp', '-o', '/tmp/tubes-null-bus-host-test'],
    {cwd: root, encoding: 'utf8'});
  assert.equal(result.status, 0, result.stderr);
  const run = spawnSync('/tmp/tubes-null-bus-host-test', [], {encoding: 'utf8'});
  assert.equal(run.status, 0, run.stderr);
});

test('AMOLED strand reads the canonical WLED framebuffer, not the geometry-only bus', () => {
  const ui = read('usermods/WaveshareS3TubesRemote/WaveshareS3TubesRemote.cpp');
  assert.match(ui, /::strip\.getLengthTotal\(\) >= 60/);
  assert.match(ui, /::strip\.getPixelColor\(i\)/);
  assert.doesNotMatch(ui, /BusManager::getBus|addressed->getPixelColor/);
  const tubes = read('usermods/Tubes/Tubes.h');
  assert.doesNotMatch(tubes, /TUBES_S3_FRAME_DIAGNOSTICS|TUBES_S3_FRAME/);
});
