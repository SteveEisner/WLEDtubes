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
  assert.match(source, /tubesCopyReadOnlySnapshot/);
  assert.doesNotMatch(source, /TUBES S3 v14|Tubes release: 14/);
  const controller = read('usermods/Tubes/controller.h');
  assert.match(controller, /TUBES_READ_ONLY_FIELD_SHELL[\s\S]*RebootOperation[\s\S]*UpdateOperation[\s\S]*UpdateOfferOperation[\s\S]*SelectOperation/);
  assert.match(controller, /case COMMAND_UPGRADE:[\s\S]*TUBES_READ_ONLY_FIELD_SHELL[\s\S]*return false/);
  const report = read('usermods/Tubes/device_report_protocol.h');
  assert.match(report, /TubeHardwareWaveshareS3 = 6/);
});

test('S3 I2C probing has one owner and preview repaint is dirty-cell bounded', () => {
  const source = read('usermods/WaveshareS3CompileCanary/WaveshareS3CompileCanary.cpp');
  assert.equal((source.match(/Wire\.begin\(/g) || []).length, 1);
  assert.match(source, /probeOnce\(0x34\)/);
  assert.match(source, /probeOnce\(0x6B\)/);
  assert.match(source, /previewColors\[index\] == color/);
});

test('null output is an internal framebuffer bus with no transport or GPIO path', () => {
  const header = read('wled00/bus_manager.h');
  const source = read('wled00/bus_manager.cpp');
  assert.match(header, /class BusVirtualFramebuffer[\s\S]*getPixelColor/);
  assert.match(header, /uint32_t \*_data/);
  const body = source.match(/#ifdef TUBES_NULL_OUTPUT([\s\S]*?)#endif/)[1];
  assert.match(body, /BusVirtualFramebuffer::setPixelColor/);
  assert.match(body, /BusVirtualFramebuffer::getPixelColor/);
  assert.doesNotMatch(body, /(?:PinManager|pinMode|digitalWrite|Network\.|beginPacket|rmt_|i2s_)/);
  assert.match(source, /TUBES_NULL_OUTPUT[\s\S]*make_unique<BusVirtualFramebuffer>/);
});

test('Tubes S3 setup requests only the null bus and keeps WLED effects on the real strip engine', () => {
  const tubes = read('usermods/Tubes/Tubes.h');
  const pattern = read('usermods/Tubes/pattern.h');
  assert.match(tubes, /#ifdef TUBES_NULL_OUTPUT[\s\S]*TYPE_VIRTUAL_FRAMEBUFFER_RGB[\s\S]*PIXEL_COUNTS/);
  assert.match(pattern, /void draw_wled_fx\(VirtualStrip \*strip\)/);
  assert.match(pattern, /static const uint8_t numInternalPatterns = 24/);
  assert.match(pattern, /\{FX_MODE_[A-Z0-9_]+, draw_wled_fx/);
  const controller = read('usermods/Tubes/controller.h');
  assert.match(controller, /strip\.getPixelColor\(i\)/);
});

test('framebuffer taxonomy is unique and normal add semantics are mutation-safe', () => {
  const constants = read('wled00/const.h');
  assert.match(constants, /TYPE_VIRTUAL_FRAMEBUFFER_RGB\s+83/);
  assert.match(constants, /static_assert\(TYPE_VIRTUAL_FRAMEBUFFER_RGB >= TYPE_VIRTUAL_MIN/);
  assert.match(constants, /TYPE_VIRTUAL_FRAMEBUFFER_RGB != TYPE_NET_DDP_RGB[\s\S]*TYPE_NET_ARTNET_RGBW/);

  const source = read('wled00/bus_manager.cpp');
  const add = source.match(/int BusManager::add\(const BusConfig &bc, bool placeholder\)([\s\S]*?)\n}\n\n/);
  assert.ok(add, 'could not isolate BusManager::add');
  assert.doesNotMatch(add[1], /busses\.clear\(/);
  assert.match(add[1], /bc\.type != TYPE_VIRTUAL_FRAMEBUFFER_RGB[\s\S]*return -1/);
  assert.match(add[1], /make_unique<BusVirtualFramebuffer>[\s\S]*!framebuffer->isOk\(\)[\s\S]*errorFlag = ERR_(?:NORAM|LOW_MEM)[\s\S]*return -1[\s\S]*busses\.push_back/);
});

test('Tubes owns idempotent exactly-one framebuffer initialization', () => {
  const tubes = read('usermods/Tubes/Tubes.h');
  assert.match(tubes, /hasExpectedLogicalOutput\(\)/);
  assert.match(tubes, /getNumBusses\(\) != 1/);
  assert.match(tubes, /getType\(\) == TYPE_VIRTUAL_FRAMEBUFFER_RGB/);
  assert.match(tubes, /busConfigs\.size\(\) == 1/);
  assert.match(tubes, /busConfigs\[0\]\.type == TYPE_VIRTUAL_FRAMEBUFFER_RGB/);
  assert.match(tubes, /if \(BusManager::getNumBusses\(\) > 0\)[\s\S]*!hasExpectedLogicalOutput\(\)[\s\S]*return;/);
  assert.match(tubes, /if \(!busConfigs\.empty\(\)\)[\s\S]*!hasExpectedLogicalOutputConfig\(\)[\s\S]*return;/);
});
