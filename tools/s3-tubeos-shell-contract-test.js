const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');

const source = fs.readFileSync(
  require.resolve('../usermods/WaveshareS3TubesRemote/WaveshareS3TubesRemote.cpp'),
  'utf8',
);

// Proves the permanent shell has the requested readable home affordance and app layout.
test('TubeOS header is large, avoids home redraws, and preserves the four-app grid', () => {
  assert.match(source, /setHeaderFont\(\);\s*display\.setCursor\(14, 12\);\s*display\.print\(F\("tubeOS"\)\)/);
  assert.match(source, /display\.setCursor\(220, 24\);\s*display\.printf\("%04X v%u", status\.localNodeId, status\.tubesVersion\)/);
  assert.match(source, /display\.fillRect\(385, 0, 95, FIELD_OS_HEADER_HEIGHT, COLOR_BACKGROUND\)/);
  assert.match(source, /if \(y < FIELD_OS_HEADER_HEIGHT\) \{\s*if \(active->id\(\) != FieldViewId::Home\) navigate\(FieldViewId::Home\)/);
  assert.match(source, /\{20, 128, 210, 150\}[\s\S]*F\("Patterns"\)/);
  assert.match(source, /\{250, 128, 210, 150\}[\s\S]*F\("Beats"\)/);
  assert.match(source, /\{20, 294, 210, 150\}[\s\S]*F\("Colors"\)/);
  assert.match(source, /\{250, 294, 210, 150\}[\s\S]*F\("Mesh"\)/);
  assert.doesNotMatch(source, /F\("Channels"\)/);
});

// Proves every text role uses the CO5300-safe built-in font at a readable scale.
test('all TubeOS text roles use scaled built-in fonts', () => {
  const fieldOs = source.match(/class WaveshareS3FieldOs[\s\S]*?#else\s*class WaveshareS3PeripheralSmoke/)[0];
  assert.equal((fieldOs.match(/display\.setFont\(nullptr\)/g) || []).length, 4);
  assert.match(fieldOs, /void setDetailFont\(\)[\s\S]*display\.setTextSize\(2\)/);
  assert.match(fieldOs, /void setBodyFont\(\)[\s\S]*display\.setTextSize\(2\)/);
  assert.match(fieldOs, /void setAppFont\(\)[\s\S]*display\.setTextSize\(3\)/);
  assert.match(fieldOs, /void setHeaderFont\(\)[\s\S]*display\.setTextSize\(5\)/);
});

// Proves every TubeOS label uses the documented opaque CO5300 rendering path.
test('all TubeOS text supplies a background color', () => {
  const fieldOs = source.match(/class WaveshareS3FieldOs[\s\S]*?#else\s*class WaveshareS3PeripheralSmoke/)[0];
  const textColorCalls = fieldOs.match(/display\.setTextColor\([^\n]+\);/g) || [];
  assert.ok(textColorCalls.length > 10);
  assert.ok(textColorCalls.every((call) => call.includes(',')));
  assert.match(source, /Arduino_CO5300 display\(&displayBus/);
  assert.match(source, /Arduino_GFX\/issues\/780/);
  assert.doesNotMatch(source, /class TubesCO5300/);
});

// Proves all 128 virtual pixels form one opaque bar above a complete 16-beat lane.
test('live strip uses one gapless bitmap transfer with beat and downbeat shading', () => {
  assert.match(source, /FIELD_OS_STRIP_HEIGHT = 32/);
  assert.match(source, /PREVIEW_INTERVAL_MS = 50/);
  assert.match(source, /i < TUBES_S3_PREVIEW_PIXELS/);
  assert.match(source, /static_cast<uint32_t>\(column\)\s*\* TUBES_S3_PREVIEW_PIXELS \/ width/);
  assert.match(source, /const uint8_t beat = status\.beat & 0x0F/);
  assert.match(source, /const int16_t laneTop = height \* 3 \/ 4/);
  assert.match(source, /const uint8_t segment = static_cast<uint32_t>\(column\) \* 16 \/ width/);
  assert.match(source, /segment % 4 == 0 \? COLOR_DOWNBEAT : COLOR_BEAT/);
  assert.match(source, /segment == beat\s*\? RGB565_WHITE/);
  assert.match(source, /display\.draw16bitRGBBitmap\(x, y, frame, width, height\)/);
  assert.doesNotMatch(source, /markerLeft|markerRight|fillCell/);
});

// Proves the header distinguishes plugged and battery operation without scaled text.
test('header battery omits percent text but reports charging and unplugged pulse state', () => {
  assert.match(source, /pmu\.getBatteryPercent\(\)/);
  assert.match(source, /pmu\.isCharging\(\)/);
  assert.match(source, /drawBatteryIcon\(usbPresent \? COLOR_MINT : batteryPulseColor\(millis\(\)\), usbPresent\)/);
  assert.match(source, /if \(!showChargingSymbol\) return/);
  assert.match(source, /display\.fillTriangle\([\s\S]*RGB565_WHITE\)/);
  assert.match(source, /BATTERY_PULSE_CYCLE_MS = 2400/);
  assert.doesNotMatch(source, /display\.printf\("%d%%"|F\("--%"\)/);
});

// Proves battery inactivity, physical wake, manual sleep, and hardware restart are wired.
test('display power follows USB, inactivity, and physical-button policy', () => {
  assert.match(source, /FIELD_OS_IDLE_TIMEOUT_MS = 30000/);
  assert.match(source, /if \(wasUsbPresent && !usbPresent\) lastActivityMs = now/);
  assert.match(source, /if \(!usbPresent && now - lastActivityMs >= FIELD_OS_IDLE_TIMEOUT_MS\)/);
  assert.match(source, /display\.displayOff\(\)/);
  assert.match(source, /samplePower\(now\);\s*if \(!screenOn\) return;/);
  assert.match(source, /if \(!wasUsbPresent && usbPresent && !screenOn\) setScreenOn\(true\)/);
  assert.match(source, /if \(pmu\.isPekeyShortPressIrq\(\)\)[\s\S]*setScreenOn\(!screenOn\)/);
  assert.match(source, /if \(pressed && !physicalButtonDown\[b\]\)[\s\S]*if \(!screenOn\) setScreenOn\(true\)/);
  assert.match(source, /pmu\.setPowerKeyPressOffTime\(XPOWERS_POWEROFF_10S\)/);
  assert.match(source, /pmu\.setLongPressRestart\(\)/);
});

// Proves every detected touch refreshes inactivity while navigation remains edge-triggered.
test('screen touch continuously resets the battery timeout', () => {
  assert.match(source, /if \(screenOn && pressed\) \{\s*lastActivityMs = millis\(\);\s*if \(!touchDown\) viewManager\.tap\(x, y\);/);
});

// Proves the onboard codec is initialized for the same pins and rate used by AudioReactive.
test('onboard ES7210 microphones are initialized for 16 kHz beat tracking', () => {
  assert.match(source, /initializeMicrophones\(\)/);
  assert.match(source, /writeEs7210\(0x02, 0xC1\)/);
  assert.match(source, /writeEs7210\(0x45, 0x1E\)/);
  assert.match(source, /writeEs7210\(0x46, 0x1E\)/);
  assert.match(source, /writeEs7210\(0x11, 0x60\)/);
});
