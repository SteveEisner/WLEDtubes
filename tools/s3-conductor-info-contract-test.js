const assert = require('assert');
const fs = require('fs');
const api = fs.readFileSync('usermods/Tubes/s3_field_api.h', 'utf8');
const tubes = fs.readFileSync('usermods/Tubes/Tubes.h', 'utf8');
const ui = fs.readFileSync('usermods/WaveshareS3TubesRemote/WaveshareS3TubesRemote.cpp', 'utf8');


assert.match(api, /currentPatternPhrase/);
assert.match(api, /nextPatternPhrase/);
assert.match(api, /beatFrame/);
assert.match(tubes, /current_state\.beat_frame >> 12/);
assert.match(tubes, /next_state\.pattern_phrase/);
assert.match(tubes, /extractModeName\(status\.patternId, JSON_mode_names, status\.patternName/);
assert.match(tubes, /extractModeName\(status\.paletteId, JSON_palette_names, status\.paletteName/);
assert.match(ui, /struct DeviceCard/);
assert.equal((ui.match(/"THIS DEVICE"/g) || []).length, 4);
assert.match(ui, /ID: %04X/);
assert.match(ui, / \| VERSION: /);
assert.match(ui, /UPLINK: %04X/);
assert.match(ui, /UPLINK: NONE/);
assert.doesNotMatch(ui, /"THIS S3 \/ CONDUCTOR"|"S3 FIELD CONSOLE"/);
assert.match(ui, /Pattern %u in %lu\.%lus  \|  blending live/);
assert.match(ui, /remainingFrames/);
assert.match(ui, /FIELD_OS_DEFAULT_BRIGHTNESS = 255/);
assert.match(ui, /display\.setBrightness\([\s\S]*FIELD_OS_DEFAULT_BRIGHTNESS/);
assert.match(ui, /Production colors stay RGB888/);
assert.match(ui, /rgb565\(color\)/);
// Accepted full-strand/master-compaction bypass remains byte-for-source.
assert.match(ui, /stripComponent\.draw\(31, 178, 420, 110/);
assert.match(ui, /Surveyor/);
assert.match(ui, /THIS DEVICE/);
assert.match(ui, /NEARBY DEVICES/);
assert.match(ui, /status\.radioChannel/);
assert.match(ui, /status\.peerCount/);
assert.match(ui, /FieldViewId::Update/);
assert.match(ui, /FieldViewId::Channels/);
assert(!/button\([^\n]+(Previous|Master|Settings)/.test(ui), 'unsupported mutating controls must be absent');
console.log('S3 conductor information contract passed');
