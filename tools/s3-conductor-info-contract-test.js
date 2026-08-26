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
assert.match(tubes, /snprintf\(status\.patternName, sizeof\(status\.patternName\), "Pattern %u", status\.patternId\)/);
assert.match(ui, /S3 %03X/);
assert.match(ui, /Pattern %u in %lu\.%lus  \|  blending live/);
assert.match(ui, /remainingFrames/);
assert.match(ui, /FIELD_OS_DEFAULT_BRIGHTNESS = 255/);
assert.match(ui, /display\.setBrightness\([\s\S]*FIELD_OS_DEFAULT_BRIGHTNESS/);
assert.match(ui, /Production colors stay RGB888/);
assert.match(ui, /rgb565\(color\)/);
// Accepted full-strand/master-compaction bypass remains byte-for-source.
assert.match(ui, /stripComponent\.draw\(31, 156, 420, 138\)/);
assert.match(ui, /Surveyor/);
assert.match(ui, /THIS S3/);
assert.match(ui, /NEARBY DEVICES/);
assert.match(ui, /status\.radioChannel/);
assert.match(ui, /status\.peerCount/);
assert.match(ui, /FieldScreen::Update/);
assert.match(ui, /FieldScreen::Channels/);
assert(!/button\([^\n]+(Previous|Master|Settings)/.test(ui), 'unsupported mutating controls must be absent');
console.log('S3 conductor information contract passed');
