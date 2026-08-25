const assert = require('assert');
const fs = require('fs');
const api = fs.readFileSync('usermods/Tubes/s3_field_api.h', 'utf8');
const tubes = fs.readFileSync('usermods/Tubes/Tubes.h', 'utf8');
const ui = fs.readFileSync('usermods/WaveshareS3CompileCanary/WaveshareS3CompileCanary.cpp', 'utf8');


assert.match(api, /currentPatternPhrase/);
assert.match(api, /nextPatternPhrase/);
assert.match(tubes, /current_state\.beat_frame >> 12/);
assert.match(tubes, /next_state\.pattern_phrase/);
assert.match(tubes, /snprintf\(status\.patternName, sizeof\(status\.patternName\), "Pattern %u", status\.patternId\)/);
assert.match(ui, /S3 %03X/);
assert.match(ui, /Next in %lus/);
assert.match(ui, /60000UL/);
assert.match(ui, /FIELD_OS_DEFAULT_BRIGHTNESS = 255/);
assert.match(ui, /display\.setBrightness\([\s\S]*FIELD_OS_DEFAULT_BRIGHTNESS/);
assert.match(ui, /Production colors stay RGB888/);
assert.match(ui, /rgb565\(color\)/);
// Accepted full-strand/master-compaction bypass remains byte-for-source.
assert.match(ui, /stripComponent\.draw\(31, 156, 420, 138\)/);
assert(!/Surveyor/.test(ui), 'Surveyor must be absent from the production surface');
assert.match(ui, /Local S3 %03X/);
assert.match(ui, /status\.radioChannel/);
assert.match(ui, /status\.preview\[0\]/);
assert.doesNotMatch(ui, /peerCount|uplink|syncSource|receivedPacket|lastPacket|synchronizedPacket|lastSync|transmittedPacket|lastTransmit|Waiting for sync|last RX|TX [0-9]/);
assert(!/button\([^\n]+(Next|Previous|Master|Update|Settings)/.test(ui), 'mutating controls must be absent');
console.log('S3 conductor information contract passed');
