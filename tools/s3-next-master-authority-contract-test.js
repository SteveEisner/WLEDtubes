const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const controller = fs.readFileSync('usermods/Tubes/controller.h', 'utf8');
const ui = fs.readFileSync('usermods/WaveshareS3TubesRemote/WaveshareS3TubesRemote.cpp', 'utf8');

test('Next uses Steve channel admission for Pattern and Palette', () => {
  const gate = controller.match(/bool can_force_next\(\) \{([\s\S]*?)\n  \}/)?.[1] || '';
  assert.match(gate, /if \(node\.isFollowing\(\)\) return false/);
  assert.match(gate, /channelWinners\.localMayRequest\([\s\S]*PatternChannel/);
  assert.match(gate, /channelWinners\.localMayRequest\([\s\S]*PaletteChannel/);
  assert.doesNotMatch(gate, /BeatChannel/);
});

test('authority gate precedes the only local canonical mutation', () => {
  const action = controller.match(/bool force_next_if_authoritative\(\) \{([\s\S]*?)\n  \}/)?.[1] || '';
  assert.match(action, /if \(!can_force_next\(\)\) return false;\s*force_next\(true\);\s*return true/);
  assert.doesNotMatch(action, /COMMAND_ACTION|sendV3ControlCommand|request_next/);
});

test('Field OS routes pattern tiles through the bounded S3 scheduling API', () => {
  assert.match(ui, /owner\.nextSendFailed = !tubesS3SelectPattern\(owner\.patternChoiceId\(index\)\)/);
  assert.match(controller, /if \(node\.isFollowing\(\) \|\| patternId >= gPatternCount\)/);
  assert.match(controller, /PatternProgramEntry program = tablePatternProgram/);
});

test('TubeOS gradient success means the local next-phrase schedule exists', () => {
  const schedule = controller.match(/bool scheduleS3Gradient\(const uint32_t colors\[3\]\) \{([\s\S]*?)\n  \}/)?.[1] || '';
  assert.match(schedule, /const uint16_t effectivePhrase = nextPhraseBoundary\(\)/);
  assert.match(schedule, /scheduleCustomGradient\(/);
  assert.match(schedule, /return nextPalette16Valid/);
  assert.match(schedule, /nextPalette16Phrase == effectivePhrase/);
  assert.match(schedule, /nextPalette16Fallback == fallback/);
});
