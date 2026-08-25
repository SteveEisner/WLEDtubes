const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const controller = fs.readFileSync('usermods/Tubes/controller.h', 'utf8');
const ui = fs.readFileSync('usermods/WaveshareS3CompileCanary/WaveshareS3CompileCanary.cpp', 'utf8');

test('Next requires non-following ownership of Pattern and Palette', () => {
  const gate = controller.match(/bool can_force_next\(\) \{([\s\S]*?)\n  \}/)?.[1] || '';
  assert.match(gate, /if \(node\.isFollowing\(\)\) return false/);
  assert.match(gate, /ownsVisualChannel\(PatternChannel, now\)/);
  assert.match(gate, /ownsVisualChannel\(PaletteChannel, now\)/);
  assert.doesNotMatch(gate, /BeatChannel/);
});

test('authority gate precedes the only local canonical mutation', () => {
  const action = controller.match(/bool force_next_if_authoritative\(\) \{([\s\S]*?)\n  \}/)?.[1] || '';
  assert.match(action, /if \(!can_force_next\(\)\) return false;\s*force_next\(true\);\s*return true/);
  assert.doesNotMatch(action, /COMMAND_ACTION|sendV3ControlCommand|request_next/);
});

test('Field OS disables followers and rechecks authority on touch', () => {
  assert.match(ui, /if \(!status\.canForceNext\)/);
  assert.match(ui, /if \(status\.canForceNext\) nextSendFailed = !tubesS3ForceNext\(\)/);
});
