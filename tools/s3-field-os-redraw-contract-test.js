import assert from 'node:assert/strict';
import fs from 'node:fs';
import test from 'node:test';

const source = fs.readFileSync(
  new URL('../usermods/WaveshareS3TubesRemote/WaveshareS3TubesRemote.cpp', import.meta.url),
  'utf8',
);

test('held touch is edge-triggered and cannot repeat actions', () => {
  assert.match(source, /if \(pressed && !touchDown\) onTouch\(x, y\);\s*touchDown = pressed;/);
});

test('periodic refreshes update content without clearing whole screens', () => {
  const loop = source.match(/void loop\(\) override \{([\s\S]*?)\n  \}\n\n  void addToJsonInfo/)[1];
  assert.doesNotMatch(loop, /drawSurveyor\(\)|drawUpdate\(\)|drawChannels\(\)|fillScreen/);
  assert.match(loop, /drawSurveyorContent\(\)/);
  assert.match(loop, /drawUpdateContent\(\)/);
  assert.match(loop, /drawChannelsContent\(\)/);
});

test('Conductor redraws only changed canonical framebuffer cells', () => {
  assert.match(source, /colors\[i\] != previous\[i\]/);
  assert.match(source, /::strip\.getPixelColor\(i\)/);
  assert.doesNotMatch(source, /BusManager::getBus\([^)]*\)->getPixelColor/);
});
