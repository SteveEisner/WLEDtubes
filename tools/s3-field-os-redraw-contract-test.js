const assert = require('node:assert/strict');
const fs = require('node:fs');
const test = require('node:test');

const source = fs.readFileSync(
  require.resolve('../usermods/WaveshareS3TubesRemote/WaveshareS3TubesRemote.cpp'),
  'utf8',
);

test('held touch is edge-triggered and cannot repeat actions', () => {
  assert.match(source, /if \(pressed && !touchDown\) viewManager\.tap\(x, y\);\s*touchDown = pressed;/);
});

test('periodic refreshes update content without clearing whole screens', () => {
  const loop = source.match(/void loop\(\) override \{([\s\S]*?)\n  \}\n\n  void addToJsonInfo/)[1];
  assert.doesNotMatch(loop, /fillScreen|drawSurveyorContent|drawUpdateContent|drawChannelsContent/);
  assert.match(loop, /viewManager\.tick\(millis\(\)\)/);
  assert.match(source, /if \(nextRevision != lastRevision\) render\(false\)/);
  assert.match(source, /if \(nextRevision != lastTelemetryRevision\)/);
  assert.doesNotMatch(source, /else active->render\(false\)/);
});

test('all workspaces share one inherited lifecycle and one view manager', () => {
  assert.match(source, /class FieldView \{/);
  assert.match(source, /class HomeView final : public FieldView/);
  assert.match(source, /class ConductorView final : public FieldView/);
  assert.match(source, /class SurveyorView final : public FieldView/);
  assert.match(source, /class UpdateView final : public FieldView/);
  assert.match(source, /class ChannelsView final : public FieldView/);
  assert.match(source, /class ViewManager \{/);
  assert.match(source, /viewManager\.tap\(x, y\)/);
});

test('Conductor redraws only changed canonical framebuffer cells', () => {
  assert.match(source, /colors\[i\] != previous\[i\]/);
  assert.match(source, /::strip\.getPixelColor\(i\)/);
  assert.doesNotMatch(source, /BusManager::getBus\([^)]*\)->getPixelColor/);
});
