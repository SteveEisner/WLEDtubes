const assert = require('node:assert/strict');
const fs = require('node:fs');
const test = require('node:test');

const source = fs.readFileSync(
  require.resolve('../usermods/WaveshareS3TubesRemote/WaveshareS3TubesRemote.cpp'),
  'utf8',
);

test('held touch is edge-triggered and cannot repeat actions', () => {
  assert.match(source, /if \(screenOn && pressed\) \{\s*lastActivityMs = millis\(\);\s*if \(!touchDown\) viewManager\.tap\(x, y\);\s*\}\s*touchDown = pressed;/);
});

test('periodic refreshes update content without clearing whole screens', () => {
  const loop = source.match(/void loop\(\) override \{([\s\S]*?)\n  \}\n\n  void addToJsonInfo/)[1];
  assert.doesNotMatch(loop, /fillScreen|drawMeshContent|drawUpdateContent|drawColorsContent/);
  assert.match(loop, /if \(screenOn\) viewManager\.tick\(now\)/);
  assert.match(source, /if \(nextRevision != lastRevision\) render\(false\)/);
  assert.doesNotMatch(source, /else active->render\(false\)/);
});

test('all workspaces share one inherited lifecycle and one view manager', () => {
  assert.match(source, /class FieldView \{/);
  assert.match(source, /class HomeView final : public FieldView/);
  assert.match(source, /class PatternsView final : public FieldView/);
  assert.match(source, /class BeatsView final : public FieldView/);
  assert.match(source, /class MeshView final : public FieldView/);
  assert.match(source, /class ColorsView final : public FieldView/);
  assert.match(source, /class UpdateView final : public FieldView/);
  assert.match(source, /class ViewManager \{/);
  assert.match(source, /viewManager\.tap\(x, y\)/);
});

test('strip redraws the canonical framebuffer in one opaque transfer', () => {
  assert.match(source, /draw16bitRGBBitmap\(x, y, frame, width, height\)/);
  assert.match(source, /::strip\.getPixelColor\(i\)/);
  assert.doesNotMatch(source, /BusManager::getBus\([^)]*\)->getPixelColor/);
});
