'use strict';

const assert = require('node:assert');
const { it } = require('node:test');
const fs = require('node:fs');
const path = require('node:path');

const source = fs.readFileSync(path.resolve(__dirname, '..',
  'usermods/WaveshareS3TubesRemote/WaveshareS3TubesRemote.cpp'), 'utf8');

it('offers scan and exact target arming from Update without touching Conductor controls', () => {
  assert.match(source, /FieldScreen::Update/);
  assert.match(source, /void drawUpdate\(\)/);
  assert.match(source, /tubesS3ScanCarrierTargets\(\)/);
  assert.match(source, /tubesS3ReadCarrierTarget\(index, target\)/);
  assert.match(source, /tubesS3ArmCarrier\(target\.mac, target\.family, target\.variant, target\.release\)/);
  assert.doesNotMatch(source, /Previous/);
});

it('uses four focused home workspaces and removes the generic Status screen', () => {
  for (const label of ['Conductor', 'Surveyor', 'Update', 'Channels'])
    assert.match(source, new RegExp(`F\\("${label}"\\)`));
  assert.doesNotMatch(source, /FieldScreen::Status/);
  assert.match(source, /void drawChannels\(\)/);
});
