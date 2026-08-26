'use strict';

const assert = require('node:assert');
const { it } = require('node:test');
const fs = require('node:fs');
const path = require('node:path');

const source = fs.readFileSync(path.resolve(__dirname, '..',
  'usermods/WaveshareS3TubesRemote/WaveshareS3TubesRemote.cpp'), 'utf8');

it('offers scan and exact target arming from Update without touching Conductor controls', () => {
  assert.match(source, /FieldViewId::Update/);
  assert.match(source, /class UpdateView final : public FieldView/);
  assert.match(source, /void drawUpdateContent\(\)/);
  assert.match(source, /tubesS3ScanCarrierTargets\(\)/);
  assert.match(source, /tubesS3ReadCarrierTarget\(index, target\)/);
  assert.match(source, /tubesS3ArmCarrier\(target\.mac, target\.family, target\.variant, target\.release\)/);
  assert.match(source, /THIS S3 UPDATE CARRIER/);
  assert.match(source, /FIRMWARE AVAILABLE/);
  assert.match(source, /tubesS3CarrierArtifactCount\(\)/);
  assert.match(source, /target\.nodeId, target\.release, target\.uplinkId/);
  assert.doesNotMatch(source, /Previous/);
});

it('uses four focused home workspaces and removes the generic Status screen', () => {
  for (const label of ['Conductor', 'Surveyor', 'Update', 'Channels'])
    assert.match(source, new RegExp(`F\\("${label}"\\)`));
  assert.doesNotMatch(source, /FieldViewId::Status/);
  assert.match(source, /class ChannelsView final : public FieldView/);
  assert.match(source, /void drawChannelsContent\(\)/);
});
