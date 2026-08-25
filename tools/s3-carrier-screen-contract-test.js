'use strict';

const assert = require('node:assert');
const { it } = require('node:test');
const fs = require('node:fs');
const path = require('node:path');

const source = fs.readFileSync(path.resolve(__dirname, '..',
  'usermods/WaveshareS3CompileCanary/WaveshareS3CompileCanary.cpp'), 'utf8');

it('offers scan and exact target arming without touching Conductor controls', () => {
  assert.match(source, /FieldScreen::Carrier/);
  assert.match(source, /void drawCarrier\(\)/);
  assert.match(source, /tubesS3ScanCarrierTargets\(\)/);
  assert.match(source, /tubesS3ReadCarrierTarget\(index, target\)/);
  assert.match(source, /tubesS3ArmCarrier\(target\.mac, target\.family, target\.variant, target\.release\)/);
  assert.doesNotMatch(source, /Previous/);
});
