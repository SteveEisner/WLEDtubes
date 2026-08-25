'use strict';

const assert = require('node:assert');
const { it } = require('node:test');
const fs = require('node:fs');
const path = require('node:path');

const repository = path.resolve(__dirname, '..');
const source = fs.readFileSync(path.join(repository,
  'usermods/WaveshareS3TubesRemote/S3FirmwareCarrier.cpp'), 'utf8');

it('serves the exact fleet pull endpoint with integrity headers', () => {
  assert.match(source, /"\/tubes\/firmware\.bin"/);
  assert.match(source, /"nonce", "release", "family", "variant", "mac"/);
  assert.match(source, /request->args\(\) != count/);
  assert.match(source, /"application\/octet-stream"/);
  assert.match(source, /"x-MD5"/);
  assert.match(source, /"Cache-Control", "no-store"/);
});

it('completes a body only after the response reaches fully acknowledged end state', () => {
  assert.match(source, /class AcknowledgedProgmemResponse/);
  assert.match(source, /AsyncProgmemResponse::_ack/);
  assert.match(source, /_state == RESPONSE_END && _ackedLength >= _writtenLength/);
  assert.match(source, /policy\.bodyCompleted\(pendingResponseMac, now\)/);
  assert.match(source, /responseFinished\(false, mac_\)/);
  const callback = source.match(/void responseFinished[\s\S]*?\n}/)[0];
  assert.doesNotMatch(callback, /WiFi\.|softAP|policy\./);
  assert.match(source, /TCP_DRAIN_GRACE_MS/);
});

it('keeps a bounded fresh target ledger for laptop-free selection', () => {
  assert.match(source, /TARGET_CAPACITY = 7/);
  assert.match(source, /TARGET_MAX_AGE_MS = 60000/);
  assert.match(source, /rememberTarget\(report\)/);
  assert.match(source, /tubesS3ScanCarrierTargets/);
  assert.match(source, /tubesS3ReadCarrierTarget/);
});

it('arms explicitly and broadcasts Steve fleet offer vocabulary', () => {
  assert.match(source, /"\/tubes\/carrier\/arm"/);
  assert.match(source, /S3VaultOfferFactory::make/);
  assert.match(source, /tubesS3BroadcastFleetOffer/);
  assert.match(source, /tubesS3RequestDeviceReport/);
  assert.match(source, /report\.nonce != probeNonce/);
  assert.match(source, /report\.hardwareFamily != probeFamily/);
  assert.match(source, /report\.firmwareVariant != probeVariant/);
  assert.match(source, /report\.tubesVersion != probeCurrentRelease/);
  assert.match(source, /WiFi\.softAP\(CARRIER_SSID, CARRIER_PASSWORD, channel, false, 1\)/);
  assert.match(source, /WiFi\.softAPIP\(\)/);
  assert.match(source, /WiFi\.softAPdisconnect\(true\)/);
});
