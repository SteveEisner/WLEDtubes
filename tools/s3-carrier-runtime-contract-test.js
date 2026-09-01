'use strict';

const assert = require('node:assert');
const { it } = require('node:test');
const fs = require('node:fs');
const path = require('node:path');

const repository = path.resolve(__dirname, '..');
const source = fs.readFileSync(path.join(repository,
  'usermods/WaveshareS3TubesRemote/S3FirmwareCarrier.cpp'), 'utf8');
const ui = fs.readFileSync(path.join(repository,
  'usermods/WaveshareS3TubesRemote/WaveshareS3TubesRemote.cpp'), 'utf8');
const protocol = fs.readFileSync(path.join(repository,
  'usermods/Tubes/fleet_update_protocol.h'), 'utf8');
const propagation = fs.readFileSync(path.join(repository,
  'usermods/Tubes/modern_propagation_lease.h'), 'utf8');

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

it('seeds proven Dig2Go propagation only from explicit exact-artifact input', () => {
  assert.match(ui, /DIG2GO \/ TAP TO SEED P2P/);
  assert.match(ui, /artifact\.peerPropagation/);
  assert.match(source, /artifact\.peerPropagation = true/);
  assert.match(source, /artifact\.peerPropagation = false/);
  assert.match(ui, /tubesS3SeedDig2GoPropagation/);
  assert.match(ui, /isSeedableDig2GoTarget/);
  assert.match(ui, /tubesS3ReadCarrierArtifact\(index, artifact\)/);
  assert.match(ui, /artifact\.release == target\.release/);
  assert.match(source, /target\.family == TubeHardwareDig2Go/);
  assert.match(source, /target\.variant == TubeVariantStandard/);
  assert.match(source, /target\.release == CARRIER_RELEASE/);
  assert.match(source, /carrierCatalogReady/);
  assert.match(source, /catalog\.select/);
  assert.match(source, /embeddedSize != S3_VAULT_DIG2GO_SIZE/);
  assert.match(source, /makeModernPropagationServeCommand/);
  assert.match(source, /tubesS3BroadcastFleetOffer/);
  assert.match(propagation, /command\.flags = FleetUpdatePropagate/);
  assert.match(propagation, /command\.serverPort = 0/);
  assert.match(propagation, /command\.targetDeviceId = targetDeviceId/);
  assert.doesNotMatch(source, /MODERN_PROPAGATION_LEASE_PATH|CURRENT_RELEASE_MARKER_PATH/);
});

it('keeps current Dig2Go visible for seeding without admitting current C3 targets', () => {
  assert.match(source, /report\.tubesVersion > CARRIER_RELEASE/);
  assert.match(source, /report\.tubesVersion == CARRIER_RELEASE[\s\S]*report\.hardwareFamily != TubeHardwareDig2Go/);
  assert.match(ui, /SEED REQUEST SENT/);
  assert.match(ui, /SEED REQUEST FAILED/);
  assert.doesNotMatch(ui, /propagationSeedSent = tubesS3SeedDig2GoPropagation[\s\S]{0,160}drawUpdateContent/);
});

it('keeps propagation seed commands out of the S3 receiver path', () => {
  const controller = fs.readFileSync(path.join(repository,
    'usermods/Tubes/controller.h'), 'utf8');
  assert.match(controller, /#ifdef TUBES_S3_FIELD_OS[\s\S]*if \(offer\.flags & FleetUpdatePropagate\)[\s\S]*return true;[\s\S]*updater\.startFleet\(offer\)/);
});
