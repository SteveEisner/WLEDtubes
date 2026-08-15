import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import {fileURLToPath, pathToFileURL} from 'node:url';

import {
  contractPath,
  cppOutputPath,
  generatedOutputs,
  jsOutputPath,
  loadContract,
  parsePartitionCsv,
  renderCpp,
  renderJs, validatePartitionRows,
  validateContract
} from './update-contract.mjs';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const clone = value => structuredClone(value);

test('canonical update contract and pinned files validate', () => {
  assert.deepEqual(validateContract(loadContract()), []);
});

test('duplicate and unknown identities fail closed', () => {
  const canonical = loadContract();
  const duplicateTarget = clone(canonical);
  duplicateTarget.targets.push(clone(duplicateTarget.targets[0]));
  assert.match(validateContract(duplicateTarget, {checkFiles: false}).join('\n'), /duplicate target id/);

  const duplicateArtifact = clone(canonical);
  duplicateArtifact.artifacts.push(clone(duplicateArtifact.artifacts[0]));
  assert.match(validateContract(duplicateArtifact, {checkFiles: false}).join('\n'), /duplicate artifact id/);

  const unknownTarget = clone(canonical);
  unknownTarget.artifacts[0].targetId = 'unknown-target';
  assert.match(validateContract(unknownTarget, {checkFiles: false}).join('\n'), /unknown targetId/);

  const unknownClass = clone(canonical);
  unknownClass.artifacts[0].releaseClass = 'Unknown';
  assert.match(validateContract(unknownClass, {checkFiles: false}).join('\n'), /unknown releaseClass/);

  const missingIdentity = clone(canonical);
  delete missingIdentity.artifacts[0].releaseIdentity;
  assert.match(validateContract(missingIdentity, {checkFiles: false}).join('\n'), /missing releaseIdentity/);
});

test('numeric discriminators are unique, ranged, and state values are frozen', () => {
  for (const key of ['releaseClasses', 'targets', 'artifacts', 'updateStates']) {
    const contract = loadContract();
    contract[key][1].cppValue = contract[key][0].cppValue;
    assert.match(validateContract(contract, {checkFiles: false}).join('\n'), /duplicate .* cppValue/);
    contract[key][1].cppValue = 256;
    assert.match(validateContract(contract, {checkFiles: false}).join('\n'), /invalid cppValue/);
  }
  const changedState = loadContract();
  changedState.updateStates.find(item => item.id === 'Complete').cppValue = 7;
  assert.match(validateContract(changedState, {checkFiles: false}).join('\n'), /must preserve cppValue 5/);
});

test('partition CSV parser and semantic geometry fail closed', () => {
  const contract = loadContract();
  for (const target of contract.targets) {
    const rows = parsePartitionCsv(fs.readFileSync(path.join(root, target.partition.csvPath), 'utf8'));
    const errors = [];
    validatePartitionRows(errors, target, rows);
    assert.deepEqual(errors, []);
    assert.equal(rows.filter(row => row.type === 'app' && row.subtype.startsWith('ota_')).length, 2);
  }
  assert.throws(() => parsePartitionCsv('bad,app,ota_0,0x10000,wat,'), /invalid offset or size/);
  assert.throws(() => parsePartitionCsv('bad,app,ota_0,0x10000,0x10000'), /6 columns/);
  assert.throws(() => parsePartitionCsv('bad,garbage,ota_0,0x10000,0x10000,'), /unsupported tokens/);
  assert.throws(() => parsePartitionCsv('bad,app,ota 0,0x10000,0x10000,'), /unsupported tokens/);
  assert.throws(() => parsePartitionCsv('bad,app,ota_0,0x10000,0x10000,encrypted'), /unsupported tokens/);
  const target = contract.targets[0];
  const rows = parsePartitionCsv(fs.readFileSync(path.join(root, target.partition.csvPath), 'utf8'));
  rows.find(row => row.subtype === 'ota_1').offset += 0x10000;
  const errors = [];
  validatePartitionRows(errors, target, rows);
  assert.match(errors.join('\n'), /geometry does not match|rows overlap/);
});

test('generated C++ names cannot collide', () => {
  const contract = loadContract();
  const duplicate = clone(contract.artifacts[0]);
  duplicate.id = 'dig2go_v14_usb_merged';
  duplicate.cppValue = 3;
  contract.artifacts.push(duplicate);
  assert.match(validateContract(contract, {checkFiles: false}).join('\n'), /duplicate generated artifact name/);
});

test('artifact vocabularies, combinations, identities, and bounds are closed', () => {
  const cases = [
    ['kind', 'mystery', /unsupported kind/],
    ['transport', 'wifi', /unsupported transport/],
    ['transport', 'usb', /application image must use OTA/],
    ['offset', 0, /does not fit a matching OTA slot/],
    ['lengthBytes', 99999999, /exceeds target flash/],
  ];
  for (const [field, value, pattern] of cases) {
    const contract = loadContract();
    Object.assign(contract.artifacts[1], {[field]: value});
    assert.match(validateContract(contract, {checkFiles: false}).join('\n'), pattern);
  }
  const merged = loadContract();
  merged.artifacts[0].components[0].sha256 = 'bad';
  assert.match(validateContract(merged, {checkFiles: false}).join('\n'), /invalid sha256/);
  const duplicate = loadContract();
  duplicate.artifacts[0].components[1].id = duplicate.artifacts[0].components[0].id;
  assert.match(validateContract(duplicate, {checkFiles: false}).join('\n'), /duplicate .* component id/);
});

test('generated outputs are deterministic and committed clean', () => {
  const contract = loadContract();
  assert.equal(renderJs(contract), renderJs(clone(contract)));
  assert.equal(renderCpp(contract), renderCpp(clone(contract)));
  for (const [file, content] of generatedOutputs(contract))
    assert.equal(fs.readFileSync(file, 'utf8'), content, `${path.relative(root, file)} is stale`);
});

test('JS and minimal C++ admission fields have parity for two slots and artifacts', async () => {
  const moduleUrl = `${pathToFileURL(jsOutputPath).href}?test=${Date.now()}`;
  const {updateContract} = await import(moduleUrl);
  const header = fs.readFileSync(cppOutputPath, 'utf8');
  for (const target of updateContract.targets) {
    assert.match(header, new RegExp(`${target.flashSizeBytes}U, 2,`));
    for (const slot of target.partition.otaSlots)
      assert.match(header, new RegExp(`\\{${slot.offset}U, ${slot.sizeBytes}U\\}`));
    for (const byte of target.partition.sha256.match(/../g)) assert.match(header, new RegExp(`0x${byte}`));
  }
  for (const artifact of updateContract.artifacts) {
    assert.match(header, new RegExp(`${artifact.offset}U, ${artifact.lengthBytes}U`));
    for (const byte of artifact.sha256.match(/../g)) assert.match(header, new RegExp(`0x${byte}`));
  }
  assert.match(header, /Minimal firmware admission projection/);
  const headerCode = header.split('\n').filter(line => !line.trim().startsWith('//')).join('\n');
  for (const omittedField of [
    'hardwareFamily', 'board', 'csvPath', 'hardwareAcceptance', 'tubesRelease',
    'wledBaseVersion', 'releaseIdentity', 'buildCommit', 'path', 'components'
  ])
    assert.doesNotMatch(headerCode, new RegExp(`\\b${omittedField}\\b`));
  for (const omitted of ['DIG2GO_TUBES', 'c6522acef3e954b14aad30d6f687cdb99bd1624e'])
    assert.doesNotMatch(header, new RegExp(omitted));
});

test('wire layout and command/action registry remain frozen', () => {
  const mesh = fs.readFileSync(path.join(root, 'usermods/Tubes/mesh_protocol.h'), 'utf8');
  const state = fs.readFileSync(path.join(root, 'usermods/Tubes/global_state.h'), 'utf8');
  const report = fs.readFileSync(path.join(root, 'usermods/Tubes/device_report_protocol.h'), 'utf8');
  const controller = fs.readFileSync(path.join(root, 'usermods/Tubes/controller.h'), 'utf8');
  assert.match(mesh, /static_assert\(sizeof\(NodeMessage\) == 84/);
  const commands = [...state.matchAll(/COMMAND_[A-Z]+\s*=\s*(0x[0-9A-F]+)/g)].map(match => match[1]);
  assert.deepEqual(commands, ['0x10', '0x20', '0x30', '0x40', '0x50', '0xE0', '0xF0']);
  assert.match(report, /DEVICE_REPORT_ACTION_KEY = 'z'/);
  assert.match(report, /static_assert\(sizeof\(DeviceReportMessage\) == 38/);
  assert.match(controller, /static_assert\(sizeof\(Action\) == 2/);
});

test('canonical source remains valid JSON', () => {
  assert.doesNotThrow(() => JSON.parse(fs.readFileSync(contractPath, 'utf8')));
});
