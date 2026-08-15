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
  renderCpp,
  renderJs,
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

test('generated outputs are deterministic and committed clean', () => {
  const contract = loadContract();
  assert.equal(renderJs(contract), renderJs(clone(contract)));
  assert.equal(renderCpp(contract), renderCpp(clone(contract)));
  for (const [file, content] of generatedOutputs(contract))
    assert.equal(fs.readFileSync(file, 'utf8'), content, `${path.relative(root, file)} is stale`);
});

test('Dig2Go JS and compact C++ fields have parity', async () => {
  const moduleUrl = `${pathToFileURL(jsOutputPath).href}?test=${Date.now()}`;
  const {updateContract} = await import(moduleUrl);
  const target = updateContract.targets.find(item => item.id === 'quinled-dig2go');
  const header = fs.readFileSync(cppOutputPath, 'utf8');
  assert.match(header, /CANONICAL_TARGET_QUINLED_DIG2GO/);
  assert.match(header, new RegExp(`CanonicalTargetQuinledDig2go, 1, 1, ${target.flashSizeBytes}U, ${target.partition.otaSlots[0].offset}U, ${target.partition.otaSlots[0].sizeBytes}U`));
  for (const byte of target.partition.sha256.match(/../g)) assert.match(header, new RegExp(`0x${byte}`));
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
