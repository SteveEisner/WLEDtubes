import crypto from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
export const contractPath = path.join(root, 'contracts/update/update-contract.json');
export const jsOutputPath = path.join(root, 'contracts/update/generated/update-contract.generated.mjs');
export const cppOutputPath = path.join(root, 'usermods/Tubes/generated/update_contract_generated.h');

const hex64 = /^[0-9a-f]{64}$/;
const requiredArtifactFields = [
  'id', 'targetId', 'releaseClass', 'tubesRelease', 'wledBaseVersion',
  'releaseIdentity', 'buildCommit', 'kind', 'transport', 'path', 'offset',
  'lengthBytes', 'sha256'
];

function fail(errors, condition, message) {
  if (!condition) errors.push(message);
}

function unique(errors, records, label) {
  const seen = new Set();
  for (const record of records) {
    fail(errors, record && typeof record.id === 'string' && record.id.length > 0, `${label} has invalid id`);
    if (!record || typeof record.id !== 'string') continue;
    fail(errors, !seen.has(record.id), `duplicate ${label} id: ${record.id}`);
    seen.add(record.id);
  }
}

export function loadContract(file = contractPath) {
  return JSON.parse(fs.readFileSync(file, 'utf8'));
}

export function validateContract(contract, {checkFiles = true} = {}) {
  const errors = [];
  fail(errors, contract?.schemaVersion === 1, 'schemaVersion must be 1');
  for (const key of ['releaseClasses', 'targets', 'artifacts', 'updateStates'])
    fail(errors, Array.isArray(contract?.[key]), `${key} must be an array`);
  if (errors.length) return errors;

  unique(errors, contract.releaseClasses, 'release class');
  unique(errors, contract.targets, 'target');
  unique(errors, contract.artifacts, 'artifact');
  unique(errors, contract.updateStates, 'update state');
  const classes = new Set(contract.releaseClasses.map(item => item.id));
  for (const expected of ['Legacy', 'Current', 'Next', 'Unknown'])
    fail(errors, classes.has(expected), `missing release class: ${expected}`);
  fail(errors, contract.releaseClasses.find(item => item.id === 'Unknown')?.cppValue === 0,
    'Unknown release class must be fail-closed value 0');
  const targets = new Map(contract.targets.map(item => [item.id, item]));

  for (const target of contract.targets) {
    for (const key of ['id', 'hardwareFamily', 'chipFamily', 'board', 'flashMode'])
      fail(errors, typeof target[key] === 'string' && target[key].length > 0, `${target.id || 'target'} missing ${key}`);
    fail(errors, Number.isSafeInteger(target.cppValue) && target.cppValue > 0, `${target.id} has invalid cppValue`);
    fail(errors, Number.isSafeInteger(target.flashSizeBytes) && target.flashSizeBytes > 0, `${target.id} has invalid flashSizeBytes`);
    if (!target.partition) continue;
    fail(errors, hex64.test(target.partition.sha256), `${target.id} has invalid partition sha256`);
    fail(errors, Array.isArray(target.partition.otaSlots) && target.partition.otaSlots.length > 0, `${target.id} has no OTA slots`);
    for (const slot of target.partition.otaSlots || []) {
      fail(errors, Number.isSafeInteger(slot.offset) && slot.offset >= 0, `${target.id} has invalid OTA offset`);
      fail(errors, Number.isSafeInteger(slot.sizeBytes) && slot.sizeBytes > 0, `${target.id} has invalid OTA size`);
      fail(errors, slot.offset + slot.sizeBytes <= target.flashSizeBytes, `${target.id} OTA slot exceeds flash`);
    }
    if (checkFiles && typeof target.partition.csvPath === 'string') {
      const file = path.join(root, target.partition.csvPath);
      fail(errors, fs.existsSync(file), `${target.id} partition CSV missing: ${target.partition.csvPath}`);
      if (fs.existsSync(file)) fail(errors, sha256(fs.readFileSync(file)) === target.partition.sha256, `${target.id} partition CSV hash mismatch`);
    }
  }

  for (const artifact of contract.artifacts) {
    for (const key of requiredArtifactFields)
      fail(errors, artifact[key] !== undefined && artifact[key] !== '', `${artifact.id || 'artifact'} missing ${key}`);
    fail(errors, targets.has(artifact.targetId), `${artifact.id} has unknown targetId: ${artifact.targetId}`);
    fail(errors, classes.has(artifact.releaseClass) && artifact.releaseClass !== 'Unknown', `${artifact.id} has unknown releaseClass`);
    fail(errors, hex64.test(artifact.sha256), `${artifact.id} has invalid sha256`);
    fail(errors, Number.isSafeInteger(artifact.offset) && artifact.offset >= 0, `${artifact.id} has invalid offset`);
    fail(errors, Number.isSafeInteger(artifact.lengthBytes) && artifact.lengthBytes > 0, `${artifact.id} has invalid lengthBytes`);
    const target = targets.get(artifact.targetId);
    if (artifact.transport === 'ota' && target?.partition) {
      const fits = target.partition.otaSlots.some(slot => artifact.offset === slot.offset && artifact.lengthBytes <= slot.sizeBytes);
      fail(errors, fits, `${artifact.id} does not fit a matching OTA slot`);
    }
    if (!checkFiles || typeof artifact.path !== 'string') continue;
    const file = path.join(root, artifact.path);
    fail(errors, fs.existsSync(file), `${artifact.id} file missing: ${artifact.path}`);
    if (!fs.existsSync(file)) continue;
    const bytes = fs.readFileSync(file);
    fail(errors, bytes.length === artifact.lengthBytes, `${artifact.id} length mismatch`);
    fail(errors, sha256(bytes) === artifact.sha256, `${artifact.id} hash mismatch`);
    const components = [...(artifact.components || [])].sort((a, b) => a.offset - b.offset);
    for (let index = 0; index < components.length; index++) {
      const component = components[index];
      fail(errors, component.offset >= artifact.offset && component.offset + component.lengthBytes <= artifact.offset + bytes.length,
        `${artifact.id}/${component.id} exceeds merged image`);
      if (index > 0) fail(errors, components[index - 1].offset + components[index - 1].lengthBytes <= component.offset,
        `${artifact.id} components overlap`);
      const start = component.offset - artifact.offset;
      fail(errors, sha256(bytes.subarray(start, start + component.lengthBytes)) === component.sha256,
        `${artifact.id}/${component.id} hash mismatch`);
    }
  }
  return errors;
}

function sha256(bytes) {
  return crypto.createHash('sha256').update(bytes).digest('hex');
}

function stable(value) {
  if (Array.isArray(value)) return value.map(stable);
  if (value && typeof value === 'object') return Object.fromEntries(Object.keys(value).sort().map(key => [key, stable(value[key])]));
  return value;
}

export function jsProjection(contract) {
  return stable({
    schemaVersion: contract.schemaVersion,
    releaseClasses: contract.releaseClasses,
    targets: [...contract.targets].sort((a, b) => a.id.localeCompare(b.id)),
    artifacts: [...contract.artifacts].sort((a, b) => a.id.localeCompare(b.id)),
    updateStates: contract.updateStates,
    receiptVocabulary: contract.receiptVocabulary
  });
}

export function renderJs(contract) {
  return `// GENERATED FILE. DO NOT EDIT.\n// Source: contracts/update/update-contract.json (schema ${contract.schemaVersion})\nexport const updateContract = Object.freeze(${JSON.stringify(jsProjection(contract), null, 2)});\n`;
}

function cppName(value) {
  return value.replace(/(^|[^A-Za-z0-9]+)([A-Za-z0-9])/g, (_, _sep, letter) => letter.toUpperCase()).replace(/[^A-Za-z0-9]/g, '');
}

function hashBytes(hash) {
  return hash.match(/../g).map(value => `0x${value}`).join(', ');
}

export function renderCpp(contract) {
  const classes = [...contract.releaseClasses].sort((a, b) => a.cppValue - b.cppValue)
    .map(item => `  CanonicalRelease${cppName(item.id)} = ${item.cppValue},`).join('\n');
  const targetEnums = [...contract.targets].sort((a, b) => a.cppValue - b.cppValue)
    .map(item => `  CanonicalTarget${cppName(item.id)} = ${item.cppValue},`).join('\n');
  const states = [...contract.updateStates].sort((a, b) => a.cppValue - b.cppValue)
    .map(item => `  CanonicalUpdate${cppName(item.id)} = ${item.cppValue},`).join('\n');
  const targets = [...contract.targets].sort((a, b) => a.cppValue - b.cppValue).map(item => {
    const partition = item.partition;
    const slot = partition?.otaSlots?.[0];
    const chip = item.chipFamily === 'ESP32' ? 1 : item.chipFamily === 'ESP32-S3' ? 3 : 0;
    const mode = item.flashMode === 'dio' ? 1 : item.flashMode === 'qio' ? 2 : item.flashMode === 'opi' ? 3 : 0;
    const hash = partition?.sha256 || '0'.repeat(64);
    return `static constexpr CanonicalTargetRecord CANONICAL_TARGET_${item.id.replace(/[^A-Za-z0-9]/g, '_').toUpperCase()} = {\n  CanonicalTarget${cppName(item.id)}, ${chip}, ${mode}, ${item.flashSizeBytes}U, ${slot?.offset || 0}U, ${slot?.sizeBytes || 0}U,\n  {${hashBytes(hash)}}\n};`;
  }).join('\n\n');
  return `#pragma once\n\n#include <stdint.h>\n\n// GENERATED FILE. DO NOT EDIT.\n// Source: contracts/update/update-contract.json (schema ${contract.schemaVersion})\n// AI: below section was generated by an AI\nenum CanonicalReleaseClass : uint8_t {\n${classes}\n};\n\nenum CanonicalTargetId : uint8_t {\n  CanonicalTargetUnknown = 0,\n${targetEnums}\n};\n\nenum CanonicalUpdateState : uint8_t {\n${states}\n};\n\nstruct CanonicalTargetRecord {\n  CanonicalTargetId targetId;\n  uint8_t chipFamily;\n  uint8_t flashMode;\n  uint32_t flashSizeBytes;\n  uint32_t otaSlotOffset;\n  uint32_t otaSlotSizeBytes;\n  uint8_t partitionTableSha256[32];\n};\n\n${targets}\n// AI: end\n`;
}

export function generatedOutputs(contract) {
  return new Map([[jsOutputPath, renderJs(contract)], [cppOutputPath, renderCpp(contract)]]);
}
