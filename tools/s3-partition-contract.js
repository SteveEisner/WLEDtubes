'use strict';

const fs = require('node:fs');

const FLASH_SIZE = 16 * 1024 * 1024;
const ALIGNMENT = {app: 0x10000, data: 0x1000};
const REQUIRED = new Map([
	['nvs', ['data', 'nvs']],
	['otadata', ['data', 'ota']],
	['ota_0', ['app', 'ota_0']],
	['ota_1', ['app', 'ota_1']],
	['spiffs', ['data', 'spiffs']],
	['coredump', ['data', 'coredump']]
]);

function number(value) {
	if (typeof value !== 'string' || value === '') return undefined;
	const match = value.match(/^(?:(0[xX][0-9a-fA-F]+)|([0-9]+)([KM])?)$/);
	if (!match) return undefined;
	if (match[1]) return Number.parseInt(match[1], 16);
	const multiplier = match[3] ? {K: 1024, M: 1024 ** 2}[match[3].toUpperCase()] : 1;
	const parsed = Number(match[2]) * multiplier;
	return Number.isSafeInteger(parsed) ? parsed : undefined;
}

function align(value, alignment) {
	return Math.ceil(value / alignment) * alignment;
}

function parsePartitions(csv) {
	let nextOffset;
	const partitions = [];
	for (const [index, source] of csv.split(/\r?\n/).entries()) {
		const line = source.replace(/#.*/, '').trim();
		if (!line) continue;
		const fields = line.split(',').map(value => value.trim());
		if (fields.length < 5 || fields.length > 6) throw new Error(`malformed partition row ${index + 1}`);
		const [name, type, subtype, offsetText, sizeText] = fields;
		if (!name || !type || !subtype) throw new Error(`malformed partition row ${index + 1}: missing name, type, or subtype`);
		const size = number(sizeText);
		if (size === undefined || size <= 0) throw new Error(`invalid size in partition row ${index + 1}`);
		let offset = number(offsetText);
		const explicitOffset = offsetText !== '';
		if (explicitOffset && offset === undefined) throw new Error(`invalid offset in partition row ${index + 1}`);
		if (!explicitOffset) {
			if (nextOffset === undefined) throw new Error(`missing first partition offset in row ${index + 1}`);
			offset = align(nextOffset, ALIGNMENT[type] || 1);
		}
		partitions.push({name, type, subtype, offset, size, explicitOffset});
		nextOffset = offset + size;
	}
	return partitions;
}

function validatePartitions(partitions) {
	const errors = [];
	const names = new Set();
	const byName = new Map();
	for (const partition of partitions) {
		if (names.has(partition.name)) errors.push(`duplicate partition name: ${partition.name}`);
		else {
			names.add(partition.name);
			byName.set(partition.name, partition);
		}
		if (!Number.isSafeInteger(partition.offset) || !Number.isSafeInteger(partition.size)) errors.push(`invalid numeric offset or size: ${partition.name}`);
		const alignment = ALIGNMENT[partition.type];
		if (partition.explicitOffset && alignment && partition.offset % alignment !== 0)
			errors.push(`${partition.name} explicit offset must be 0x${alignment.toString(16)} aligned`);
	}
	for (const [name, [type, subtype]] of REQUIRED) {
		const partition = byName.get(name);
		if (!partition) errors.push(`missing required partition: ${name}`);
		else if (partition.type !== type || partition.subtype !== subtype)
			errors.push(`${name} must have type/subtype ${type}/${subtype}`);
	}
	const ordered = [...partitions].filter(partition => Number.isFinite(partition.offset) && Number.isFinite(partition.size)).sort((a, b) => a.offset - b.offset);
	for (let index = 1; index < ordered.length; index++) {
		if (ordered[index].offset < ordered[index - 1].offset + ordered[index - 1].size)
			errors.push(`partition overlap: ${ordered[index - 1].name} and ${ordered[index].name}`);
	}
	for (const partition of ordered) {
		if (partition.offset + partition.size > FLASH_SIZE) errors.push(`${partition.name} extends beyond 16MB flash`);
	}
	const ota0 = byName.get('ota_0');
	const ota1 = byName.get('ota_1');
	if (ota0 && ota1 && ota0.size !== ota1.size) errors.push('OTA slots must have equal size');
	return errors;
}

function validateBinarySize(binarySize, slotSize) {
	return binarySize * 100 <= slotSize * 85 ? [] :
		[`application size ${binarySize} leaves less than required 15% headroom in OTA slot ${slotSize}`];
}

if (require.main === module) {
	const [csvPath, binaryPath] = process.argv.slice(2);
	if (!csvPath) {
		console.error('usage: node tools/s3-partition-contract.js PARTITIONS.csv [firmware.bin]');
		process.exit(2);
	}
	let partitions;
	let errors;
	try {
		partitions = parsePartitions(fs.readFileSync(csvPath, 'utf8'));
		errors = validatePartitions(partitions);
	} catch (error) {
		errors = [error.message];
	}
	if (binaryPath && partitions) {
		const ota0 = partitions.find(partition => partition.name === 'ota_0');
		if (ota0) errors.push(...validateBinarySize(fs.statSync(binaryPath).size, ota0.size));
	}
	if (errors.length) {
		console.error(errors.join('\n'));
		process.exit(1);
	}
	console.log('S3 partition and application-size contract satisfied');
}

module.exports = {parsePartitions, validatePartitions, validateBinarySize};
