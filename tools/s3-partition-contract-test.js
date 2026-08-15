const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const {execFileSync, spawnSync} = require('node:child_process');

const root = path.resolve(__dirname, '..');
const {parsePartitions, validatePartitions, validateBinarySize} = require('./s3-partition-contract');

const validCsv = `# Name, Type, SubType, Offset, Size, Flags\r\nnvs,data,nvs,0x9000,20K,\r\notadata,data,ota,,8K, # placed at 0xe000\r\nota_0,app,ota_0,,6M,\r\nota_1,app,ota_1,,0x600000,\r\nspiffs,data,spiffs,,4063232,\r\ncoredump,data,coredump,,64K,\r\n`;

function validation(csv = validCsv) {
	try { return validatePartitions(parsePartitions(csv)); }
	catch (error) { return [error.message]; }
}

function replaceRow(csv, name, row) {
	return csv.replace(new RegExp(`^${name},.*$`, 'm'), row);
}

test('partition parser supports comments, CRLF, decimal, hex, and integer K/M suffixes with aligned blank-offset placement', () => {
	const partitions = parsePartitions(validCsv);
	assert.deepEqual(partitions.map(({name, offset, size}) => ({name, offset, size})), [
		{name: 'nvs', offset: 0x9000, size: 20 * 1024},
		{name: 'otadata', offset: 0xe000, size: 8 * 1024},
		{name: 'ota_0', offset: 0x10000, size: 6 * 1024 * 1024},
		{name: 'ota_1', offset: 0x610000, size: 0x600000},
		{name: 'spiffs', offset: 0xc10000, size: 4063232},
		{name: 'coredump', offset: 0xff0000, size: 64 * 1024}
	]);
	assert.deepEqual(validatePartitions(partitions), []);
});

test('partition parser rejects missing first offset and malformed rows or numbers', () => {
	for (const csv of [
		validCsv.replace('0x9000', ''),
		validCsv.replace('20K', ''),
		validCsv.replace('20K', 'wat'),
		validCsv.replace('nvs,data,nvs,0x9000,20K,', 'nvs,data,nvs,0x9000'),
		validCsv.replace('nvs,data,nvs,0x9000,20K,', ',data,nvs,0x9000,20K,')
	]) assert.match(validation(csv).join('\n'), /invalid|missing|malformed/i);
});

test('partition validation rejects duplicate names and wrong required type/subtype pairs', () => {
	assert.match(validation(validCsv + 'nvs,data,nvs,0x200000,4K,\n').join('\n'), /duplicate partition name: nvs/);
	const pairs = {nvs: ['app','nvs'], otadata: ['data','nvs'], ota_0: ['data','ota_0'], ota_1: ['app','ota_0'], spiffs: ['data','coredump'], coredump: ['app','coredump']};
	for (const [name, [type, subtype]] of Object.entries(pairs)) {
		const csv = validCsv.replace(new RegExp(`^${name},[^,]+,[^,]+`, 'm'), `${name},${type},${subtype}`);
		assert.match(validation(csv).join('\n'), new RegExp(`${name}.*type.*subtype`, 'i'));
	}
});

test('partition validation rejects misaligned explicit app and data offsets', () => {
	assert.match(validation(replaceRow(validCsv, 'ota_0', 'ota_0,app,ota_0,0x11000,6M,')).join('\n'), /ota_0.*0x10000 aligned/i);
	assert.match(validation(replaceRow(validCsv, 'nvs', 'nvs,data,nvs,0x9001,20K,')).join('\n'), /nvs.*0x1000 aligned/i);
});

test('partition contract retains missing, overlap, end, equal OTA, and headroom checks', () => {
	assert.match(validation(validCsv.replace(/^otadata.*\r?\n/m, '')).join('\n'), /missing required partition: otadata/);
	assert.match(validation(replaceRow(validCsv, 'ota_1', 'ota_1,app,ota_1,0x600000,6M,')).join('\n'), /overlap/);
	assert.match(validation(replaceRow(validCsv, 'coredump', 'coredump,data,coredump,0xff0000,128K,')).join('\n'), /16MB flash/);
	assert.match(validation(replaceRow(validCsv, 'ota_1', 'ota_1,app,ota_1,,5M,')).join('\n'), /equal size/);
	assert.deepEqual(validateBinarySize(0x519999, 0x600000), []);
	assert.match(validateBinarySize(0x51999a, 0x600000).join('\n'), /15% headroom/);
});

test('partition CLI checks the actual firmware binary file size', () => {
	const dir = fs.mkdtempSync(path.join(os.tmpdir(), 's3-contract-'));
	const csv = path.join(dir, 'partitions.csv');
	const firmware = path.join(dir, 'firmware.bin');
	fs.writeFileSync(csv, validCsv);
	fs.writeFileSync(firmware, Buffer.alloc(0x51999a));
	const result = spawnSync(process.execPath, [path.join(__dirname, 's3-partition-contract.js'), csv, firmware], {encoding: 'utf8'});
	assert.equal(result.status, 1);
	assert.match(result.stderr, /15% headroom/);
});

test('Waveshare effective environment has unique artifact identity, one enabled USB CDC definition, and immutable pins', () => {
	const output = execFileSync('pio', ['project', 'config', '--json-output'], {
		cwd: root, encoding: 'utf8', env: {...process.env, PLATFORMIO_PROJECT_CONFIG: 'platformio_tubes.ini'}
	});
	const sections = new Map(JSON.parse(output).map(([name, options]) => [name, Object.fromEntries(options)]));
	const env = sections.get('env:waveshare_s3_tubes_remote');
	assert.ok(env, 'missing effective waveshare_s3_tubes_remote environment');
	let flags = [].concat(env.build_flags).join(' ');
	const unflags = [].concat(env.build_unflags).join(' ');
	for (const unflag of unflags.split(/\s+(?=-D)/).map(value => value.trim()).filter(Boolean))
		flags = flags.split(unflag).join('');
	assert.match(flags, /WLED_RELEASE_NAME=\\\"WAVESHARE_S3_TUBES_REMOTE\\\"/);
	assert.doesNotMatch(flags, /WLED_RELEASE_NAME=\\\"ESP32-S3_16MB_opi\\\"/);
	assert.equal((flags.match(/ARDUINO_USB_CDC_ON_BOOT=1/g) || []).length, 1);
	assert.doesNotMatch(flags, /ARDUINO_USB_CDC_ON_BOOT=0/);
	const dependencies = [].concat(env.lib_deps).join('\n');
	for (const sha of ['3cc08c4e9ab6d85807e49b657d73fae10871616e', 'eb462146d537a8103c0f680d2b4d78cde4fc8529']) assert.match(dependencies, new RegExp(sha));
	for (const library of ['Arduino_GFX', 'SensorLib'])
		assert.doesNotMatch(dependencies, new RegExp(`${library}\\.git#v\\d`));
});

test('runtime prototype documentation preserves hardware and authority gates', () => {
	const readme = fs.readFileSync(path.join(root, 'usermods/WaveshareS3CompileCanary/README.md'), 'utf8');
	assert.match(readme, /initializes the CO5300 AMOLED and CST9220 touch controller/i);
	assert.match(readme, /authority-neutral/i);
	assert.match(readme, /cannot claim or drive a physical\s+LED bus/i);
	assert.match(readme, /require proof on the exact board/i);
});
