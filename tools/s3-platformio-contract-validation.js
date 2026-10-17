const test = require('node:test');
const assert = require('node:assert/strict');
const {execFileSync} = require('node:child_process');
const path = require('node:path');

const root = path.resolve(__dirname, '..');

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
	assert.match(flags, /TUBES_S3_FIELD_OS/);
	const dependencies = [].concat(env.lib_deps).join('\n');
	for (const sha of ['3cc08c4e9ab6d85807e49b657d73fae10871616e', 'eb462146d537a8103c0f680d2b4d78cde4fc8529', 'f142ed8356333357fa9cb0873392112907e8a578']) assert.match(dependencies, new RegExp(sha));
	for (const library of ['Arduino_GFX', 'SensorLib', 'XPowersLib'])
		assert.doesNotMatch(dependencies, new RegExp(`${library}\\.git#v\\d`));
	assert.doesNotMatch(flags, /TUBES_S3_FIRMWARE_CARRIER/);
	assert.match(flags, /TUBES_HARDWARE_FAMILY=TubeHardwareMatrixM1/);
	assert.ok(!env['board_build.embed_files'], 'base S3 unexpectedly requires generated vault files');
	const carrier = sections.get('env:waveshare_s3_tubes_carrier');
	assert.ok(carrier, 'missing effective carrier environment');
	const carrierFlags = [].concat(carrier.build_flags).join(' ');
	assert.match(carrierFlags, /TUBES_S3_FIRMWARE_CARRIER/);
	assert.match(carrierFlags, /WLED_RELEASE_NAME=\\"WAVESHARE_S3_TUBES_CARRIER\\"/);
	assert.match([].concat(carrier['board_build.embed_files']).join(' '), /esp32_quinled_dig2go_tubes_p2p\.bin/);
	assert.match([].concat(carrier['board_build.embed_files']).join(' '), /esp32-c3-athom_tubes\.bin/);
});
