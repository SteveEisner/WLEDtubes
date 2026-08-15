const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

const root = path.resolve(__dirname, '..');
const read = file => fs.readFileSync(path.join(root, file), 'utf8');

function environment(ini, name) {
	const escaped = name.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
	const match = ini.match(new RegExp(`\\[env:${escaped}\\]([\\s\\S]*?)(?=\\n\\[|$)`));
	assert.ok(match, `missing ${name} environment`);
	return match[1];
}

test('S3 read-only and logical-output flags do not leak into Dig2Go', () => {
	const ini = read('platformio_tubes.ini');
	const s3 = environment(ini, 'waveshare_s3_tubes_remote');
	const dig2go = environment(ini, 'esp32_quinled_dig2go_tubes');

	assert.match(s3, /TUBES_HARDWARE_FAMILY=TubeHardwareWaveshareS3/);
	assert.match(s3, /-D TUBES_READ_ONLY_FIELD_SHELL/);
	assert.match(s3, /-D TUBES_(?:NULL|LOGICAL)_OUTPUT/);
	assert.doesNotMatch(s3, /(?:LEDPIN|DATA_PINS|TYPE_NET_)/);

	assert.match(dig2go, /TUBES_HARDWARE_FAMILY=TubeHardwareDig2Go/);
	assert.doesNotMatch(dig2go, /WAVESHARE|TUBES_READ_ONLY_FIELD_SHELL|TUBES_(?:NULL|LOGICAL)_OUTPUT/);
});

test('S3 update and reboot denial is enforced at the controller boundary', () => {
	const controller = read('usermods/Tubes/controller.h');
	const execute = controller.match(/bool executeOperation\(const TubeOperation& operation\)([\s\S]*?)\n  void /);
	assert.ok(execute, 'could not isolate executeOperation');
	assert.match(execute[1], /#ifdef TUBES_READ_ONLY_FIELD_SHELL[\s\S]*RebootOperation[\s\S]*UpdateOperation[\s\S]*UpdateOfferOperation[\s\S]*return false;/);

	const shell = read('usermods/WaveshareS3CompileCanary/WaveshareS3CompileCanary.cpp');
	assert.doesNotMatch(shell, /Update\.begin|Update\.write|esp_ota_begin|ESP\.restart/);
});

test('legacy packet IDs, action layout, and operation bindings remain unchanged', () => {
	const state = read('usermods/Tubes/global_state.h');
	for (const [name, value] of Object.entries({
		COMMAND_OPTIONS: '0x10', COMMAND_STATE: '0x20', COMMAND_ACTION: '0x30',
		COMMAND_INFO: '0x40', COMMAND_BEATS: '0x50', COMMAND_UPGRADE: '0xE0',
		COMMAND_RESET: '0xF0'
	})) assert.match(state, new RegExp(`CommandId\\s+${name}\\s*=\\s*${value}`));

	const mesh = read('usermods/Tubes/mesh_protocol.h');
	assert.match(mesh, /MESSAGE_DATA_SIZE\s+64/);
	assert.match(mesh, /static_assert\(sizeof\(NodeMessage\) == 84/);

	const controller = read('usermods/Tubes/controller.h');
	assert.match(controller, /typedef struct \{\s*char key;\s*uint8_t arg;\s*\} Action;/);
	for (const binding of [
		"'u', UpdateOperation, LocalScope", "'U', UpdateOperation, SelectedScope",
		"'V', UpdateOfferOperation, MeshScope", "'X', RebootOperation, SelectedScope",
	]) {
		assert.ok(controller.includes(`TUBE_COMMAND(${binding})`), `legacy binding changed: ${binding}`);
	}
});
