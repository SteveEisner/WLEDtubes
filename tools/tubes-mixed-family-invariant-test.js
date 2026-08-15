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

test('S3 authority mutation is denied at the shared controller boundary', () => {
	const controller = read('usermods/Tubes/controller.h');
	const execute = controller.match(/bool executeOperation\(const TubeOperation& operation\)([\s\S]*?)\n  void /);
	assert.ok(execute, 'could not isolate executeOperation');
	const capability = execute[1].match(/#ifdef TUBES_READ_ONLY_FIELD_SHELL([\s\S]*?)#endif/);
	assert.ok(capability, 'missing read-only field-shell capability gate');
	assert.match(capability[1], /RebootOperation[\s\S]*UpdateOperation[\s\S]*UpdateOfferOperation[\s\S]*SelectOperation[\s\S]*RoleOperation[\s\S]*return false;/);
	assert.ok(execute[1].indexOf('RoleOperation') < execute[1].indexOf('case RoleOperation:'), 'role denial must precede role effects');

	const json = controller.match(/void readJsonOperations\(JsonObject& root\)([\s\S]*?)\n  void /);
	const keyboard = controller.match(/void keyboard_command\(char \*command\)([\s\S]*?)\n  void /);
	const incoming = controller.match(/void onAction\(Action\* action\)([\s\S]*?)\n#define /);
	assert.ok(json && keyboard && incoming, 'could not isolate local and incoming command funnels');
	for (const [name, body] of [['JSON', json[1]], ['keyboard', keyboard[1]], ['incoming action', incoming[1]]]) {
		assert.match(body, /decodeOperation\([\s\S]*executeOperation\(operation\)/, `${name} path must use the shared capability boundary`);
	}

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
		"'r', RoleOperation, LocalScope", "'R', RoleOperation, SelectedScope",
	]) {
		assert.ok(controller.includes(`TUBE_COMMAND(${binding})`), `legacy binding changed: ${binding}`);
	}
	const role = controller.match(/case RoleOperation:([\s\S]*?)case CancelOverrideOperation:/);
	assert.ok(role, 'could not isolate legacy RoleOperation behavior');
	assert.match(role[1], /setRole\(ControllerRole\(argument\)\)[\s\S]*broadcastAction\('R', argument\)/);
});
