import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import test, { after, before } from "node:test";
import { createEasyFlashServer } from "../server.mjs";

let port;
let server;

before(async () => {
	server = createEasyFlashServer();
	await new Promise((resolve) => server.listen(0, "127.0.0.1", resolve));
	port = server.address().port;
});

after(async () => {
	const closed = new Promise((resolve, reject) => server.close((error) => error ? reject(error) : resolve()));
	server.closeIdleConnections?.();
	server.closeAllConnections?.();
	await closed;
});

test("serves the laptop USB prototype without an unverified local-build artifact API", async () => {
	const page = await fetch(`http://127.0.0.1:${port}/`);
	assert.equal(page.status, 200);
	assert.match(await page.text(), /LOCAL USB BETA/);
	assert.equal((await fetch(`http://127.0.0.1:${port}/api/artifact`)).status, 404);
});

test("rejects path traversal", async () => {
	const response = await fetch(`http://127.0.0.1:${port}/%2e%2e/package.json`);
	assert.notEqual(response.status, 200);
});

test("serves verified firmware manifest and transport-specific downloads", async () => {
	const manifestResponse = await fetch(`http://127.0.0.1:${port}/api/firmware-manifest`);
	assert.equal(manifestResponse.status, 200);
	const manifest = await manifestResponse.json();
	assert.equal(manifest.variants.length, 1);
	assert.equal(manifest.variants.flatMap(({ artifacts }) => artifacts).length, 2);
	for (const transport of ["usb", "ota"]) {
		const response = await fetch(`http://127.0.0.1:${port}/api/firmware/previous-stable-control/${transport}`);
		assert.equal(response.status, 200);
		assert.equal(response.headers.get("content-type"), "application/octet-stream");
		assert.match(response.headers.get("content-disposition"), transport === "usb" ? /previous-stable-control-usb-merged\.bin/ : /previous-stable-control-http-ota-app\.bin/);
		const artifact = manifest.variants[0].artifacts.find((item) => item.transport === transport);
		assert.equal(createHash("sha256").update(Buffer.from(await response.arrayBuffer())).digest("hex"), artifact.sha256);
	}
});

test("firmware API rejects misuse and traversal", async () => {
	for (const path of ["/api/firmware/previous-stable-control/serial", "/api/firmware/not-real/usb", "/api/firmware/%2e%2e/usb"]) {
		assert.notEqual((await fetch(`http://127.0.0.1:${port}${path}`)).status, 200);
	}
});
