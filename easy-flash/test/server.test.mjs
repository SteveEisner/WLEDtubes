import assert from "node:assert/strict";
import test from "node:test";
import { server } from "../server.mjs";

test("serves the laptop USB prototype without an unverified local-build artifact API", async (context) => {
	await new Promise((resolve) => server.listen(0, "127.0.0.1", resolve));
	context.after(() => server.close());
	const { port } = server.address();
	const page = await fetch(`http://127.0.0.1:${port}/`);
	assert.equal(page.status, 200);
	assert.match(await page.text(), /LOCAL USB BETA/);
	assert.equal((await fetch(`http://127.0.0.1:${port}/api/artifact`)).status, 404);
});

test("rejects path traversal", async (context) => {
	if (!server.listening) await new Promise((resolve) => server.listen(0, "127.0.0.1", resolve));
	context.after(() => server.close());
	const { port } = server.address();
	const response = await fetch(`http://127.0.0.1:${port}/%2e%2e/package.json`);
	assert.notEqual(response.status, 200);
});

test("serves verified firmware manifest and transport-specific downloads", async (context) => {
	if (!server.listening) await new Promise((resolve) => server.listen(0, "127.0.0.1", resolve));
	context.after(() => server.close());
	const { port } = server.address();
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
	}
});

test("firmware API rejects misuse and traversal", async (context) => {
	if (!server.listening) await new Promise((resolve) => server.listen(0, "127.0.0.1", resolve));
	context.after(() => server.close());
	const { port } = server.address();
	for (const path of ["/api/firmware/previous-stable-control/serial", "/api/firmware/not-real/usb", "/api/firmware/%2e%2e/usb"]) {
		assert.notEqual((await fetch(`http://127.0.0.1:${port}${path}`)).status, 200);
	}
});
