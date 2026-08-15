import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

import { getHardwareArtifacts, makeFirmwareOperationReceipt } from "../firmware-ui.mjs";
import { loadFirmwareManifest } from "../firmware-manifest.mjs";

test("test bench is collapsed and explains laptop-local USB flashing", async () => {
	const html = await readFile(new URL("../index.html", import.meta.url), "utf8");
	assert.match(html, /<details[^>]+id="firmwareTestBench"/);
	assert.match(html, /Advanced — Hardware Firmware/);
	assert.match(html, /Install on the ESP connected to this laptop/);
	assert.match(html, /secure local Web Serial support/);
});

test("portal opens with the participant product promise", async () => {
	const html = await readFile(new URL("../index.html", import.meta.url), "utf8");
	assert.match(html, /Buy\. Build\.<br><em>Rave\.<\/em>/);
	assert.match(html, /no developer tools required/);
});

test("catalog contains only the canonical Dig2Go hardware artifact", async () => {
	const manifest = await loadFirmwareManifest();
	const artifacts = getHardwareArtifacts(manifest);
	assert.deepEqual(artifacts.map((variant) => variant.id), ["previous-stable-control"]);
});

test("firmware receipt is prepared-not-written and independent of participant plan", async () => {
	const manifest = await loadFirmwareManifest();
	const variant = manifest.variants[0];
	for (const transport of ["usb", "ota"]) {
		const artifact = variant.artifacts.find((item) => item.transport === transport);
		const receipt = makeFirmwareOperationReceipt(variant, artifact, new Date("2026-08-08T12:00:00Z"));
		assert.deepEqual(Object.keys(receipt).sort(), ["artifactSha256", "createdAt", "partition", "result", "sourceCommit", "target", "transport", "variantId"].sort());
		assert.equal(receipt.variantId, variant.id);
		assert.equal(receipt.transport, transport);
		assert.equal(receipt.artifactSha256, artifact.sha256);
		assert.equal(receipt.result, "prepared-not-written");
	}
});

test("participant step sequence remains Controller to Lights to Power to Review", async () => {
	const html = await readFile(new URL("../index.html", import.meta.url), "utf8");
	assert.match(html, /1 <span>Controller<\/span>.*2 <span>Lights<\/span>.*3 <span>Power<\/span>.*4 <span>Review<\/span>/s);
});

test("client offers direct laptop flash while preserving downloads", async () => {
	const source = await readFile(new URL("../app.mjs", import.meta.url), "utf8");
	assert.match(source, /getHardwareArtifacts\(manifest\)/);
	assert.match(source, /Download complete USB image/);
	assert.match(source, /Download HTTP OTA app image/);
	assert.match(source, /Flash from this laptop/);
	assert.match(source, /Participant Controller → Lights → Power → Review remains fully usable/);
});

test("Easy Flash presents firmware as hardware and software profiles as deferred runtime state", async () => {
	const html = await readFile(new URL("../index.html", import.meta.url), "utf8");
	assert.match(html, /Hardware firmware/);
	assert.match(html, /Software profiles are waiting for Steve’s split-packet contract/);
	assert.doesNotMatch(html, /Parked behavior-firmware experiments/);
	assert.doesNotMatch(html, /SEVEN HARDWARE TARGETS/);
});

test("local flash checks chip and image before writing", async () => {
	const source = await readFile(new URL("../local-flash.mjs", import.meta.url), "utf8");
	const chipCheck = source.indexOf("Wrong chip:");
	const hashCheck = source.indexOf("Firmware image hash mismatch");
	const write = source.indexOf("await loader.writeFlash");
	assert.ok(chipCheck > 0 && chipCheck < write);
	assert.ok(hashCheck > 0 && hashCheck < write);
	assert.match(source, /baudrate: 115200/);
	assert.match(source, /compress: false/);
	assert.match(source, /data: image\.slice\(/);
	assert.doesNotMatch(source, /bytesToBinaryString/);
	assert.match(source, /navigator\.serial\.requestPort/);
});
