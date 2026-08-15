import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { cp, mkdtemp, readFile, rm, stat, unlink, writeFile } from "node:fs/promises";
import { join } from "node:path";
import { tmpdir } from "node:os";
import test from "node:test";

import { loadFirmwareManifest, resolveFirmwareArtifact } from "../firmware-manifest.mjs";

const variantId = "previous-stable-control";

test("contains only canonical Dig2Go v14 hardware firmware", async () => {
	const manifest = await loadFirmwareManifest();
	assert.equal(manifest.schemaVersion, 2);
	assert.equal(manifest.variants.length, 1);
	const variant = manifest.variants[0];
	assert.equal(variant.id, variantId);
	assert.equal(variant.status, "stable");
	assert.equal(variant.source.commit, "c6522acef3e954b14aad30d6f687cdb99bd1624e");
	assert.equal(variant.source.clean, true);
	assert.equal(variant.target.environment, "esp32_quinled_dig2go_tubes");
	assert.equal(variant.target.hardwareFamily, "quinled-dig2go");
	assert.equal(variant.target.chip, "ESP32");
	assert.equal(variant.target.flashSizeBytes, 4194304);
	assert.equal(variant.target.flashMode, "dio");
	assert.equal(variant.partition.otaSlot.offset, 0x10000);
	assert.equal(variant.artifacts.length, 2);
	for (const artifact of variant.artifacts) {
		const resolved = await resolveFirmwareArtifact(variant.id, artifact.transport);
		const bytes = await readFile(resolved.absolutePath);
		assert.equal((await stat(resolved.absolutePath)).size, artifact.sizeBytes);
		assert.equal(createHash("sha256").update(bytes).digest("hex"), artifact.sha256);
	}
	const usb = variant.artifacts.find(({ transport }) => transport === "usb");
	const ota = variant.artifacts.find(({ transport }) => transport === "ota");
	assert.equal(usb.kind, "complete-merged-image");
	assert.equal(usb.offset, 0);
	assert.deepEqual(usb.components.map(({ offset }) => offset), [0x1000, 0x8000, 0xe000, 0x10000]);
	assert.equal(ota.kind, "application-image");
	assert.equal(ota.offset, 0x10000);
	assert.ok(ota.sizeBytes <= variant.partition.otaSlot.sizeBytes);
});

test("resolver rejects target mismatch, transport misuse, traversal, missing, and tampered artifacts", async () => {
	await assert.rejects(() => resolveFirmwareArtifact(variantId, "serial"), /transport/i);
	await assert.rejects(() => resolveFirmwareArtifact("../package.json", "usb"), /variant/i);
	await assert.rejects(() => resolveFirmwareArtifact("missing", "ota"), /variant/i);
	await assert.rejects(() => resolveFirmwareArtifact(variantId, "ota", undefined, {
		hardwareFamily: "waveshare-esp32-s3-touch-amoled-2.16",
		chip: "ESP32-S3",
		flashSizeBytes: 16777216
	}), /target contract/i);

	const fixtureRoot = await mkdtemp(join(tmpdir(), "easy-flash-integrity-"));
	try {
		await cp(new URL("../artifacts", import.meta.url), join(fixtureRoot, "artifacts"), { recursive: true });
		const fixtureManifest = JSON.parse(await readFile(new URL("../firmware-manifest.json", import.meta.url), "utf8"));
		await writeFile(join(fixtureRoot, "firmware-manifest.json"), JSON.stringify(fixtureManifest));
		const artifactPath = join(fixtureRoot, fixtureManifest.variants[0].artifacts[0].path);
		const original = await readFile(artifactPath);
		await writeFile(artifactPath, Buffer.concat([original.subarray(0, -1), Buffer.from([original.at(-1) ^ 0xff])]));
		await assert.rejects(() => resolveFirmwareArtifact(variantId, "usb", join(fixtureRoot, "firmware-manifest.json")), /integrity/i);
		await unlink(artifactPath);
		await assert.rejects(() => resolveFirmwareArtifact(variantId, "usb", join(fixtureRoot, "firmware-manifest.json")), /ENOENT/);
	} finally {
		await rm(fixtureRoot, { recursive: true, force: true });
	}
});
