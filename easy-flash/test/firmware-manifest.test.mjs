import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { cp, mkdir, mkdtemp, readFile, realpath, rm, symlink, unlink, writeFile } from "node:fs/promises";
import path, { join } from "node:path";
import { win32 } from "node:path";
import { tmpdir } from "node:os";
import test from "node:test";

import { isPathInside, loadFirmwareManifest, pathsHaveSameIdentity, resolveFirmwareArtifact } from "../firmware-manifest.mjs";

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
		assert.equal(resolved.bytes.length, artifact.sizeBytes);
		assert.equal(createHash("sha256").update(resolved.bytes).digest("hex"), artifact.sha256);
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

async function fixture() {
	const fixtureRoot = await mkdtemp(join(tmpdir(), "easy-flash-manifest-"));
	await cp(new URL("../artifacts", import.meta.url), join(fixtureRoot, "artifacts"), { recursive: true });
	const manifest = JSON.parse(await readFile(new URL("../firmware-manifest.json", import.meta.url), "utf8"));
	const manifestPath = join(fixtureRoot, "firmware-manifest.json");
	await writeFile(manifestPath, JSON.stringify(manifest));
	return { fixtureRoot, manifest, manifestPath };
}

test("manifest validation rejects duplicate registry keys", async () => {
	for (const mutate of [
		(manifest) => manifest.variants.push(structuredClone(manifest.variants[0])),
		(manifest) => { manifest.variants[0].artifacts[0].id = "same-artifact"; manifest.variants[0].artifacts[1].id = "same-artifact"; },
		(manifest) => { manifest.variants[0].artifacts[1].transport = manifest.variants[0].artifacts[0].transport; },
	]) {
		const { fixtureRoot, manifest, manifestPath } = await fixture();
		try {
			mutate(manifest);
			await writeFile(manifestPath, JSON.stringify(manifest));
			await assert.rejects(() => loadFirmwareManifest(manifestPath), /duplicate/i);
		} finally {
			await rm(fixtureRoot, { recursive: true, force: true });
		}
	}
});

test("containment applies Windows separators and drive boundaries", () => {
	assert.equal(isPathInside("C:\\bundle\\artifacts", "C:\\bundle\\artifacts\\variant\\firmware.bin", win32), true);
	assert.equal(isPathInside("C:\\bundle\\artifacts", "C:\\bundle\\artifacts-evil\\firmware.bin", win32), false);
	assert.equal(isPathInside("C:\\bundle\\artifacts", "C:\\bundle\\outside.bin", win32), false);
	assert.equal(isPathInside("C:\\bundle\\artifacts", "D:\\bundle\\artifacts\\firmware.bin", win32), false);
});

test("root identity normalization follows native macOS and Windows path rules", () => {
	assert.equal(pathsHaveSameIdentity("/bundle/artifacts", "/bundle/artifacts", path.posix, false), true);
	assert.equal(pathsHaveSameIdentity("/bundle/artifacts", "/bundle/ARTIFACTS", path.posix, false), false);
	assert.equal(pathsHaveSameIdentity("C:\\Bundle\\artifacts", "c:/bundle/artifacts\\", win32, true), true);
	assert.equal(pathsHaveSameIdentity("C:\\Bundle\\artifacts", "D:\\Bundle\\artifacts", win32, true), false);
});

test("resolver rejects canonical root mismatch and replacement deterministically", async () => {
	for (const replaceOnSecondRootRead of [false, true]) {
		const { fixtureRoot, manifestPath } = await fixture();
		try {
			const rootPath = join(fixtureRoot, "artifacts");
			let rootReads = 0;
			const canonicalize = async (value) => {
				if (value === rootPath && ++rootReads > (replaceOnSecondRootRead ? 1 : 0)) return join(fixtureRoot, "attacker-artifacts");
				return realpath(value);
			};
			await assert.rejects(() => resolveFirmwareArtifact(variantId, "usb", manifestPath, null, { canonicalize }), /artifact root.*changed|real directory/i);
			assert.equal(rootReads, replaceOnSecondRootRead ? 2 : 1);
		} finally {
			await rm(fixtureRoot, { recursive: true, force: true });
		}
	}
});

test("resolver rejects traversal, separator-prefix collisions, and invalid artifact roots", async () => {
	for (const artifactPath of ["../outside.bin", "artifacts-evil/outside.bin"]) {
		const { fixtureRoot, manifest, manifestPath } = await fixture();
		try {
			const outsidePath = join(fixtureRoot, artifactPath);
			await mkdir(join(outsidePath, ".."), { recursive: true });
			await cp(join(fixtureRoot, manifest.variants[0].artifacts[0].path), outsidePath);
			manifest.variants[0].artifacts[0].path = artifactPath;
			await writeFile(manifestPath, JSON.stringify(manifest));
			await assert.rejects(() => resolveFirmwareArtifact(variantId, "usb", manifestPath), /escapes bundle/i);
		} finally {
			await rm(fixtureRoot, { recursive: true, force: true });
		}
	}

	const missing = await fixture();
	try {
		await rm(join(missing.fixtureRoot, "artifacts"), { recursive: true });
		await assert.rejects(() => resolveFirmwareArtifact(variantId, "usb", missing.manifestPath), /artifact root/i);
	} finally {
		await rm(missing.fixtureRoot, { recursive: true, force: true });
	}
});

test("resolver rejects symlink escapes and symlinked artifact roots", async (context) => {
	const escaped = await fixture();
	try {
		const outsidePath = join(escaped.fixtureRoot, "outside.bin");
		await cp(join(escaped.fixtureRoot, escaped.manifest.variants[0].artifacts[0].path), outsidePath);
		const linkPath = join(escaped.fixtureRoot, "artifacts", "escape.bin");
		try { await symlink(outsidePath, linkPath, "file"); } catch (error) { if (["EPERM", "EACCES", "ENOSYS"].includes(error.code)) return context.skip(`symlinks unavailable: ${error.code}`); throw error; }
		escaped.manifest.variants[0].artifacts[0].path = "artifacts/escape.bin";
		await writeFile(escaped.manifestPath, JSON.stringify(escaped.manifest));
		await assert.rejects(() => resolveFirmwareArtifact(variantId, "usb", escaped.manifestPath), /escapes bundle/i);
	} finally {
		await rm(escaped.fixtureRoot, { recursive: true, force: true });
	}

	const linkedRoot = await fixture();
	try {
		const realRoot = join(linkedRoot.fixtureRoot, "real-artifacts");
		await cp(join(linkedRoot.fixtureRoot, "artifacts"), realRoot, { recursive: true });
		await rm(join(linkedRoot.fixtureRoot, "artifacts"), { recursive: true });
		try { await symlink(realRoot, join(linkedRoot.fixtureRoot, "artifacts"), "dir"); } catch (error) { if (["EPERM", "EACCES", "ENOSYS"].includes(error.code)) return context.skip(`directory symlinks unavailable: ${error.code}`); throw error; }
		await assert.rejects(() => resolveFirmwareArtifact(variantId, "usb", linkedRoot.manifestPath), /artifact root/i);
	} finally {
		await rm(linkedRoot.fixtureRoot, { recursive: true, force: true });
	}
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
		await assert.rejects(() => resolveFirmwareArtifact(variantId, "usb", join(fixtureRoot, "firmware-manifest.json")), /unavailable/i);
	} finally {
		await rm(fixtureRoot, { recursive: true, force: true });
	}
});
