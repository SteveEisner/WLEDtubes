import { createHash } from "node:crypto";
import { readFile, stat } from "node:fs/promises";
import { dirname, join, resolve, sep } from "node:path";
import { fileURLToPath } from "node:url";

const root = fileURLToPath(new URL("./", import.meta.url));
const defaultManifestPath = join(root, "firmware-manifest.json");

function validateManifest(manifest) {
	if (manifest?.schemaVersion !== 2 || !Array.isArray(manifest.variants) || manifest.variants.length < 1) throw new Error("Invalid firmware manifest schema");
	for (const variant of manifest.variants) {
		if (!/^[a-z0-9-]+$/.test(variant.id) || variant.hardwareTested !== false || !variant.target?.hardwareFamily || !variant.target?.chip || !Number.isInteger(variant.target?.flashSizeBytes) || !variant.partition?.tableSha256 || !variant.transportAssumptions || !Array.isArray(variant.artifacts) || variant.artifacts.length !== 2) throw new Error("Invalid firmware variant");
	}
	return manifest;
}

export async function loadFirmwareManifest(manifestPath = defaultManifestPath) {
	return validateManifest(JSON.parse(await readFile(manifestPath, "utf8")));
}

// Resolves exclusively through validated manifest records and verifies bytes before returning a path.
export async function resolveFirmwareArtifact(variantId, transport, manifestPath = defaultManifestPath, expectedTarget = null) {
	if (!/^[a-z0-9-]+$/.test(variantId)) throw new Error("Invalid variant ID");
	if (!new Set(["usb", "ota"]).has(transport)) throw new Error("Invalid transport");
	const manifest = await loadFirmwareManifest(manifestPath);
	const variant = manifest.variants.find(({ id }) => id === variantId);
	if (!variant) throw new Error("Unknown variant ID");
	if (expectedTarget && (expectedTarget.hardwareFamily !== variant.target.hardwareFamily || expectedTarget.chip !== variant.target.chip || expectedTarget.flashSizeBytes !== variant.target.flashSizeBytes || (expectedTarget.partitionTableSha256 && expectedTarget.partitionTableSha256 !== variant.partition.tableSha256))) throw new Error("Target contract mismatch");
	const artifact = variant.artifacts.find((item) => item.transport === transport);
	const expectedKind = transport === "usb" ? "complete-merged-image" : "application-image";
	if (!artifact || artifact.kind !== expectedKind || (transport === "usb" && (artifact.offset !== 0 || (artifact.flashSizeBytes && (artifact.flashSizeBytes !== variant.target.flashSizeBytes || artifact.sizeBytes !== artifact.flashSizeBytes)))) || (transport === "ota" && (artifact.offset !== variant.partition.otaSlot.offset || artifact.sizeBytes > variant.partition.otaSlot.sizeBytes || (artifact.headroomBytes != null && artifact.sizeBytes + artifact.headroomBytes !== variant.partition.otaSlot.sizeBytes)))) throw new Error("Artifact transport contract mismatch");
	const absolutePath = resolve(dirname(manifestPath), artifact.path);
	const artifactRoot = resolve(dirname(manifestPath), "artifacts") + sep;
	if (!absolutePath.startsWith(artifactRoot)) throw new Error("Artifact path escapes bundle");
	const bytes = await readFile(absolutePath);
	if ((await stat(absolutePath)).size !== artifact.sizeBytes || createHash("sha256").update(bytes).digest("hex") !== artifact.sha256) throw new Error("Artifact integrity verification failed");
	return { manifest, variant, artifact, absolutePath };
}
