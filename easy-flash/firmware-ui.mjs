export function getHardwareArtifacts(manifest) {
	return [...manifest.variants];
}

export function makeFirmwareOperationReceipt(variant, artifact, now = new Date()) {
	return { createdAt: now.toISOString(), variantId: variant.id, transport: artifact.transport, artifactSha256: artifact.sha256, target: variant.target, partition: variant.partition, sourceCommit: variant.source.commit, result: "prepared-not-written" };
}
