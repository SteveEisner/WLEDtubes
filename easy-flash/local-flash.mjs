import { ESPLoader, Transport } from "./vendor/esptool-js/bundle.js";

let flashInProgress = false;

export function chipFamily(value) {
	const chip = String(value).toUpperCase().replaceAll("_", "-").replace(/\s+/g, "");
	for (const family of ["ESP32-S3", "ESP32-S2", "ESP32-C2", "ESP32-C3", "ESP32-C5", "ESP32-C6", "ESP32-C61", "ESP32-H2", "ESP32-P4", "ESP8266"]) {
		if (chip.startsWith(family)) return family;
	}
	if (chip === "ESP32" || chip.startsWith("ESP32-")) return "ESP32";
	return chip;
}

function expectedFlashSize(bytes) {
	const megabytes = bytes / (1024 * 1024);
	if (!Number.isInteger(megabytes)) throw new Error("Unsupported manifest flash size");
	return `${megabytes}MB`;
}

async function sha256Hex(bytes) {
	const digest = await crypto.subtle.digest("SHA-256", bytes);
	return Array.from(new Uint8Array(digest), (byte) => byte.toString(16).padStart(2, "0")).join("");
}

async function fetchVerifiedImage(variant, artifact) {
	const response = await fetch(`/api/firmware/${encodeURIComponent(variant.id)}/usb`, {
		headers: {
			"X-Easy-Flash-Hardware-Family": variant.target.hardwareFamily,
			"X-Easy-Flash-Chip": variant.target.chip,
			"X-Easy-Flash-Flash-Bytes": String(variant.target.flashSizeBytes),
			"X-Easy-Flash-Partition-Sha256": variant.partition.tableSha256,
		},
	});
	if (!response.ok) throw new Error("Firmware image unavailable");
	const bytes = new Uint8Array(await response.arrayBuffer());
	if (bytes.byteLength !== artifact.sizeBytes) throw new Error("Firmware image size mismatch");
	if (await sha256Hex(bytes) !== artifact.sha256) throw new Error("Firmware image hash mismatch");
	return bytes;
}

export function canFlashLocally() {
	return window.isSecureContext && "serial" in navigator;
}

export async function flashFromLaptop({ variant, artifact, onStatus, onProgress }) {
	if (flashInProgress) throw new Error("Another flash is already running");
	if (!canFlashLocally()) throw new Error("Use desktop Chrome or Edge over HTTPS");
	if (artifact.transport !== "usb" || artifact.kind !== "complete-merged-image") throw new Error("Local USB flashing requires a complete merged image");
	if (artifact.offset !== 0) throw new Error("Complete USB image must begin at offset 0x0");

	flashInProgress = true;
	let transport;
	try {
		onStatus("Choose the ESP connected to this laptop…");
		const port = await navigator.serial.requestPort();
		transport = new Transport(port, true);
		const terminal = {
			clean() {},
			writeLine(message) { if (/chip|detect|connect/i.test(message)) onStatus(message); },
			write(message) { if (/chip|detect|connect/i.test(message)) onStatus(message); },
		};
		const loader = new ESPLoader({ transport, baudrate: 115200, terminal, debugLogging: false });
		onStatus("Identifying chip…");
		const chipName = await loader.main();
		if (chipFamily(chipName) !== chipFamily(variant.target.chip)) {
			throw new Error(`Wrong chip: connected ${chipName}, selected ${variant.target.chip}`);
		}

		onStatus("Downloading and checking firmware…");
		const image = await fetchVerifiedImage(variant, artifact);
		const fileArray = artifact.components.map((component) => ({
			data: image.slice(component.offset, component.offset + component.sizeBytes),
			address: component.offset,
		}));
		if (!fileArray.length) throw new Error("Firmware image has no flash components");
		onStatus(`Flashing ${variant.label} without overwriting saved device settings…`);
		await loader.writeFlash({
			fileArray,
			flashMode: variant.target.flashMode || "keep",
			flashFreq: "keep",
			flashSize: expectedFlashSize(variant.target.flashSizeBytes),
			eraseAll: false,
			compress: false,
			reportProgress(_fileIndex, written, total) { onProgress(Math.round((written / total) * 100)); },
		});
		await loader.after("hard_reset");
		onProgress(100);
		onStatus(`Flash complete — ${chipName} reset`);
		return { chipName, bytesWritten: image.byteLength, sha256: artifact.sha256 };
	} finally {
		flashInProgress = false;
		if (transport) {
			try { await transport.disconnect(); } catch {}
		}
	}
}
