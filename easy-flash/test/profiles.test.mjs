import assert from "node:assert/strict";
import test from "node:test";
import { makeReceipt, resolveProfile } from "../profiles.mjs";

test("resolves the reference tube into explicit settings and ownership", () => {
	const plan = resolveProfile({ boardId: "dig2go", lightId: "referenceTube", powerId: "medium" });
	assert.equal(plan.resolved.firmwareTarget, "esp32_quinled_dig2go_tubes");
	assert.equal(plan.resolved.gpio, 16);
	assert.equal(plan.resolved.stripType, "SK6812 RGBW (confirm)");
	assert.equal(plan.resolved.colorOrder, "GRBW");
	assert.equal(plan.resolved.logicalPixelCount, 60);
	assert.equal(plan.resolved.currentLimitMilliamps, 1000);
	assert.equal(plan.hardwareArtifact.targetId, "dig2go");
	assert.equal(plan.softwareProfile.status, "waiting-for-split-protocol");
	assert.equal(plan.softwareProfile.requiresFirmware, false);
	assert.equal(plan.mode, "simulated-dry-run");
});

test("accepts bounded exact overrides", () => {
	const plan = resolveProfile({ boardId: "dig2go", lightId: "custom", powerId: "low", stripType: "WS2812B RGB", colorOrder: "RGB", pixels: 144, milliamps: 1750 });
	assert.equal(plan.resolved.logicalPixelCount, 144);
	assert.equal(plan.resolved.currentLimitMilliamps, 1750);
});

test("rejects unsupported boards and unsafe ranges", () => {
	assert.throws(() => resolveProfile({ boardId: "athomC3", lightId: "rgb", powerId: "low" }), /not supported/);
	assert.throws(() => resolveProfile({ boardId: "dig2go", lightId: "rgb", powerId: "low", pixels: 0 }), /pixel count/);
	assert.throws(() => resolveProfile({ boardId: "dig2go", lightId: "rgb", powerId: "low", milliamps: 99999 }), /limits current requests/);
	assert.throws(() => resolveProfile({ boardId: "dig2go", lightId: "rgb", powerId: "low", stripType: " " }), /Strip type/);
});

test("receipt is explicit about simulation", () => {
	const receipt = makeReceipt(resolveProfile({ boardId: "dig2go", lightId: "rgbw", powerId: "high" }), new Date("2026-08-05T12:00:00Z"));
	assert.equal(receipt.createdAt, "2026-08-05T12:00:00.000Z");
	assert.match(receipt.result, /SIMULATED/);
});
