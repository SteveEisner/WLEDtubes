import assert from "node:assert/strict";
import test from "node:test";
import { inspectRunningDevice, resolveRunningDeviceIdentity } from "../device-identity.mjs";

const target={ hardwareFamily:"quinled-dig2go" };

test("running report needs exact release and hardware identity", () => {
	assert.equal(resolveRunningDeviceIdentity({info:{arch:"esp32",release:"DIG2GO_TUBES",hardwareFamily:"quinled-dig2go"},config:{}},target).matched,true);
	assert.equal(resolveRunningDeviceIdentity({info:{arch:"esp32",name:"WLED",version:"16.0.1",mac:"aa:bb"},config:{pin:16}},target).matched,false);
	assert.equal(resolveRunningDeviceIdentity({info:{arch:"esp32",release:"DIG2GO_TUBES"},config:{}},target).matched,false);
});

test("running-device acquisition is injected and fetches identity plus config", async () => {
	const seen=[]; const fetchImpl=async (url) => { seen.push(url.pathname); return {ok:true,async json(){return url.pathname.endsWith("si") ? {arch:"esp32",release:"DIG2GO_TUBES",hardwareVariant:"DIG2GO"} : {};}}; };
	assert.equal((await inspectRunningDevice("http://wled.local",fetchImpl,target)).matched,true);
	assert.deepEqual(seen.sort(),["/json/cfg","/json/si"]);
});
