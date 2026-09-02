const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const ui = fs.readFileSync('usermods/WaveshareS3TubesRemote/WaveshareS3TubesRemote.cpp','utf8');
const tubes = fs.readFileSync('usermods/Tubes/Tubes.h','utf8');
const bridge = fs.readFileSync('usermods/Tubes/Tubes.cpp','utf8');
const controller = fs.readFileSync('usermods/Tubes/controller.h','utf8');

function readHarmonyControls(name) {
  const controls = ui.match(new RegExp(`static constexpr HarmonyControl ${name}\\[\\d+\\] = \\{([\\s\\S]*?)\\n    \\};`));
  assert.ok(controls, `missing ${name}`);
  return [...controls[1].matchAll(/\{(\d+),\s*(-?\d+),\s*(\d+),\s*(\d+)\}/g)]
    .map(([, position, hueOffset, saturation, value]) => ({
      position: Number(position),
      hueOffset: Number(hueOffset),
      saturation: Number(saturation),
      value: Number(value)
    }));
}

function readSamplePositions(name) {
  const samples = ui.match(new RegExp(`static constexpr uint8_t ${name}\\[GRADIENT_MAX_STOPS\\] = \\{\\s*([\\d,\\s]+)\\n    \\};`));
  assert.ok(samples, `missing ${name}`);
  return samples[1].split(',').map(value => Number(value.trim())).filter(Number.isFinite);
}

function sampleHueOffset(controls, position) {
  if (position <= controls[0].position) return controls[0].hueOffset;
  for (let index = 1; index < controls.length; index++) {
    const first = controls[index - 1];
    const second = controls[index];
    if (position > second.position) continue;
    const amount = (position - first.position) / (second.position - first.position);
    const eased = amount * amount * (3 - 2 * amount);
    return first.hueOffset + (second.hueOffset - first.hueOffset) * eased;
  }
  return controls.at(-1).hueOffset;
}

function circularHueDistance(first, second) {
  const difference = Math.abs(first - second) % 256;
  return Math.min(difference, 256 - difference);
}

test('Patterns remains on canonical completed WLED framebuffer', () => {
  assert.match(ui, /::strip\.getPixelColor\(i\)/);
  assert.doesNotMatch(ui, /BusManager::getBus\([^)]*\)->getPixelColor/);
});
test('Field OS offers direct pattern and gradient scheduling without legacy controls', () => {
  assert.match(ui, /tubesS3SelectPattern/);
  assert.match(ui, /tubesS3ScheduleGradient/);
  assert.match(bridge, /return tubes\.s3ForceNext\(\)/);
  assert.match(tubes, /return controller\.force_next_if_authoritative\(\)/);
  assert.doesNotMatch(ui, /Previous|SetMasterAuthority|SetAnchorAuthority|Update\.begin|esp_ota_begin|F\("Channels"\)/);
  assert.doesNotMatch(tubes, /force_next_pattern\(/);
  assert.match(ui, /COULD NOT SET PATTERN/);
});
test('Mesh is bounded, fresh, and read-only', () => {
  const mesh = ui.match(/void drawMeshContent\(\) \{([\s\S]*?)\n  \}\n\n  static uint32_t blendColor/)[1];
  assert.match(ui, /TubesS3PeerStatus sorted\[4\]/);
  assert.match(ui, /now - candidate\.lastSeenMs > 60000/);
  assert.match(ui, /class MeshView final : public FieldView/);
  assert.doesNotMatch(mesh, /THIS S3|THIS DEVICE|NEARBY DEVICES/);
  assert.match(ui, /drawDeviceCard\(106 \+ i \* 70/);
  assert.match(ui, /Rect\{120, 408, 240, 56\}[\s\S]*FieldViewId::Update/);
});

test('Colors samples vivid anchored harmony paths into eight positioned stops', () => {
  assert.match(ui, /FieldViewId::Colors/);
  assert.match(ui, /class ColorsView final : public FieldView/);
  assert.match(ui, /TubesS3GradientStop gradientStops\[GRADIENT_MAX_STOPS\]/);
  assert.match(ui, /GradientPreset gradientPresets\[GRADIENT_PRESET_COUNT\]/);
  assert.match(ui, /void regenerateGradientPresets\(\)/);
  assert.match(ui, /struct HarmonyControl/);
  assert.match(ui, /struct HarmonySample/);
  assert.match(ui, /GRADIENT_PRESET_MIN_SATURATION = 144/);
  assert.match(ui, /GRADIENT_PRESET_MAX_SATURATION = 232/);
  assert.match(ui, /GRADIENT_PRESET_MIN_VALUE = 208/);
  assert.match(ui, /GRADIENT_PRIMARY_HUE_RADIUS = 22/);
  assert.match(ui, /primaryHues\[3\] = \{0, 85, 171\}/);
  assert.match(ui, /shapeCompanionHue\(sampledHue, presetIndex \+ stop\)/);
  assert.match(ui, /hsv2rgb_spectrum\(CHSV\(hue, saturation, value\), color\)/);
  assert.doesNotMatch(ui, /const CRGB color = CHSV\(hue, saturation, value\)/);
  assert.match(ui, /sampleHarmony/);
  assert.match(ui, /ease8InOutCubic\(linearAmount\)/);
  assert.match(ui, /presetSaturation\(sample\.saturation\)/);
  assert.match(ui, /presetValue\(sample\.value\)/);
  assert.match(ui, /writeHarmonyPreset/);
  const harmonies = [
    ['analogousControls', 'smoothSamples'],
    ['accentedMonochromaticControls', 'accentedMonochromaticSamples'],
    ['complementaryControls', 'complementarySamples'],
    ['splitComplementaryControls', 'splitComplementarySamples'],
    ['compoundControls', 'compoundSamples'],
    ['triadicControls', 'triadicSamples'],
    ['tetradicControls', 'tetradicSamples'],
    ['kulerCustomControls', 'kulerCustomSamples']
  ];
  const sampledHarmonyOffsets = [];
  for (const [controlName, sampleName] of harmonies) {
    const controls = readHarmonyControls(controlName);
    const samples = readSamplePositions(sampleName);
    assert.ok(controls.length > 1, `${controlName} needs a color path`);
    assert.equal(samples.length, 8, `${sampleName} must fill the wire format`);
    assert.deepEqual(controls.map(stop => stop.position),
      controls.map(stop => stop.position).toSorted((left, right) => left - right),
      `${controlName} positions must remain ordered`);
    assert.deepEqual(samples, samples.toSorted((left, right) => left - right),
      `${sampleName} positions must remain ordered`);
    for (const control of controls) {
      assert.ok(control.saturation >= 144, `washed out control in ${controlName}`);
      assert.ok(control.saturation <= 232, `primary-fill control in ${controlName}`);
      assert.ok(control.value >= 208, `dark control in ${controlName}`);
    }
    const sampledOffsets = samples.map(position => sampleHueOffset(controls, position));
    sampledHarmonyOffsets.push([controlName, sampledOffsets]);
    for (let index = 1; index < samples.length; index++) {
      const hueDistance = circularHueDistance(sampledOffsets[index - 1], sampledOffsets[index]);
      const positionDistance = samples[index] - samples[index - 1];
      assert.ok(hueDistance <= 48 || positionDistance <= 16,
        `${controlName} spreads a large hue jump across too much of the gradient`);
    }
  }
  for (let first = 0; first < sampledHarmonyOffsets.length; first++) {
    for (let second = first + 1; second < sampledHarmonyOffsets.length; second++) {
      const [firstName, firstOffsets] = sampledHarmonyOffsets[first];
      const [secondName, secondOffsets] = sampledHarmonyOffsets[second];
      const difference = firstOffsets.reduce((total, offset, index) =>
        total + circularHueDistance(offset, secondOffsets[index]), 0);
      assert.ok(difference >= 96, `${firstName} is too similar to ${secondName}`);
    }
  }
  const analogousOffsets = readHarmonyControls('analogousControls').map(stop => stop.hueOffset);
  assert.ok(Math.max(...analogousOffsets) - Math.min(...analogousOffsets) >= 80,
    'the first suggestion must cover more than one narrow color family');
  const accentedMonochromaticOffsets = readHarmonyControls('accentedMonochromaticControls')
    .map(stop => stop.hueOffset);
  assert.ok(accentedMonochromaticOffsets.filter(offset => Math.abs(offset) <= 8).length >= 4,
    'the second suggestion must retain a recognizable base-color family');
  assert.ok(accentedMonochromaticOffsets.some(offset => Math.abs(offset - 128) <= 8),
    'the second suggestion must include a complementary accent');
  assert.ok(readHarmonyControls('complementaryControls').some(stop => Math.abs(stop.hueOffset - 128) <= 16));
  assert.ok(readHarmonyControls('splitComplementaryControls').some(stop => Math.abs(stop.hueOffset - 106) <= 12));
  assert.ok(readHarmonyControls('splitComplementaryControls').some(stop => Math.abs(stop.hueOffset - 150) <= 12));
  assert.ok(readHarmonyControls('triadicControls').some(stop => Math.abs(stop.hueOffset - 78) <= 12));
  assert.ok(readHarmonyControls('triadicControls').some(stop => Math.abs(stop.hueOffset - 180) <= 12));
  assert.ok(readHarmonyControls('tetradicControls').some(stop => Math.abs(stop.hueOffset - 58) <= 12));
  assert.ok(readHarmonyControls('tetradicControls').some(stop => Math.abs(stop.hueOffset - 118) <= 16));
  assert.ok(readHarmonyControls('tetradicControls').some(stop => Math.abs(stop.hueOffset - 202) <= 16));
  assert.doesNotMatch(ui, /HarmonyRecipeStop|writeWheelHarmonyPreset|writePaletteTemplatePreset|templateSaturation|templateValue|nearestRecipeStop|nearestTemplateStop|blueCyanYellowTemplate|fireIceTemplate|partyTemplate|shadesPositions/);
  assert.match(ui, /preset\.stops\[stop\]\.color = anchor/);
  assert.match(ui, /sampleHarmony\(\s*controls, controlCount, anchorPosition\)/);
  assert.match(ui, /tubesS3ScheduleGradient\(gradientStops, gradientStopCount\)/);
  assert.match(ui, /for \(uint8_t preset = 0; preset < GRADIENT_PRESET_COUNT; preset\+\+\)/);
  assert.match(ui, /const int16_t y = 312 \+ \(preset \/ 4\) \* 64/);
  assert.match(ui, /drawGradientPreview\(\{x, y, 105, 52\}, gradientPresets\[preset\]\.stops/);
  assert.match(ui, /owner\.selectGradientStopAt\(x\)/);
  assert.match(ui, /owner\.regenerateGradientPresets\(\);\s*owner\.queueGradient\(\)/);
  assert.match(ui, /void applyGradientPreset[\s\S]*queueGradient\(\)/);
  assert.doesNotMatch(ui, /EDIT NEXT GRADIENT|GRADIENT QUEUED|F\("Set next"\)/);
  assert.doesNotMatch(ui, /FieldViewId::Channels|class ChannelsView|LIVE CHANNEL AUTHORITY/);
  assert.match(ui, /hueFromPaletteX\(x\)/);
  assert.match(ui, /for \(int16_t x = 24; x < 456; x \+= hueSliceWidth\)/);
  assert.doesNotMatch(ui, /gradientHue/);
});

test('Beats controls listening, downbeat, and a bounded overlay grid', () => {
  assert.match(ui, /class BeatsView final : public FieldView/);
  assert.match(ui, /tubesS3SetTempoListening\(!status\.tempoListening\)/);
  assert.match(ui, /tubesS3TapDownbeat\(\)/);
  assert.match(ui, /tubesS3SelectBeatOverlay\(row \* 3 \+ column\)/);
  assert.match(ui, /for \(uint8_t choice = 0; choice < 9; choice\+\+\)/);
  assert.match(ui, /display\.printf\("%u BPM", status\.bpm\)/);
  assert.match(ui, /bin < TUBES_S3_MICROPHONE_BINS/);
  assert.match(ui, /status\.microphoneSpectrum\[bin\]/);
  assert.match(ui, /F\("Listen"\)/);
  assert.match(ui, /F\("1"\)/);
  assert.doesNotMatch(ui, /F\("Listen ON"\)|F\("Listen OFF"\)|F\("Downbeat"\)/);
  assert.match(tubes, /status\.microphoneSpectrum\[bin\] = controller\.sound\.spectrumLevels\[bin\]/);
  assert.match(bridge, /return tubes\.s3SetTempoListening\(enabled\)/);
  assert.match(bridge, /return tubes\.s3TapDownbeat\(\)/);
  assert.match(bridge, /return tubes\.s3SelectBeatOverlay\(choice\)/);
  assert.match(controller, /sound\.setTempoTracking\(enabled\)/);
  assert.match(controller, /beats\.start_phrase\(\);\s*update_beat\(\);\s*publishApplicationChannel\(BeatChannel\);\s*send_update\(\)/);
  assert.match(controller, /choice > S3_BEAT_OVERLAY_COUNT/);
  assert.match(controller, /gSoundPrograms\[choice - 1\]/);
  assert.doesNotMatch(controller, /AUTO_TEMPO pinned on/);
});
