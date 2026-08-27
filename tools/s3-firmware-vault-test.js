'use strict';

const assert = require('node:assert');
const { it } = require('node:test');
const path = require('node:path');
const { spawnSync } = require('node:child_process');
const { mkdtempSync, rmSync } = require('node:fs');
const { tmpdir } = require('node:os');

it('enforces the S3 one-client firmware baton policy', () => {
  const repository = path.resolve(__dirname, '..');
  const temporary = mkdtempSync(path.join(tmpdir(), 's3-vault-test-'));
  const executable = path.join(temporary, 's3-vault-test');
  try {
    const compile = spawnSync(process.env.CXX || 'c++', [
      '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
      path.join(__dirname, 's3-firmware-vault-test.cpp'), '-o', executable,
    ], { cwd: repository, encoding: 'utf8' });
    assert.strictEqual(compile.status, 0, compile.stderr || compile.stdout);
    const run = spawnSync(executable, [], { cwd: repository, encoding: 'utf8' });
    assert.strictEqual(run.status, 0, run.stderr || run.stdout);
    assert.match(run.stdout, /vault policy scenarios passed/);
  } finally {
    rmSync(temporary, { recursive: true, force: true });
  }
});
