// Build script for Vercel deployment
import { spawnSync } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const site = path.join(root, 'site');

function run(command, args, cwd = site) {
  const isWin = process.platform === 'win32';
  const executable = isWin ? `${command}.cmd` : command;
  const result = spawnSync(executable, args, { cwd, stdio: 'inherit', shell: isWin });
  if (result.error || result.status !== 0) {
    throw result.error || new Error(`${command} failed with exit code ${result.status}`);
  }
}

console.log('Installing locked dependencies in site/...');
run('pnpm', ['install', '--frozen-lockfile']);

console.log('Checking configuration...');
run('pnpm', ['check']);

console.log('Building and verifying site...');
run('pnpm', ['build']);
