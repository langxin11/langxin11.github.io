// Vercel and other local builds consume the same content repository as Pages.
import { existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { spawnSync } from 'node:child_process';
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const content = path.join(root, 'content');
function run(command, args, cwd = root) {
  const result = spawnSync(command, args, { cwd, stdio:'inherit', shell:process.platform === 'win32' });
  if (result.error || result.status !== 0) throw result.error || new Error(`${command} failed with ${result.status}`);
}
if (!existsSync(path.join(content, '.git'))) {
  run('git', ['clone', '--depth', '1', 'https://github.com/langxin11/Mizuki_Content_XDL.git', content]);
} else {
  run('git', ['fetch', '--depth', '1', 'origin', 'main'], content);
  run('git', ['checkout', '--detach', 'FETCH_HEAD'], content);
}
const manager = process.platform === 'win32' ? 'pnpm.cmd' : 'pnpm';
run(manager, ['install', '--frozen-lockfile'], path.join(content, 'site'));
run(manager, ['check'], path.join(content, 'site'));
run(manager, ['build'], path.join(content, 'site'));
