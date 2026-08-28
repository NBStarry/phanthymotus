/** Local simulation services remain visible without a Resource Center entry. */
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const panel = readFileSync(new URL('./deploy-panel.js', import.meta.url), 'utf8');

test('local service status becomes a visible orphan item', () => {
  assert.match(panel, /_localManaged: Boolean\(s\.local_managed\)/);
  assert.match(panel, />本地仿真<\/span>/);
});

test('local compose services cannot be uninstalled from their status row', () => {
  assert.match(panel, /if \(!s\.local_managed\) \{/);
  assert.match(panel, /data-action="remove"/);
});
