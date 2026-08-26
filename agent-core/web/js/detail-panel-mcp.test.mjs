/**
 * Topic details must mount renderers with the card's real MCP id.
 *
 * SkeletonRenderer uses that id to call the matching Driver's `model` tool.
 * Passing the old literal `detail` made the detail panel fall back to a generic
 * skeleton while the monitor dashboard used the correct G1 URDF.
 *
 * Run: node --test "agent-core/web/js/*.test.mjs"
 */
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const detailPanel = readFileSync(new URL('./detail-panel.js', import.meta.url), 'utf8');
const canvas = readFileSync(new URL('./canvas.js', import.meta.url), 'utf8');

test('the detail panel forwards a real MCP id to the renderer', () => {
  assert.match(detailPanel, /showTopicDetail\(topicPath, format, mcpId = ''\)/);
  assert.match(detailPanel, /_renderer\.mount\(body, mcpId \|\| 'detail'\)/);
});

test('both canvas topic-detail entry points pass their card MCP id', () => {
  const calls = canvas.match(/showTopicDetail\(topics\[0\]\.topic, topics\[0\]\.format \|\| '', mcpId\)/g) || [];
  assert.equal(calls.length, 2);
});
