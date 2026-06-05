#!/usr/bin/env node
// Mermaid linter: validates every ```mermaid block in the given markdown files by
// rendering each with mmdc (the only faithful check — it runs the real parser).
//
// Usage:
//   node lint.mjs <file.md> [more.md ...]
//   npm run lint -- <file.md> ...
// Exit code is non-zero if any diagram fails to parse/render.

import { spawnSync } from "node:child_process";
import { mkdtempSync, writeFileSync, readFileSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import { join, dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const mmdc = resolve(here, "node_modules/.bin/mmdc");
const puppeteerCfg = resolve(here, "puppeteer-config.json");

const files = process.argv.slice(2);
if (files.length === 0) {
  console.error("usage: node lint.mjs <file.md> [more.md ...]");
  process.exit(2);
}

// Extract fenced ```mermaid blocks with their starting line number.
function blocks(md) {
  const lines = md.split("\n");
  const out = [];
  let cur = null;
  lines.forEach((line, i) => {
    if (cur === null && /^```mermaid\s*$/.test(line)) {
      cur = { startLine: i + 1, body: [] };
    } else if (cur !== null && /^```\s*$/.test(line)) {
      out.push({ startLine: cur.startLine, body: cur.body.join("\n") });
      cur = null;
    } else if (cur !== null) {
      cur.body.push(line);
    }
  });
  return out;
}

const work = mkdtempSync(join(tmpdir(), "mmlint-"));
let total = 0,
  failed = 0;

for (const file of files) {
  let md;
  try {
    md = readFileSync(file, "utf8");
  } catch {
    console.error(`✗ cannot read ${file}`);
    failed++;
    continue;
  }
  const bs = blocks(md);
  console.log(`\n${file} — ${bs.length} mermaid block(s)`);
  bs.forEach((b, idx) => {
    total++;
    const first = (b.body.trim().split("\n")[0] || "").trim();
    const inFile = join(work, `b${idx}.mmd`);
    writeFileSync(inFile, b.body);
    const r = spawnSync(
      mmdc,
      ["-i", inFile, "-o", join(work, `b${idx}.svg`), "-p", puppeteerCfg, "-q"],
      { encoding: "utf8" },
    );
    if (r.status === 0) {
      console.log(`  ✓ block ${idx + 1} (line ${b.startLine}) [${first.slice(0, 32)}]`);
    } else {
      failed++;
      const err = (r.stderr || r.stdout || "").trim().split("\n").slice(-6).join("\n      ");
      console.log(`  ✗ block ${idx + 1} (line ${b.startLine}) [${first.slice(0, 32)}]`);
      console.log(`      ${err}`);
    }
  });
}

rmSync(work, { recursive: true, force: true });
console.log(`\n${total - failed}/${total} diagrams valid` + (failed ? ` — ${failed} FAILED` : " — all good"));
process.exit(failed ? 1 : 0);
