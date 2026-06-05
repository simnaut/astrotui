# mermaid-lint

Validates every ` ```mermaid ` block in a markdown file by **rendering it with
[`mmdc`](https://github.com/mermaid-js/mermaid-cli)** — the only faithful check,
since it runs mermaid's real parser. Catches the things hand-authoring trips on
(bare `#NN` read as entity refs, quotes in `-. .->` link labels, stray `<…>` in
node text, unbalanced brackets).

## Setup

```sh
cd tools/mermaid-lint
npm install            # pulls @mermaid-js/mermaid-cli + a headless Chromium
```

In a container/CI you typically need Chromium's sandbox disabled — that's already
configured in `puppeteer-config.json` (`--no-sandbox`), which `lint.mjs` passes to
`mmdc`.

## Use

```sh
# lint specific files
node lint.mjs ../../docs/DESIGN.md
node lint.mjs ../../docs/DESIGN.md /path/to/other.md

# or the convenience script for the design doc
npm run lint:design
```

Prints a ✓/✗ per diagram (with its line number and first line) and exits non-zero
if any diagram fails — suitable for a pre-commit hook or a CI step.

## Notes

- `node_modules/` and the rendered `*.svg` are git-ignored; run `npm install` once.
- The wiki lives in a separate repo; point the linter at a local clone, e.g.
  `node lint.mjs ../../../astrotui.wiki/Reference-Frame-Architecture.md`.
