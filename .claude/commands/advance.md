---
description: Advance the astrotui roadmap one cycle — shepherd open PRs to merge, then start the earliest unblocked task. Idempotent.
---

You are the autonomous build driver for the **simnaut/astrotui** repository and its
roadmap **Project #1**. Advance the project by one cycle, idempotently, then continue
until there is no actionable work or you hit a stop condition. Reconcile ALL state from
GitHub — never assume memory from a prior run.

## Context
- Repo: `simnaut/astrotui` (default branch: `main`). Project #1 (owner `simnaut`) with
  fields **Phase**, **Type**, **Status**.
- Backlog = GitHub issues. `type:epic` issues are containers — **never implement them
  directly**; do work only on `type:task` (code) and `type:spec` (write the design doc,
  not code) issues.
- Dependencies are fully encoded as GitHub **blocked-by** edges (phase chain +
  open-item gates + intra-phase order). An issue is **unblocked** iff every issue under
  `gh api repos/simnaut/astrotui/issues/<N>/dependencies/blocked_by` is CLOSED.
- The design doc `docs/DESIGN.md` is the source of truth. Respect the architectural
  firewall: **astrotui-core links no Bevy and no ANISE/ephemeris** (Bevy lives only in
  the sim-side exporter / `apps/refsim`; ephemeris only in the body-filler producer).
- Follow my global `CLAUDE.md` conventions: fetch unresolved PR review threads via the
  GraphQL query and resolve them with `resolveReviewThread`; edit PR bodies via the REST
  PATCH (`gh pr edit` fails on this repo).
- **Merge gate:** `main` is protected (squash-only, no direct pushes, enforced for
  admins) and requires three status checks green before merge — `test`, `firewall`, and
  `review`. `review` is the rigorous review gate (Phase A below); this loop sets it on the
  PR head commit only after an independent review comes back clean. Never bypass with
  `gh pr merge --admin`.

## Selection order ("earliest")
1. Restrict to the **earliest phase (milestone)** that still has any open
   `type:task`/`type:spec` issue — milestone order is Pre-P0 → P0 → P1 → P2 → P3 → P4.
   Do not start a later phase while an earlier one has open work.
2. Within that phase, pick the earliest **unblocked** issue by ascending issue number.

## The cycle — run PHASE A first (open PRs are closest to done), then PHASE B.

### PHASE A — drive existing PRs to merge
1. `gh pr list --repo simnaut/astrotui --state open --json number,title,headRefName,body,isDraft`.
   Process oldest first.
2. For each open, non-draft PR:
   - **CI:** `gh pr checks <N>`. If `test` or `firewall` is FAILING → `gh pr checkout <N>`,
     diagnose, fix, run `cargo fmt && cargo clippy --all-targets -- -D warnings && cargo test`
     locally, commit (with the `Co-Authored-By` trailer) and push. If checks are only
     PENDING, leave the PR for a later cycle.
   - **Rigorous review gate:** once `test`+`firewall` are green, before merging, run an
     INDEPENDENT, rigorous review of the PR diff — use the `code-review` skill at high
     effort with `--comment`, or spawn a dedicated reviewer subagent for deeper analysis.
     Review for:
       - **correctness** — bugs, edge cases, error handling, concurrency/ordering;
       - **design fidelity** — matches `docs/DESIGN.md` and the locked decisions;
       - **firewall** — `astrotui-core` links no Bevy/ANISE; producers stay external;
       - **tests** — present, meaningful, and genuinely exercised (never green-washed);
       - **quality** — no shims/hacks/dead TODOs, no needless complexity, idiomatic Rust,
         naming + comment density matching surrounding code;
       - **scope** — one issue, a reviewable diff.
     If anything blocking is found: post it as PR review comments, fix it on the branch,
     push (re-triggers CI), and re-review until clean. When the review comes back clean,
     record the gate on the PR head commit:
     `gh api -X POST repos/simnaut/astrotui/statuses/$(git rev-parse HEAD) -f state=success -f context=review -f description="rigorous review passed"`
     (re-set it on the new HEAD sha after any later fix push).
   - **Review threads:** ensure every unresolved thread (from the gate above or anywhere
     else) is addressed on the branch and resolved via `resolveReviewThread`.
   - **Merge:** only when **all three required checks are green** (`test`, `firewall`,
     `review`) AND zero unresolved threads AND the PR is mergeable → `gh pr merge <N>
     --auto --squash` (auto-merge is enabled; GitHub merges once the gates pass). Never
     bypass with `--admin`.
3. A merged PR with `Closes #<issue>` closes its issue; set that issue's Project Status
   to **Done** if it didn't happen automatically.

### PHASE B — start the next task (only if it has no open PR yet)
1. Select the earliest unblocked open `type:task`/`type:spec` issue (per the order
   above) that has **no existing open PR** (check open PR bodies for `Closes #<N>` and
   branches named `issue-<N>-*`). If none exists, **STOP** and report "no actionable work."
2. **Idempotency:** if a branch `issue-<N>-*` or a PR for `#<N>` already exists, switch
   to PHASE A on it instead of creating a duplicate.
3. Mark start: set the issue's Project Status to **In Progress**; comment that you're
   starting it.
4. `git fetch origin && git switch -c issue-<N>-<slug> origin/main`.
5. Implement strictly per the issue body + `docs/DESIGN.md`. For `type:spec` issues,
   write the design document instead of code.
6. Add/update tests; run `cargo fmt`, `cargo clippy --all-targets -- -D warnings`, and
   `cargo test` until green locally. (Until the workspace/CI exists — the Pre-P0 issue —
   there may be nothing to run yet; bootstrap it exactly as that issue specifies.)
7. Commit (with the `Co-Authored-By: Claude` trailer), push, and open a PR: base `main`,
   body starting with `Closes #<N>`, noting the parent epic and what you verified. One
   issue per PR; keep the diff reviewable.
8. Return to PHASE A to shepherd the new PR, or end the cycle if you should pause for review.

## Guardrails
- Never commit directly to `main`; always go through an issue branch + PR.
- Don't fabricate green tests. If something fails and you can't fix it, leave the PR
  open, comment with the blocker, and move to the next item.
- If a task is genuinely ambiguous or needs a product decision, comment the question on
  the issue and skip it — don't guess.
- Stop and summarize when: no actionable work remains; the only open PRs are pending-CI
  or awaiting human review; or you hit a blocker needing my input.

**Begin now:** report what you find (open PRs, the selected next task), then act.
