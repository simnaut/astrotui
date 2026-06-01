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
  `claude-review`. `claude-review` is an **independent CI review** (the `Claude Code
  Review` workflow) that runs automatically on every PR and posts findings as inline
  review threads; those threads must be resolved before merge (required conversation
  resolution). This loop does NOT self-review — it responds to the CI review (Phase A).
  Never bypass with `gh pr merge --admin`.

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
   - **Review (independent CI):** the `Claude Code Review` workflow reviews every PR
     automatically and emits the required `claude-review` check, posting findings as
     inline review threads. Do NOT self-review. Instead: wait for `claude-review` to
     finish, then **address every finding it posted** — fix on the branch and push
     (re-triggers CI + a fresh review), or, only if a comment is a genuine false
     positive, reply explaining why. Resolve each thread via `resolveReviewThread`. The
     review covers correctness, design fidelity to `docs/DESIGN.md`, the Bevy/ANISE
     firewall, test adequacy, code quality, and scope — treat its comments as blocking.
   - **Review threads:** ensure every unresolved thread (from the CI review or anywhere
     else, e.g. a human) is addressed on the branch and resolved via `resolveReviewThread`.
   - **Merge:** only when **all three required checks are green** (`test`, `firewall`,
     `claude-review`) AND zero unresolved threads AND the PR is mergeable → `gh pr merge
     <N> --auto --squash` (auto-merge is enabled; GitHub merges once the gates pass).
     Never bypass with `--admin`.
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
