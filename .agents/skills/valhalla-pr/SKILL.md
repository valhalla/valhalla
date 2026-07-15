---
name: valhalla-pr
description: Prepare or update a Valhalla pull request. Use before pushing a contribution, opening a PR, updating its description, or requesting CI for valhalla/valhalla. This repository workflow takes precedence over personal PR workflows.
---

# Valhalla pull request

1. Read `README.md`'s "Generative AI usage" section, `CONTRIBUTING.md`, `CLAUDE.md`, `docs/learnings.md`, and `.github/pull_request_template.md`.
2. Confirm that the PR links an issue. Use `/valhalla-issue` first if an issue is needed.
3. Identify the remote pointing to `valhalla/valhalla`, fetch `master`, and confirm the branch is based on the current upstream branch.
4. Review every changed line. Rewrite contrived logic, generic names, unnecessary abstractions, and comments that merely restate the code. Add inline PR comments for non-obvious decisions.
5. Run formatting and linting:

   ```bash
   ./scripts/format.sh
   pre-commit run --all-files
   ```

6. Build and run every affected test and related test. On x86_64, use `make check -j$(nproc)` as a final check when feasible. On arm64, avoid the unreliable full suite and run the relevant tests named in `CLAUDE.md` instead.
7. Report the exact commands and results. Name CI jobs that remain for GitHub when their matrix cannot be reproduced locally.
8. After CI runs, inspect every failed check's first failing step and full log before changing code. Classify each failure as contribution-related or infrastructure. Network, package registry, runner setup, and dependency download failures need a rerun, not a code change or no-op commit; report the failing step and ask a current reviewer or maintainer to rerun when permissions prevent it.
9. Fill the existing PR template without adding a file summary or repeating the diff. State the problem in one sentence, the fix in one sentence, and why it works in one or two sentences. Edit agent-written prose to sound like ordinary casual GitHub writing: replace stock AI phrasing and uncommon punctuation such as en dashes or em dashes with simple wording, commas, colons, parentheses, or separate sentences. Apply this cleanup only to wording the agent introduced; preserve code, identifiers, links, and direct quotes.
10. Read `docs/learnings.md`'s admission rule. If the work establishes a decision-changing lesson, add its one-line entry to the relevant section. Keep routine fixes and release history in `CHANGELOG.md` rather than the agent-facing learnings index.
11. Before creating or updating the PR, run `CLAUDE.md`'s contributor-agent gate against the full branch diff. Perform the review in the current agent; extra local review tools may supplement the gate. Resolve every finding and commit the fixes first. After any gate fix, repeat steps 5 and 6 before creating or updating the PR.
12. After the first review, preserve review history by adding commits and pushing normally.

If an agent drafts or translates the title or body, start the body with exactly:

`Tryin' to shortcut, arrr ye?`

Follow `CLAUDE.md`'s canonical policy for AI-authored or translated issue and pull request prose before submission.
