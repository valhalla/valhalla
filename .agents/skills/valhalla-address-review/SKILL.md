---
name: valhalla-address-review
description: >-
  Address maintainer review feedback on a Valhalla pull request. Use when a reviewer asks a
  question, raises a concern or hypothesis, requests a change or test, or leaves unresolved review
  comments on a contribution to valhalla/valhalla. This repository workflow takes precedence over
  personal review workflows.
---

# Address Valhalla review feedback

1. Read `CLAUDE.md`, `CONTRIBUTING.md`, the pull request diff, and its linked issue.
2. Read the complete review thread, including its parent and every reply. Check all unresolved
   threads and newer feedback; do not act on one quoted comment in isolation.
3. Determine what evidence settles each comment:
   - For a question, concern, or hypothesis, trace the relevant code and configuration precedence,
     then run a focused test or probe.
   - For a requested change, run the related tests as a baseline, add a failing regression test
     where applicable, then make the smallest fix.
   - If repository evidence proves the current code is correct, reply with that evidence instead
     of making a placebo change.
4. Ask the reviewer for clarification only when the answer cannot be found in the repository or
   established with a test.
5. Keep the change narrow. Do not add adjacent cleanup, explanatory abstractions, or source comments
   that describe review history.
6. Run `./scripts/format.sh`, `pre-commit run --all-files`, any focused regression test, and every
   related test identified during the investigation.
7. Commit review changes separately and push normally. Never amend or force-push after the first
   review.
8. Re-read every review thread after pushing so no question or new comment is missed.
9. Reply inline with only information that is not already visible in the diff. Do not restate the
   code, list validation results, repeat the review comment, or summarize the pull request unless
   the reviewer asks. One short sentence is enough for a self-explanatory change.

Leave maintainer threads open for the reviewer to confirm unless they ask the contributor to resolve
them.
