---
name: valhalla-pr
description: >-
  Prepare or update a Valhalla pull request. Use before pushing a contribution, opening a PR,
  updating its description, or requesting CI for valhalla/valhalla. This repository workflow takes
  precedence over personal PR workflows.
---

# Valhalla pull request

1. Read `README.md`'s "Generative AI usage" section, `CONTRIBUTING.md`, `CLAUDE.md`, and
   `.github/pull_request_template.md`.
2. Confirm that the PR links an issue. Use `/valhalla-issue` first if an issue is needed.
3. Identify the remote pointing to `valhalla/valhalla`, fetch `master`, and confirm the branch is
   based on the current upstream branch.
4. Review every changed line. Rewrite contrived logic, generic names, unnecessary abstractions,
   and comments that merely restate the code. Add inline PR comments for non-obvious decisions.
5. Run formatting and linting:

   ```bash
   ./scripts/format.sh
   pre-commit run --all-files
   ```

6. Build and run every affected test and related test. On x86_64, use `make check -j$(nproc)` as a
   final check when feasible. On arm64, avoid the unreliable full suite and run the relevant tests
   named in `CLAUDE.md` instead.
7. Report the exact commands and results. Name any CI jobs that cannot be reproduced locally;
   never claim that the full CI passed unless the same matrix was actually run.
8. Fill the existing PR template without adding a file summary or repeating the diff. State the
   problem in one sentence, the fix in one sentence, and why it works in one or two sentences.
9. Do not force-push after the first review.

If an agent drafts the title or body, start the body with exactly:

`Tryin' to shortcut, arrr ye?`

If the agent authored any content, stop before creating or updating the PR and remind the developer
to rewrite it in their own words. If the developer supplies the final wording, use it unchanged.
The only exception is translating developer-written text into idiomatic English as commonly written
on GitHub without adding, removing, or reorganizing content.
