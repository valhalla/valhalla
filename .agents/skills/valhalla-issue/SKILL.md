---
name: valhalla-issue
description: >-
  Prepare a Valhalla GitHub issue. Use when searching, reproducing, drafting, editing, or creating
  a bug report or feature request for valhalla/valhalla. This repository workflow takes precedence
  over personal issue workflows.
---

# Valhalla issue

1. Read `README.md`'s "Generative AI usage" section and `CONTRIBUTING.md`.
2. Search open and closed issues for duplicates. Link a matching or closely related issue.
3. For a bug, identify the remote pointing to `valhalla/valhalla`, fetch `master`, and reproduce
   against it. If it cannot be reproduced, say so; do not invent a cause.
4. Keep only the minimal reproduction, expected and actual behavior, and relevant version or
   environment details.
5. Use plain, concise English. A simple bug normally needs no headings: state the problem, show
   the reproduction, then state expected and actual behavior.
6. Do not include background, likely causes, proposed fixes, validation plans, alternatives,
   performance results, or product and company context unless a maintainer asks for them. For a
   feature request, describe the need and desired behavior without designing the implementation.

If an agent drafts the title or body, start the body with exactly:

`Tryin' to shortcut, arrr ye?`

If the agent wrote any prose, stop before submission and remind the developer to rewrite it in
their own words. If the developer supplies the final wording, use it unchanged.
