---
name: valhalla-issue
description: Prepare a Valhalla GitHub issue. Use when searching, reproducing, drafting, editing, or creating a bug report or feature request for valhalla/valhalla. This repository workflow takes precedence over personal issue workflows.
---

# Valhalla issue

1. Read `README.md`'s "Generative AI usage" section and `CONTRIBUTING.md`.
2. Read `docs/learnings.md` first. Use its prior decisions to narrow the GitHub search and carry settled project context into the issue.
3. Search open and closed issues and Discussions for duplicates and prior decisions. Inspect merged or closed pull requests when the same subsystem, path, or symbol appears. Link the most relevant source so the next participant starts with the same context.
4. For a bug, identify the remote pointing to `valhalla/valhalla`, fetch `master`, and reproduce against it. If it cannot be reproduced, say so; do not invent a cause.
5. Treat the initial issue as the start of a conversation. It needs enough verified context to discuss, not a comprehensive investigation. Assume maintainers know the project background: link established context instead of restating it, then add only the new observation, reproduction, or question; expand when the discussion exposes a gap.
6. Keep only the minimal reproduction, expected and actual behavior, and relevant version or environment details.
7. Use plain, concise English and keep each Markdown paragraph or list item on one physical line; let GitHub soft-wrap the body. A simple bug normally needs no headings: state the problem, show the reproduction, then state expected and actual behavior.
8. Keep a bug report centered on observable behavior. Add likely causes, proposed fixes, validation plans, alternatives, or performance results when a maintainer asks for them. For a feature request, describe the need and desired behavior while leaving implementation design to the follow-up discussion.

If an agent drafts the title or body, start the body with exactly:

`Tryin' to shortcut, arrr ye?`

If the agent authored any content, stop before submission and remind the developer to rewrite it in their own words. If the developer supplies the final wording, use it unchanged. The only exception is translating text the developer wrote in another language. Translate for meaning, not style: preserve their voice, order, level of detail, and non-idiomatic wording that remains understandable. Do not polish, summarize, expand, or add transitions. Ask about ambiguous wording instead of guessing.
