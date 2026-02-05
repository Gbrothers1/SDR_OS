---
name: phase-completion-docs
description: Use when a development phase (Phase 1, 2, 3, etc.) is complete and ready for PR — captures architecture diagrams, solutions, and static documentation into MkDocs before finishing the branch
---

# Phase Completion Documentation

## Overview

When a development phase finishes implementation and passes all tests, this skill captures the architecture artifacts before creating the PR. It ensures that solved designs, diagrams, and key decisions are preserved in the docs site.

**Announce at start:** "I'm using the phase-completion-docs skill to capture architecture artifacts for Phase N."

## When to Use

- After all implementation tasks for a phase are complete
- After tests pass (verified by `verify.sh` or `just verify`)
- Before running the `finishing-a-development-branch` skill
- When the user asks to "save diagrams" or "document the architecture"

## The Process

### Step 1: Identify Artifacts

Scan the conversation and implementation for:

1. **Service topology diagrams** — what services exist, how they connect, which networks/ports
2. **Memory layouts** — byte-level structures for IPC, protocols, wire formats
3. **Data flow diagrams** — how data moves through the pipeline end-to-end
4. **State machines** — protocol states, transitions, error handling
5. **CI/CD architecture** — jobs, runners, triggers, dependencies
6. **Container hierarchy** — base images, what gets installed, build order
7. **Key design decisions** — why this approach over alternatives

### Step 2: Create Solutions Page

Create `docs/solutions/phaseN-<short-title>.md` with:

```markdown
# Phase N: <Title> Solutions

**Phase:** N — <Title>
**Branch:** `feature/<branch-name>`
**Date:** <YYYY-MM-DD>

---

## 1. <Diagram Title>

<description of what the diagram shows>

```
<ASCII diagram>
```

### <Supporting Table or Details>

| Column | Column | Column |
|--------|--------|--------|
| ...    | ...    | ...    |

---

## 2. <Next Diagram>
...
```

Rules for diagrams:
- Use ASCII art (renders in any terminal or markdown viewer)
- Label every box with service name and port
- Show network boundaries explicitly
- Include a companion table with details the diagram can't show
- Number each diagram section

### Step 3: Update Solutions Index

Add the new phase to `docs/solutions/index.md`:

```markdown
| [Phase N](phaseN-<title>.md) | <Title> | Complete | <key diagrams list> |
```

### Step 4: Update MkDocs Navigation

Add the solutions page to `mkdocs.yml` nav under the Solutions section:

```yaml
- Solutions:
    - Overview: solutions/index.md
    - "Phase N: <Title>": solutions/phaseN-<title>.md
```

### Step 5: Update Dev Documentation

Check if any of these need updates based on the phase's work:

- `docs/dev-workflow.md` — new workflow patterns?
- `docs/testing.md` — new test categories or verification steps?
- `docs/architecture.md` — architecture changes?
- `docs/ci.md` — new CI jobs or pipeline stages?

Only update if the phase introduced changes relevant to these docs.

### Step 6: Create Phase Summary

Create or update `docs/phaseN-summary.md` with:

- What was built (deliverables table with status)
- Verification results (test counts, checks passed)
- Files created/modified
- What's next (Phase N+1 preview)

Add to mkdocs.yml nav.

### Step 7: Verify Docs Build

```bash
# Check that mkdocs can build without errors
pip install mkdocs-material mkdocs-git-revision-date-localized-plugin mike 2>/dev/null
mkdocs build --strict 2>&1 || echo "mkdocs not installed — verify nav YAML manually"
```

If mkdocs isn't available, manually verify:
- All files referenced in `mkdocs.yml` nav exist
- No broken cross-references between docs
- All ASCII diagrams render correctly in a markdown viewer

## Checklist

- [ ] Solutions page created at `docs/solutions/phaseN-*.md`
- [ ] All architecture diagrams captured as labeled ASCII art
- [ ] Companion tables included for each diagram
- [ ] Solutions index updated
- [ ] MkDocs nav updated
- [ ] Phase summary page created/updated
- [ ] Dev docs updated if needed (workflow, testing, architecture, CI)
- [ ] Docs build verified (or nav YAML verified manually)

## Integration

**Called before:** `finishing-a-development-branch`
**Called after:** All implementation tasks complete and tests pass
**Pairs with:** `executing-plans` (Step 5, after all batches)

## Common Mistakes

| Mistake | Fix |
|---------|-----|
| Forgetting memory layouts | Check for any IPC, wire format, or protocol structures |
| Diagrams without labels | Every box needs a name and port/role |
| Missing network boundaries | Draw explicit boundary lines between networks |
| No companion tables | ASCII can't show everything — add detail tables |
| Stale solutions index | Always update the phase table in index.md |
