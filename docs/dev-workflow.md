# Development Workflow

## Git Strategy

SDR_OS uses a **worktree-based feature branch** workflow. Each phase of development gets an isolated worktree so you can work on features without disrupting the main checkout.

### Branch Layout

```
main                          ← stable, deployable
  └── feature/phase1-cuda-docker-pipeline   ← Phase 1 work
  └── feature/phase2-rust-transport         ← Phase 2 (future)
  └── feature/phase3-...                    ← Phase 3 (future)
```

### Why Worktrees?

Git worktrees let you check out multiple branches simultaneously in separate directories. This means:

- **No stashing** — you never lose work switching branches
- **Parallel work** — run the dev server on `main` while building features in a worktree
- **Clean isolation** — each worktree has its own working tree, but shares the same `.git` history
- **Easy cleanup** — remove the directory when the branch merges

## Step-by-Step: Feature Development

### 1. Create a Worktree

```bash
# From the main repo root
cd /home/ethan/SDR_OS

# Create worktree for a new feature
git worktree add .worktrees/phase2 -b feature/phase2-rust-transport

# List all worktrees
git worktree list
```

Output:
```
/home/ethan/SDR_OS                          abc1234 [main]
/home/ethan/SDR_OS/.worktrees/phase1        def5678 [feature/phase1-cuda-docker-pipeline]
/home/ethan/SDR_OS/.worktrees/phase2        ghi9012 [feature/phase2-rust-transport]
```

### 2. Work in the Worktree

```bash
cd .worktrees/phase2

# Normal git commands work here
git status
git add <files>
git commit -m "Add transport server scaffold"
```

### 3. Verify Before Committing

Always run the verification script before committing:

```bash
./scripts/verify.sh          # Fast CPU-only checks (~2 sec)
./scripts/verify.sh --gpu    # Include Docker builds (~2 min)
./scripts/verify.sh --all    # Full check with GPU (~5 min)
```

Or use the justfile:

```bash
just verify                  # Same as verify.sh
just verify-gpu              # Same as verify.sh --gpu
just verify-all              # Same as verify.sh --all
```

### 4. Push and Create PR

```bash
# Push the feature branch
git push -u origin feature/phase2-rust-transport

# Create a pull request
gh pr create --title "Phase 2: Rust transport server" --body "## Summary
- Scaffold Rust crate with tokio + axum
- SHM reader with lap detection
- WebSocket frame fanout

## Test Plan
- [ ] Unit tests pass
- [ ] Integration test with genesis-sim
"
```

### 5. Merge via PR

After CI passes and review is complete:

```bash
# Merge on GitHub (recommended — preserves PR history)
gh pr merge <PR-number> --merge

# Or merge locally
cd /home/ethan/SDR_OS          # Back to main checkout
git checkout main
git pull
git merge feature/phase2-rust-transport
git push
```

### 6. Clean Up

```bash
# Remove the worktree
git worktree remove .worktrees/phase2

# Delete the remote branch (GitHub does this on PR merge)
git push origin --delete feature/phase2-rust-transport

# Prune stale worktree refs
git worktree prune
```

## Commit Guidelines

### Message Format

```
<type>: <short description>

<optional body — what and why, not how>

Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>
```

### Types

| Type | When |
|------|------|
| `Add` | New feature or file |
| `Update` | Enhancement to existing feature |
| `Fix` | Bug fix |
| `Refactor` | Code restructure, no behavior change |
| `Docs` | Documentation only |
| `CI` | CI/CD pipeline changes |
| `Test` | Test additions or fixes |

### Examples

```
Add SHM ringbuffer with atomic commit protocol

Implements zero-copy frame transport for the video pipeline.
32-byte headers, CRC validation, lap detection, latest-wins drop policy.

Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>
```

## CI Integration

Every push and PR triggers GitHub Actions automatically:

```
Push to branch → CI runs → Results on PR page
                              ├── validate-compose ✅
                              ├── lint-and-unit ✅
                              ├── ros-jazzy-build ✅
                              ├── docs-build ✅
                              └── cuda-smoke (needs GPU runner)
```

See [GitHub Actions Guide](github-actions.md) for details on how CI works.

## Worktree Directory Convention

All worktrees live under `.worktrees/` in the repo root:

```
SDR_OS/
├── .worktrees/           ← All feature worktrees (gitignored)
│   ├── phase1/           ← Phase 1: CUDA Docker + Pipeline
│   ├── phase2/           ← Phase 2: Rust Transport (future)
│   └── ...
├── .git/                 ← Shared git database
├── src/                  ← Main branch working tree
└── ...
```

The `.worktrees/` directory is in `.gitignore` so worktree checkouts are never committed.

## Merge Playbook

This is the complete playbook for getting a worktree branch merged into main. Follow it every time.

### The golden rule

**Never commit directly to main.** Always work in a worktree branch, PR it, and merge on GitHub. Direct commits to main cause conflicts when worktree branches try to merge.

### The full flow

```
Step 1: Verify          "Does my code work?"
Step 2: Push            "Get my branch on GitHub"
Step 3: PR              "Ask to merge"
Step 4: CI              "Wait for green checks"
Step 5: Merge           "One click / one command"
Step 6: Sync            "Pull main so it's up to date"
Step 7: Clean up        "Remove the worktree"
```

### Step 1 — Verify

```bash
cd .worktrees/phase2/
./scripts/verify.sh             # CPU checks (~2s)
```

If tests fail, fix them. Never merge broken code.

### Step 2 — Push your branch

```bash
git push -u origin feature/phase2-rust-transport
```

The `-u` sets up tracking so future `git push` just works.

### Step 3 — Create the PR

```bash
gh pr create --title "Phase 2: Rust transport server" --body "## Summary
- SHM reader in Rust
- WebSocket fanout
- NATS telemetry relay

## Test plan
- [ ] Unit tests pass
- [ ] Docker build succeeds"
```

### Step 4 — Wait for CI

```bash
gh pr checks <PR_NUMBER>        # e.g., gh pr checks 2
```

Run this until CPU jobs show `pass`. GPU jobs (`cuda-smoke`, etc.) stay `pending` until self-hosted runners are configured — that's expected.

If CI fails: fix, commit, push — CI re-runs automatically.

### Step 5 — Check for conflicts, then merge

Before merging, check if main and your branch touch the same files:

```bash
git fetch origin main
git diff origin/main...HEAD --name-only
```

**No overlapping files?** Safe to merge:

```bash
gh pr merge <PR_NUMBER> --merge
```

**Conflicts?** Rebase first (see [Handling Conflicts](#handling-conflicts) below).

### Step 6 — Sync local main

```bash
# From the main repo (NOT the worktree)
cd ~/SDR_OS
git pull origin main
```

### Step 7 — Clean up

```bash
git worktree remove .worktrees/phase2
git push origin --delete feature/phase2-rust-transport
git worktree prune
```

### Handling conflicts

Conflicts happen when main changed the same files as your branch. Here's the procedure:

```bash
# 1. Fetch latest main
git fetch origin main

# 2. Rebase your branch onto main
git rebase origin/main

# 3. Git will stop at each conflict. For each one:
#    - Open the file, find the <<<<<<< markers
#    - Pick the right version (or combine both)
#    - Save the file

# 4. Mark resolved and continue
git add <fixed-file>
git rebase --continue

# 5. Repeat steps 3-4 for each conflicted commit

# 6. Force push the rebased branch (safe with --force-with-lease)
git push --force-with-lease

# 7. Now merge the PR
gh pr merge <PR_NUMBER> --merge
```

**What conflict markers look like:**

```
<<<<<<< HEAD
This is what's on main (the base)
=======
This is what's on your branch (the incoming change)
>>>>>>> your-commit-message
```

You choose which version to keep (or merge both), then delete the marker lines.

### What NOT to do

| Don't | Why | Do this instead |
|-------|-----|-----------------|
| `git commit` on main | Creates conflicts for every open worktree branch | Always commit in a worktree |
| `git merge` locally | No CI checks, no paper trail | Use `gh pr merge` |
| `git push --force` | Can overwrite someone else's work | Use `--force-with-lease` |
| Merge with failing CI | Broken code on main | Fix CI first, then merge |
| `git reset --hard` when scared | You'll lose work permanently | Ask for help or `git stash` |
| Delete a branch before merging | Commits are gone | Merge or PR first, then delete |

### Emergency: "I accidentally committed on main"

If you committed on main instead of a feature branch:

```bash
# 1. Create a branch from your current position (saves your work)
git branch feature/oops

# 2. Reset main back to where origin/main is
git reset --hard origin/main

# 3. Create a worktree from the saved branch
git worktree add .worktrees/oops feature/oops

# 4. Push and PR as normal
cd .worktrees/oops
git push -u origin feature/oops
gh pr create --title "..." --body "..."
```

Your commits are safe on the new branch. Main is clean again.

## Quick Reference

| Task | Command |
|------|---------|
| List worktrees | `git worktree list` |
| Create worktree | `git worktree add .worktrees/<name> -b feature/<branch>` |
| Remove worktree | `git worktree remove .worktrees/<name>` |
| Run tests | `just verify` or `./scripts/verify.sh` |
| Push branch | `git push -u origin <branch>` |
| Create PR | `gh pr create --title "..." --body "..."` |
| Check CI | `gh pr checks <number>` |
| Preview conflicts | `git diff origin/main...HEAD --name-only` |
| Merge PR | `gh pr merge <number> --merge` |
| Sync main | `git pull origin main` (from main repo, not worktree) |
| Rebase on main | `git fetch origin main && git rebase origin/main` |
| Force push after rebase | `git push --force-with-lease` |
| Clean up | `git worktree remove .worktrees/<name> && git worktree prune` |
