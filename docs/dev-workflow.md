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

## Quick Reference

| Task | Command |
|------|---------|
| List worktrees | `git worktree list` |
| Create worktree | `git worktree add .worktrees/<name> -b feature/<branch>` |
| Remove worktree | `git worktree remove .worktrees/<name>` |
| Run tests | `just verify` or `./scripts/verify.sh` |
| Push branch | `git push -u origin <branch>` |
| Create PR | `gh pr create --title "..." --body "..."` |
| Merge PR | `gh pr merge <number> --merge` |
| Clean up | `git worktree remove .worktrees/<name> && git worktree prune` |
