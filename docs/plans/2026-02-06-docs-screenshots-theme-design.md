# Docs Screenshots, Theme & Versioning Design

**Goal:** Add automated screenshot capture, visual theme improvements, and version-aligned doc deployments to SDR_OS MkDocs.

**Date:** 2026-02-06

---

## 1. Overview

Four deliverables:

1. **Theme upgrade** — Dark slate + cyan accent, light/dark toggle, Mermaid diagrams, glightbox, custom logo slot
2. **Screenshot capture** — shot-scraper YAML manifest captures ~8 key UI states, images embedded in docs
3. **CI: auto-screenshots** — GitHub Actions workflow updates screenshots on PRs that touch UI files
4. **CI: versioned docs** — mike deploys versioned docs on tag push, `dev` alias on main push, version dropdown in header

---

## 2. Theme

### Color scheme

| Property | Value |
|----------|-------|
| Default scheme | `slate` (dark) |
| Primary | `cyan` |
| Accent | `light blue` |
| Toggle | Light ↔ Dark with Material icons |

### mkdocs.yml theme block

```yaml
theme:
  name: material
  palette:
    - scheme: slate
      primary: cyan
      accent: light blue
      toggle:
        icon: material/brightness-4
        name: Switch to light mode
    - scheme: default
      primary: cyan
      accent: light blue
      toggle:
        icon: material/brightness-7
        name: Switch to dark mode
  features:
    - navigation.tabs
    - navigation.top
    - navigation.sections
    - navigation.instant
    - content.code.copy
    - content.tabs.link
    - toc.integrate
    - header.autohide
  logo: assets/logo.png
  favicon: assets/favicon.png
  icon:
    repo: fontawesome/brands/github
```

### New plugins

```yaml
plugins:
  - search
  - glightbox:
      effect: zoom
      zoomable: true
      auto_caption: true
  - git-revision-date-localized
  - mike
```

### New markdown extensions

```yaml
markdown_extensions:
  - pymdownx.superfences:
      custom_fences:
        - name: mermaid
          class: mermaid
          format: !!python/name:pymdownx.superfences.fence_code_format
  - pymdownx.emoji:
      emoji_index: !!python/name:material.extensions.emoji.twemoji
      emoji_generator: !!python/name:material.extensions.emoji.to_svg
```

### New pip dependencies

```
mkdocs-glightbox
```

### Logo

Place logo at `docs/assets/logo.png` (and `docs/assets/favicon.png`). Material for MkDocs will display it in the header. Until a custom logo exists, use a Material icon:

```yaml
theme:
  icon:
    logo: material/robot
```

---

## 3. Screenshot Capture

### Tool: shot-scraper

```bash
pip install shot-scraper
shot-scraper install   # Downloads Playwright Chromium
```

### Manifest: `docs/shots.yml`

```yaml
# SDR_OS documentation screenshots
# Run: shot-scraper multi docs/shots.yml

- url: http://localhost:3000/
  output: docs/screenshots/app-dashboard.png
  width: 1280
  height: 720
  wait: 2000

- url: http://localhost:3000/
  output: docs/screenshots/sim-viewer.png
  width: 1280
  height: 720
  javascript: |
    // Switch to genesis mode if available
    const btn = document.querySelector('[data-mode="genesis"]');
    if (btn) btn.click();
  wait: 1000

- url: http://localhost:3000/
  output: docs/screenshots/robot-viewer.png
  width: 1280
  height: 720
  javascript: |
    const btn = document.querySelector('[data-mode="robot"]');
    if (btn) btn.click();
  wait: 1000

- url: http://localhost:3000/
  output: docs/screenshots/settings-modal.png
  width: 1280
  height: 720
  javascript: |
    const btn = document.querySelector('.settings-icon, [data-panel="settings"]');
    if (btn) btn.click();
  wait: 500

- url: http://localhost:3000/
  output: docs/screenshots/trust-strip.png
  width: 1280
  height: 100
  selector: ".trust-strip"

- url: http://localhost:3000/
  output: docs/screenshots/control-overlay.png
  width: 1280
  height: 720
  javascript: |
    // Trigger control overlay visibility
    const overlay = document.querySelector('.control-overlay');
    if (overlay) overlay.style.opacity = '1';
  wait: 500

- url: http://localhost:3000/
  output: docs/screenshots/telemetry-panel.png
  width: 800
  height: 600
  javascript: |
    const btn = document.querySelector('[data-panel="telemetry"]');
    if (btn) btn.click();
  wait: 500
  selector: ".telemetry-panel, .floating-panel"

- url: http://localhost:3000/
  output: docs/screenshots/training-dashboard.png
  width: 1000
  height: 600
  javascript: |
    const btn = document.querySelector('[data-panel="training"]');
    if (btn) btn.click();
  wait: 500
  selector: ".training-dashboard, .floating-panel"
```

### Usage in docs

```markdown
![Main dashboard](screenshots/app-dashboard.png)

![SimViewer with genesis mode](screenshots/sim-viewer.png)
```

Glightbox makes every image clickable for full-size zoom. `auto_caption: true` uses the alt text as the caption.

### Directory structure

```
docs/
├── assets/
│   ├── logo.png
│   └── favicon.png
├── screenshots/
│   ├── app-dashboard.png
│   ├── sim-viewer.png
│   ├── robot-viewer.png
│   ├── settings-modal.png
│   ├── trust-strip.png
│   ├── control-overlay.png
│   ├── telemetry-panel.png
│   └── training-dashboard.png
├── shots.yml
└── ...
```

---

## 4. CI: Auto-Update Screenshots

### Workflow: `.github/workflows/update-screenshots.yml`

**Trigger:** PRs that modify `src/client/**`, `*.jsx`, `*.css`, `webpack.config.js`

**Steps:**
1. `dorny/paths-filter@v3` checks if UI files changed
2. `npm ci && npm run build`
3. `node server.js &` + `npx wait-on http://localhost:3000`
4. `pip install shot-scraper && shot-scraper install`
5. `shot-scraper multi docs/shots.yml`
6. `stefanzweifel/git-auto-commit-action@v5` commits updated screenshots to PR branch with `[skip ci]`

**Filter paths:**
```yaml
filters: |
  ui:
    - 'src/client/**'
    - '*.jsx'
    - '*.css'
    - 'webpack.config.js'
    - 'src/client/styles/**'
    - 'src/client/components/**'
```

**Key behaviors:**
- Only runs when UI code changes (no wasted CI minutes on backend PRs)
- Commits use `[skip ci]` to prevent infinite loops
- `GITHUB_TOKEN` auth (no PAT needed; screenshots commit won't re-trigger workflows)

---

## 5. CI: Versioned Doc Deployment

### Workflow: `.github/workflows/deploy-docs.yml`

**Two triggers:**

| Trigger | Action | Result |
|---------|--------|--------|
| Push to `main` | `mike deploy --push dev latest` | `dev` alias updated with latest docs |
| Tag push (`v*`) | `mike deploy --push <version> stable` | Versioned deploy (e.g., `0.2.0`) |

**Version extraction from tag:**
```bash
VERSION=${GITHUB_REF#refs/tags/v}   # v0.2.0 → 0.2.0
```

**mike commands:**
```bash
mike deploy --push --update-aliases 0.2.0 stable
mike set-default --push latest
```

### Version dropdown config

```yaml
extra:
  version:
    provider: mike
    default: stable
    alias: true
```

This shows a version dropdown in the header with entries like:
- `dev` (latest main)
- `0.2.0` (stable)
- `0.1.0`

### Version alignment

| Source | Version |
|--------|---------|
| `pyproject.toml` | `0.2.0` (canonical) |
| Git tag | `v0.2.0` |
| mike deploy | `0.2.0` with alias `stable` |
| Docs header | Shows version from dropdown |

---

## 6. Dependencies Summary

### New pip packages

| Package | Purpose |
|---------|---------|
| `mkdocs-glightbox` | Image lightbox/zoom |
| `shot-scraper` | Screenshot capture (wraps Playwright) |
| `pillow` | Image processing (already in venv) |

### New GitHub Actions

| Action | Purpose |
|--------|---------|
| `dorny/paths-filter@v3` | Filter PRs by changed file paths |
| `stefanzweifel/git-auto-commit-action@v5` | Commit screenshots back to PR |

### New files

| File | Purpose |
|------|---------|
| `docs/shots.yml` | Screenshot manifest |
| `docs/assets/logo.png` | Docs logo (placeholder until custom) |
| `docs/screenshots/*.png` | Captured screenshots |
| `.github/workflows/update-screenshots.yml` | Auto-screenshot CI |
| `.github/workflows/deploy-docs.yml` | Versioned doc deployment |

---

## 7. Implementation Order

1. Theme upgrade (mkdocs.yml + plugins)
2. Logo placeholder + assets directory
3. Screenshot manifest + initial capture
4. Embed screenshots in key doc pages
5. `update-screenshots.yml` workflow
6. `deploy-docs.yml` workflow
7. Version dropdown config
8. Initial `mike deploy` for v0.2.0
