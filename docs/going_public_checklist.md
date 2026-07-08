# Going-public checklist

Steps to open this repository to outside contributors. The ones marked
**owner-only** require repo-admin or shell access with the credentials and
cannot be done from a pull request. Do them in order — the history rewrite
must happen **before** the repo is made public.

## 1. Scrub secrets from git history — ✅ DONE 2026-07-08

Executed with git-filter-repo on both repositories (this repo and
`agv-greenhouse-sim`): the leaked password, site LAN IPs, customer
identity, and personal home paths are purged from every revision; tip
trees verified byte-identical. **Still required: rotate the exposed
Jetson SSH/VNC password** — treat it as compromised regardless of the
rewrite — and anyone with an old clone must re-clone.

<details><summary>Original instructions (for reference)</summary>

The working tree no longer contains the leaked SSH password or the home-lab
topology, but **git history still does**. Making the repo public exposes the
full history. A file deletion in a later commit does not remove it from
earlier commits.

```bash
# Install git-filter-repo (https://github.com/newren/git-filter-repo)
pip install git-filter-repo

# From a FRESH clone (filter-repo refuses to run on a repo with a remote by
# default and rewrites all history — never do this on your only copy):
git clone --mirror git@github.com:AndresIslas99/agv-greenhouse.git
cd agv-greenhouse.git

# Redact the leaked strings everywhere they appear in history.
# Put each secret on its own line in replacements.txt, e.g.:
#   <LEAKED_PASSWORD>==>REDACTED
#   orza==>sim-user
cat > /tmp/replacements.txt <<'EOF'
literal:<LEAKED_PASSWORD>==>REDACTED
EOF
git filter-repo --replace-text /tmp/replacements.txt

# Review, then force-push the rewritten history:
git push --force
```

Then, regardless of the rewrite:

- **Rotate the exposed SSH credential** on the sim host and treat it as
  compromised (it may already be scraped).
- **Scrub the sibling `agv-greenhouse-sim` repo** the same way — this repo's
  `docs/validation/RUNBOOK_lan_hil.md` cross-references it and it likely
  carries the same credentials and LAN topology.
- All collaborators must re-clone after the force-push (old clones keep the
  secrets).

</details>

## 2. Land the community-readiness work on `main` — ✅ DONE (PR #5 merged; v0.1.0 tagged) — owner-only

PR #5 carries the green CI, the fixes, and this checklist. `main` is still
the old red-CI state until it merges. Merge PR #5, then tag the release:

```bash
git checkout main && git pull
git tag -a v0.1.0 -m "First public release"
git push origin v0.1.0
```

Move the `Unreleased` entries in `CHANGELOG.md` under `v0.1.0`.

## 3. Branch protection on `main` — owner-only (Settings → Branches)

History shows merges landing with red CI; with outside contributors the gate
must be mechanical, not habitual. Require:

- Pull request before merging (at least 1 approving review).
- Status checks to pass: **`spec-verification`**, **`build-and-test`**,
  **`typescript-build`**, **`simulation`**.
- Branches up to date before merging.
- No force-pushes / no deletion of `main`.

## 4. Rename the repository to `navgreen` — owner-only, optional but decided

Settings → General → Repository name → `navgreen`. GitHub redirects the old
`agv-greenhouse` URLs (clones, links, badges keep working), but after
renaming run this once so first-party links don't rely on redirects:

```bash
grep -rl 'AndresIslas99/agv-greenhouse' --include='*.md' --include='*.yml' \
  --include='*.yaml' . | xargs sed -i 's#AndresIslas99/agv-greenhouse#AndresIslas99/navgreen#g'
# also update site_url in mkdocs.yml (…github.io/navgreen/)
```

Package names keep the `agv_` prefix — NavGreen is the project brand, like
Nav2's `nav2_*` packages.

## 5. Enable the documentation site — owner-only

The `Docs` workflow (`.github/workflows/docs.yaml`) builds the MkDocs Material
site on every PR and deploys the `gh-pages` branch on pushes to `main`. After
the first deploy: Settings → Pages → Source: **Deploy from a branch** →
branch `gh-pages` / root. The site lands at
`https://andresislas99.github.io/<repo-name>/`.

## 6. Repository metadata — owner-only (Settings → General, and the sidebar)

- **Description**: e.g. "NavGreen — autonomous navigation for greenhouse
  robots. ROS 2, spec-driven, dual-EKF localization, Nav2."
- **Website**: the GitHub Pages URL from step 5.
- **Topics**: `ros2`, `robotics`, `agv`, `nav2`, `autonomous-robots`,
  `greenhouse`, `cpp`, `differential-drive`.
- **Social preview image** (Settings → General → Social preview — the
  wordmark at `docs/assets/navgreen-logo.svg` rendered on a light card works).
- Enable **Issues** and **Discussions**; disable **Wiki** unless you'll use it.

## 7. Make it public — owner-only

Settings → General → Danger Zone → Change visibility → Public. Do this only
after steps 1–3.

## 8. First-day-of-public polish

- Seeded issues are filed from the review roadmap (labels
  `good-first-issue`, `help-wanted`, `hardware-required`) — triage and pin a
  couple of good first issues.
- Add a dashboard/HIL demo GIF to the README (the one thing text can't
  convey; biggest single lift to first impressions).
- Consider publishing `agv-greenhouse-sim` (scrubbed) so contributors can
  run the HIL loop without your hardware — today this is the main blocker to
  non-doc contributions.
