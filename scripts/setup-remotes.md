# Setup Dual-Remote Git Configuration

Use this prompt with Claude Code on a new machine to replicate the git remote setup for SDR_OS.

---

**Paste this into Claude Code:**

```
Set up the git remotes for this SDR_OS repository as follows:

1. Ensure `origin` points to GitHub:
   git remote set-url origin https://github.com/Gbrothers1/SDR_OS.git

2. Add a `gitea` remote pointing to the private Gitea server:
   git remote add gitea git@git.ethangordon.io:h1ght0w3r/SDR_OS.git

3. Add the Gitea server's SSH host key to known_hosts:
   ssh-keyscan -p 22 git.ethangordon.io >> ~/.ssh/known_hosts

4. Verify the SSH key at ~/.ssh/id_ed25519 is authorized on the Gitea account h1ght0w3r.
   If not, print the public key and prompt me to add it at:
   http://git.ethangordon.io/user/settings/keys

5. Test the Gitea connection with: ssh -T git@git.ethangordon.io

6. Confirm both remotes with: git remote -v

Remote layout when done:
  origin  https://github.com/Gbrothers1/SDR_OS.git  (GitHub, code-only)
  gitea   git@git.ethangordon.io:h1ght0w3r/SDR_OS.git  (Gitea, full mirror)

Notes:
- rl/checkpoints/go2-jump/ and rl/checkpoints/go2-jump-power/ are gitignored from GitHub.
  To restore them from Gitea: git fetch gitea && git checkout gitea/checkpoints -- rl/checkpoints/
- Push to GitHub:  git push origin main
- Push to Gitea:   git push --all gitea && git push --tags gitea
```
