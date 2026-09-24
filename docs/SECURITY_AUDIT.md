# Public-tree credential audit

Audit baseline: `2eb40fd44b48baa31b31dcd1047ab89f8a128c3b` (2026-09-24 review).

The nonempty Bing Maps credential was removed from both current copies of
`truck_bringup/params/patio_mercedes.mvc` (the root copy and the copy under
`src/gps_truck_nav/`). Each now contains an empty `bing_api_key`. No credential
value is reproduced in this report. The default `gps_wpf_demo.mvc` already had
an empty key. Mapviz is optional; Gazebo, localization and Nav2 do not require it.

The initial read-only history scan covered all 20 commits reachable from the
fetched refs and 140 unique Git blobs. It found the credential-bearing Mapviz
blob in all 20 commits, under these historical paths:

- `src/truck_bringup/params/patio_mercedes.mvc`
- `truck_bringup/params/patio_mercedes.mvc`
- `src/gps_truck_nav/truck_bringup/params/patio_mercedes.mvc`

It also found `src/truck_bringup/bing_key.txt` in the initial commit
`43f74dc88c6f802344ada4470d34b1756f857efd`. Removing that file in a later commit
did not remove it from history.

The current-tree scan included credential assignments, token formats, embedded
URL authentication, private-key markers and sensitive filenames. Independent
ripgrep filename-only searches were also used. Values were withheld from all
output. Final scanner results and limitations are recorded in
[VALIDATION.md](VALIDATION.md). A scan is not proof that no unknown secret exists.

## HISTORY CLEANUP REQUIRED

The owner should revoke/rotate the exposed Bing credential in its provider
account. It is a map-service credential, not an email password. Rotation is the
effective response to copies that may already exist. These cleanup commits
remove it only from this branch's current tree; the default branch and old
commits remain exposed until the owner takes further action.

**No history was rewritten and no force-push was performed.** If the owner later
chooses to rewrite history, first coordinate with collaborators and keep an
access-controlled backup. The following optional procedure operates only on a
new local mirror and does not push anything. Install a recent `git-filter-repo`
that supports `--sensitive-data-removal` first.

```bash
git clone --mirror https://github.com/marcusvinicius178/gps_truck_nav.git gps_truck_nav-history-review.git
cp -a gps_truck_nav-history-review.git gps_truck_nav-history-backup.git
cat > mapviz-redactions.txt <<'EOF'
regex:(?m)^([ \t]*bing_api_key:)[^\r\n]*$==>\1 ""
EOF
cd gps_truck_nav-history-review.git
git filter-repo --sensitive-data-removal --replace-text ../mapviz-redactions.txt \
  --path src/truck_bringup/bing_key.txt --invert-paths
```

Inspect and rescan the rewritten mirror before considering a remote update.
The backup still contains the old credential and must remain private. Updating
remote history, dealing with cached views/forks and asking collaborators to
reclone are separate owner decisions. The cleanup branch does not perform them.

## Local Mapviz settings

Copy the installed Mapviz configuration to a file outside the repository, edit
the `bing_api_key` there and pass that path using `mapviz_config:=...`. Local
`*.local.mvc`, environment files and common credential filenames are ignored,
but `.gitignore` cannot protect a credential pasted into an already tracked file.
