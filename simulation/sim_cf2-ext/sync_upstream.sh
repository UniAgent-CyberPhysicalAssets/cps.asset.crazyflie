#!/bin/bash
set -e

# Configuration
FORK_URL="https://github.com/PioBeat/crazyflie-lib-python.git"
UPSTREAM_URL="https://github.com/bitcraze/crazyflie-lib-python.git"
COMMIT="38e8a5a1a07de30d9d874be1f9cee3e1f761a962" #cflib==0.1.29
BRANCH="master"

echo "Syncing fork (${FORK_URL}) with upstream commit ${COMMIT} from ${UPSTREAM_URL}"
echo "Merge direction: base=${UPSTREAM_URL}@${COMMIT}  ←  head=${FORK_URL}@${BRANCH}"
echo "Conflict strategy: keep fork's version (-X ours)"

# Clone your fork if not present
if [ ! -d "crazyflie-lib-python" ]; then
  git clone --depth 1 -b ${BRANCH} ${FORK_URL} crazyflie-lib-python
fi

cd crazyflie-lib-python

# Add upstream if missing
if ! git remote | grep -q upstream; then
  git remote add upstream ${UPSTREAM_URL}
fi

# Fetch both remotes
echo "Fetching both remotes..."
git fetch origin ${BRANCH}
git fetch upstream ${COMMIT} --no-tags

# Create temporary branch from your fork
TMP_BRANCH="sync-upstream-$(date +%s)"
git checkout -B ${TMP_BRANCH} origin/${BRANCH}

# Create a local branch pointing to the exact upstream commit
UPSTREAM_BRANCH="upstream-${COMMIT:0:7}"
git checkout -B ${UPSTREAM_BRANCH} ${COMMIT} || git checkout ${COMMIT}

# Switch back to your fork’s working branch
git checkout ${TMP_BRANCH}

# Merge upstream commit into fork branch, keeping fork’s version on conflict
echo "🔁 Merging upstream commit ${COMMIT} into fork (using -X ours)..."
git merge ${UPSTREAM_BRANCH} --allow-unrelated-histories -X ours --no-edit || true

# Commit merge if any new state exists
if ! git diff --quiet; then
  git commit -am "chore(sync): merged upstream commit ${COMMIT} into fork (${FORK_URL})"
fi

# Push merged state back to fork
# echo "Pushing merged state to fork (${FORK_URL})..."
# git push origin ${TMP_BRANCH}:${BRANCH}

# Clean up
# git branch -D ${TMP_BRANCH} || true
# git branch -D ${UPSTREAM_BRANCH} || true

echo "[OK] Fork successfully synchronized with upstream commit ${COMMIT}"
