#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/github_bootstrap.sh [options]

Options:
  --repo <url>            Git remote URL (ssh or https), e.g. git@github.com:org/repo.git
  --branch <name>         Branch to checkout/push (default: current branch or test)
  --key-path <path>       SSH private key path (default: ~/.ssh/id_ed25519_github_bioxp)
  --title <title>         GitHub SSH key title for API upload
  --github-user <user>    GitHub username (required for --upload-key)
  --github-token <token>  GitHub token with admin:public_key scope (or set GITHUB_TOKEN env)
  --upload-key            Upload generated public key to GitHub via API
  --push                  Push selected branch to origin
  --test-auth             Run ssh auth test against github.com
  -h, --help              Show this help

Examples:
  scripts/github_bootstrap.sh --repo git@github.com:ORG/REPO.git --branch test --push

  scripts/github_bootstrap.sh --upload-key --github-user YOURUSER --github-token YOURTOKEN
USAGE
}

REPO_URL=""
BRANCH=""
KEY_PATH="${HOME}/.ssh/id_ed25519_github_bioxp"
TITLE=""
GITHUB_USER=""
GITHUB_TOKEN="${GITHUB_TOKEN:-}"
UPLOAD_KEY=0
PUSH_BRANCH=0
TEST_AUTH=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --repo)
      REPO_URL="${2:-}"
      shift 2
      ;;
    --branch)
      BRANCH="${2:-}"
      shift 2
      ;;
    --key-path)
      KEY_PATH="${2:-}"
      shift 2
      ;;
    --title)
      TITLE="${2:-}"
      shift 2
      ;;
    --github-user)
      GITHUB_USER="${2:-}"
      shift 2
      ;;
    --github-token)
      GITHUB_TOKEN="${2:-}"
      shift 2
      ;;
    --upload-key)
      UPLOAD_KEY=1
      shift
      ;;
    --push)
      PUSH_BRANCH=1
      shift
      ;;
    --test-auth)
      TEST_AUTH=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Error: run this script from inside a git repository." >&2
  exit 1
fi

if [[ -z "${BRANCH}" ]]; then
  BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo test)"
fi

mkdir -p "$(dirname "${KEY_PATH}")"

if [[ ! -f "${KEY_PATH}" ]]; then
  key_comment="${USER}@$(hostname)-bioxp"
  echo "Generating SSH key: ${KEY_PATH}"
  ssh-keygen -t ed25519 -C "${key_comment}" -f "${KEY_PATH}" -N ""
else
  echo "SSH key exists: ${KEY_PATH}"
fi

if [[ -z "${SSH_AUTH_SOCK:-}" ]]; then
  set +e
  agent_out="$(ssh-agent -s 2>/dev/null)"
  agent_rc=$?
  set -e
  if [[ ${agent_rc} -eq 0 ]]; then
    eval "${agent_out}" >/dev/null
  else
    echo "Warning: could not start ssh-agent in this environment; continuing without auto-add."
  fi
fi

if [[ -n "${SSH_AUTH_SOCK:-}" ]]; then
  set +e
  ssh-add "${KEY_PATH}" >/dev/null
  add_rc=$?
  set -e
  if [[ ${add_rc} -ne 0 ]]; then
    echo "Warning: ssh-add failed; you can add the key manually in your shell."
  fi
fi

PUB_KEY_PATH="${KEY_PATH}.pub"
if [[ ! -f "${PUB_KEY_PATH}" ]]; then
  echo "Error: missing public key ${PUB_KEY_PATH}" >&2
  exit 1
fi

PUB_KEY="$(cat "${PUB_KEY_PATH}")"

echo ""
echo "Public key (${PUB_KEY_PATH}):"
echo "${PUB_KEY}"
echo ""

if [[ ${UPLOAD_KEY} -eq 1 ]]; then
  if [[ -z "${GITHUB_USER}" ]]; then
    echo "Error: --github-user is required with --upload-key" >&2
    exit 1
  fi
  if [[ -z "${GITHUB_TOKEN}" ]]; then
    echo "Error: --github-token or GITHUB_TOKEN env is required with --upload-key" >&2
    exit 1
  fi
  if [[ -z "${TITLE}" ]]; then
    TITLE="BioXP-$(hostname)-$(date -u +%Y%m%dT%H%M%SZ)"
  fi

  echo "Uploading SSH key to GitHub account ${GITHUB_USER} ..."
  api_resp="$(curl -sS -w '\n%{http_code}' \
    -X POST \
    -H "Accept: application/vnd.github+json" \
    -H "Authorization: Bearer ${GITHUB_TOKEN}" \
    https://api.github.com/user/keys \
    -d "{\"title\":\"${TITLE}\",\"key\":\"${PUB_KEY}\"}")"

  http_code="$(echo "${api_resp}" | tail -n1)"
  body="$(echo "${api_resp}" | sed '$d')"

  if [[ "${http_code}" == "201" ]]; then
    echo "GitHub key upload successful."
  else
    echo "GitHub key upload failed (HTTP ${http_code}). Response:"
    echo "${body}"
    exit 1
  fi
fi

if [[ -n "${REPO_URL}" ]]; then
  if git remote get-url origin >/dev/null 2>&1; then
    echo "Updating origin -> ${REPO_URL}"
    git remote set-url origin "${REPO_URL}"
  else
    echo "Adding origin -> ${REPO_URL}"
    git remote add origin "${REPO_URL}"
  fi
fi

if git show-ref --verify --quiet "refs/heads/${BRANCH}"; then
  git checkout "${BRANCH}" >/dev/null
else
  git checkout -b "${BRANCH}" >/dev/null
fi

echo "Current branch: $(git rev-parse --abbrev-ref HEAD)"

git status --short

if [[ ${TEST_AUTH} -eq 1 ]]; then
  echo ""
  echo "Testing GitHub SSH auth (expected success message may return non-zero code)..."
  set +e
  ssh -T git@github.com
  ssh_rc=$?
  set -e
  echo "ssh test exit code: ${ssh_rc}"
fi

if [[ ${PUSH_BRANCH} -eq 1 ]]; then
  if ! git remote get-url origin >/dev/null 2>&1; then
    echo "Error: origin remote is not configured. Use --repo <url>." >&2
    exit 1
  fi
  echo "Pushing branch ${BRANCH} to origin ..."
  git push -u origin "${BRANCH}"
fi

echo "Done."
