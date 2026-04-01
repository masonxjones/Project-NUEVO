"""
Authentication helpers for NUEVO Bridge.

- Users stored in a JSON file. By default this is backend/users.json, but it can
  be overridden with NUEVO_USERS_FILE for Docker/runtime deployments.
- Passwords hashed with bcrypt via passlib.
- JWT tokens signed with a random secret (per-process).
  Users must re-login after RPi reboot — this is intentional and acceptable.
- Default accounts (admin / user) are created automatically on first boot.
"""
import json
import os
import secrets
from pathlib import Path

import bcrypt
import jwt
from datetime import datetime, timedelta, timezone
from fastapi import HTTPException

# ─── Paths ────────────────────────────────────────────────────────────────────
# users.json defaults to backend/users.json (parent of the nuevo_bridge package
# dir), but Docker deployments should override it with NUEVO_USERS_FILE so auth
# data lives outside the read-only source tree.
DEFAULT_USERS_FILE = Path(__file__).parent.parent / "users.json"
USERS_FILE = Path(os.getenv("NUEVO_USERS_FILE", str(DEFAULT_USERS_FILE)))

# ─── Password hashing (bcrypt directly — avoids passlib compatibility issues) ─

def verify_password(plain: str, hashed: str) -> bool:
    return bcrypt.checkpw(plain.encode("utf-8"), hashed.encode("utf-8"))


def hash_password(plain: str) -> str:
    return bcrypt.hashpw(plain.encode("utf-8"), bcrypt.gensalt()).decode("utf-8")


# ─── JWT ──────────────────────────────────────────────────────────────────────
# Random secret per process start — users re-login after RPi reboot.
JWT_SECRET = secrets.token_hex(32)
JWT_ALGORITHM = "HS256"
JWT_EXPIRE_DAYS = 30


def create_token(username: str, role: str) -> str:
    payload = {
        "sub": username,
        "role": role,
        "exp": datetime.now(timezone.utc) + timedelta(days=JWT_EXPIRE_DAYS),
    }
    return jwt.encode(payload, JWT_SECRET, algorithm=JWT_ALGORITHM)


def decode_token(token: str) -> dict:
    """Return JWT payload dict (keys: sub, role). Raises HTTPException on failure."""
    try:
        return jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token expired — please log in again")
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="Invalid token")


# ─── Default users ────────────────────────────────────────────────────────────
# Passwords are pre-hashed (bcrypt) — plaintext is never stored in source.
# To regenerate: python3 -c "import bcrypt; print(bcrypt.hashpw(b'<pw>', bcrypt.gensalt()).decode())"
_DEFAULT_USERS = {
    "admin": {
        "role": "admin",
        "password_hash": "$2b$12$lIim4XUxmvnT777xRMuCee9rZerLu0QHkbbxvZ72bD/8hmsnV7Mhu",
    },
    "user": {
        "role": "user",
        "password_hash": "$2b$12$ZiuN.Wowr1dJQohSctUIz./FrTGrxEoPPwSSgluqsjtlrao94MHWC",
    },
}

# The "admin" username cannot be deleted (but CAN be renamed if desired).
PROTECTED_USERNAMES = {"admin"}  # usernames that cannot be deleted


def _create_default_users() -> dict:
    """Return default users dict (called once on first boot to seed users.json)."""
    print("[Auth] Creating default users.json…")
    return {name: dict(info) for name, info in _DEFAULT_USERS.items()}


# ─── User CRUD ────────────────────────────────────────────────────────────────
# Simple in-memory cache — avoids a disk read on every auth request.
# Cache is invalidated by save_users() so it always reflects the current state.

_users_cache = None  # type: dict | None  # populated on first load


def load_users() -> dict:
    """Load users from cache or disk. Creates users.json with defaults if missing."""
    global _users_cache
    if _users_cache is not None:
        return _users_cache
    if not USERS_FILE.exists():
        users = _create_default_users()
        save_users(users)   # also populates cache
        print(f"[Auth] Created default users.json at {USERS_FILE}")
        return _users_cache  # type: ignore[return-value]
    with open(USERS_FILE) as f:
        _users_cache = json.load(f)
    return _users_cache


def save_users(users: dict) -> None:
    global _users_cache
    USERS_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(USERS_FILE, "w") as f:
        json.dump(users, f, indent=2)
    _users_cache = users  # keep cache in sync
