"""SQLite 不可变存储：方案（plans）与版本快照（versions）。

快照以规范化 JSON 字符串整体写入，只增不改；读取时原样返回，
保证同一版本重复读取结果逐字节一致。
"""

from __future__ import annotations

import json
import os
import sqlite3
import uuid
from datetime import datetime, timezone

DB_PATH = os.environ.get("WHEEL_API_DB", os.path.join(os.getcwd(), "wheel_plans.db"))

_SCHEMA = """
CREATE TABLE IF NOT EXISTS plans (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    created_at TEXT NOT NULL,
    current_version INTEGER NOT NULL
);
CREATE TABLE IF NOT EXISTS versions (
    plan_id TEXT NOT NULL,
    version INTEGER NOT NULL,
    snapshot TEXT NOT NULL,
    created_at TEXT NOT NULL,
    PRIMARY KEY (plan_id, version)
);
"""


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _conn() -> sqlite3.Connection:
    conn = sqlite3.connect(DB_PATH)
    conn.execute("PRAGMA foreign_keys = ON")
    return conn


def init_db() -> None:
    with _conn() as conn:
        conn.executescript(_SCHEMA)


def canonical(snapshot: dict) -> str:
    """规范化序列化：键排序，保证存储与读取内容稳定。"""
    return json.dumps(snapshot, ensure_ascii=False, sort_keys=True)


def new_plan_id() -> str:
    return "whl_" + uuid.uuid4().hex[:12]


def create_plan(name: str) -> tuple[str, str]:
    plan_id = new_plan_id()
    created = _now()
    with _conn() as conn:
        conn.execute(
            "INSERT INTO plans (id, name, created_at, current_version) VALUES (?, ?, ?, 0)",
            (plan_id, name, created),
        )
    return plan_id, created


def insert_plan_with_version(plan_id: str, name: str, version: int, snapshot_text: str) -> str:
    """原子写入方案与首个版本快照：任一失败则整体不写入（不留空方案）。"""
    created = _now()
    with _conn() as conn:
        conn.execute(
            "INSERT INTO plans (id, name, created_at, current_version) VALUES (?, ?, ?, ?)",
            (plan_id, name, created, version),
        )
        conn.execute(
            "INSERT INTO versions (plan_id, version, snapshot, created_at) VALUES (?, ?, ?, ?)",
            (plan_id, version, snapshot_text, created),
        )
    return created


def plan_row(plan_id: str):
    with _conn() as conn:
        return conn.execute(
            "SELECT id, name, created_at, current_version FROM plans WHERE id = ?", (plan_id,)
        ).fetchone()


def next_version(plan_id: str) -> int:
    row = plan_row(plan_id)
    return row[3] + 1


def insert_version(plan_id: str, version: int, snapshot_text: str) -> str:
    created = _now()
    with _conn() as conn:
        conn.execute(
            "INSERT INTO versions (plan_id, version, snapshot, created_at) VALUES (?, ?, ?, ?)",
            (plan_id, version, snapshot_text, created),
        )
        conn.execute(
            "UPDATE plans SET current_version = ? WHERE id = ?", (version, plan_id)
        )
    return created


def get_snapshot(plan_id: str, version: int | None = None) -> str | None:
    with _conn() as conn:
        if version is None:
            row = conn.execute(
                "SELECT snapshot FROM versions WHERE plan_id = ? ORDER BY version DESC LIMIT 1",
                (plan_id,),
            ).fetchone()
        else:
            row = conn.execute(
                "SELECT snapshot FROM versions WHERE plan_id = ? AND version = ?",
                (plan_id, version),
            ).fetchone()
    return row[0] if row else None


def list_plans() -> list[dict]:
    with _conn() as conn:
        rows = conn.execute(
            "SELECT id, name, created_at, current_version FROM plans ORDER BY created_at"
        ).fetchall()
    return [
        {"plan_id": r[0], "name": r[1], "created_at": r[2], "current_version": r[3]}
        for r in rows
    ]
