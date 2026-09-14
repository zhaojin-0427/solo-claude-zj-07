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
CREATE TABLE IF NOT EXISTS batches (
    id TEXT PRIMARY KEY,
    plan_id TEXT NOT NULL,
    plan_version INTEGER NOT NULL,
    state TEXT NOT NULL,
    created_at TEXT NOT NULL,
    updated_at TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS batch_events (
    batch_id TEXT NOT NULL,
    seq INTEGER NOT NULL,
    kind TEXT NOT NULL,
    snapshot TEXT NOT NULL,
    created_at TEXT NOT NULL,
    PRIMARY KEY (batch_id, seq)
);
CREATE INDEX IF NOT EXISTS idx_batches_plan ON batches (plan_id);
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


# ---------------------------------------------------------------------------
# 调校批次：batches 保存可变当前状态；batch_events 只增不改，完整轨迹可重建
# ---------------------------------------------------------------------------

def new_batch_id() -> str:
    return "trn_" + uuid.uuid4().hex[:12]


def insert_batch(batch_id: str, plan_id: str, plan_version: int,
                 state: dict, kind: str, event_snapshot: dict) -> str:
    """原子创建批次并写入首个事件快照。"""
    created = _now()
    state_text = canonical(state)
    event_text = canonical(event_snapshot)
    with _conn() as conn:
        conn.execute(
            "INSERT INTO batches (id, plan_id, plan_version, state, created_at, updated_at)"
            " VALUES (?, ?, ?, ?, ?, ?)",
            (batch_id, plan_id, plan_version, state_text, created, created),
        )
        conn.execute(
            "INSERT INTO batch_events (batch_id, seq, kind, snapshot, created_at)"
            " VALUES (?, 1, ?, ?, ?)",
            (batch_id, kind, event_text, created),
        )
    return created


def batch_row(batch_id: str):
    with _conn() as conn:
        return conn.execute(
            "SELECT id, plan_id, plan_version, state, created_at, updated_at"
            " FROM batches WHERE id = ?",
            (batch_id,),
        ).fetchone()


def list_batches(plan_id: str | None = None) -> list[dict]:
    with _conn() as conn:
        if plan_id is None:
            rows = conn.execute(
                "SELECT id, plan_id, plan_version, state, created_at, updated_at"
                " FROM batches ORDER BY created_at"
            ).fetchall()
        else:
            rows = conn.execute(
                "SELECT id, plan_id, plan_version, state, created_at, updated_at"
                " FROM batches WHERE plan_id = ? ORDER BY created_at",
                (plan_id,),
            ).fetchall()
    out = []
    for bid, pid, ver, state_text, created, updated in rows:
        state = json.loads(state_text)
        out.append({
            "batch_id": bid,
            "plan_id": pid,
            "plan_version": ver,
            "name": state.get("name"),
            "status": state["status"],
            "confirmed_rounds": len(state.get("rounds", [])),
            "created_at": created,
            "updated_at": updated,
        })
    return out


def update_batch(batch_id: str, state: dict, kind: str, event_snapshot: dict) -> str:
    """原子更新当前状态并追加一条不可变事件快照（seq 单调递增）。"""
    created = _now()
    state_text = canonical(state)
    event_text = canonical(event_snapshot)
    with _conn() as conn:
        row = conn.execute(
            "SELECT COALESCE(MAX(seq), 0) FROM batch_events WHERE batch_id = ?",
            (batch_id,),
        ).fetchone()
        next_seq = row[0] + 1
        conn.execute(
            "UPDATE batches SET state = ?, updated_at = ? WHERE id = ?",
            (state_text, created, batch_id),
        )
        conn.execute(
            "INSERT INTO batch_events (batch_id, seq, kind, snapshot, created_at)"
            " VALUES (?, ?, ?, ?, ?)",
            (batch_id, next_seq, kind, event_text, created),
        )
    return created


def list_batch_events(batch_id: str) -> list[dict]:
    with _conn() as conn:
        rows = conn.execute(
            "SELECT seq, kind, snapshot, created_at FROM batch_events"
            " WHERE batch_id = ? ORDER BY seq",
            (batch_id,),
        ).fetchall()
    return [{"seq": r[0], "kind": r[1], "snapshot": json.loads(r[2]),
             "created_at": r[3]} for r in rows]


def get_batch_event(batch_id: str, seq: int):
    with _conn() as conn:
        row = conn.execute(
            "SELECT seq, kind, snapshot, created_at FROM batch_events"
            " WHERE batch_id = ? AND seq = ?",
            (batch_id, seq),
        ).fetchone()
    if row is None:
        return None
    return {"seq": row[0], "kind": row[1], "snapshot": json.loads(row[2]),
            "created_at": row[3]}
