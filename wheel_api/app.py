"""FastAPI 应用：方案创建、版本化读取、优化与调校批次。

- POST /plans                          新建方案（版本 1：几何 + 穿法 + SVG）
- GET  /plans                          列出方案
- GET  /plans/{plan_id}                读取最新版本快照
- GET  /plans/{plan_id}/versions/{v}   读取指定版本快照（不可变，重复读取一致）
- POST /plans/{plan_id}/versions       以新的轮组输入生成新版本
- POST /plans/{plan_id}/optimize       提交库存/垫圈/张力约束，生成含优化结果的新版本

调校批次（从不可变方案版本取数，状态 collecting -> adjusting -> finalized）：
- POST /plans/{plan_id}/batches        创建调校批次（校准曲线、零位、螺距、影响系数）
- GET  /batches                        列出批次（可按 plan_id 过滤）
- GET  /batches/{batch_id}             读取批次当前状态
- POST /batches/{batch_id}/measurements 沿圈孔顺序提交整轮张力/径向/横向读数
- POST /batches/{batch_id}/locks       锁定/解锁辐条（采集中）
- GET  /batches/{batch_id}/proposals   步进候选方案与逐步预测（排序后）
- POST /batches/{batch_id}/rounds/confirm   确认候选，冻结该轮测量/动作/结果
- POST /batches/{batch_id}/rounds/complete  完成本轮，下一轮从快照继续
- POST /batches/{batch_id}/rounds/cancel    放弃调整，回到采集中
- POST /batches/{batch_id}/finalize    定稿（保留来源方案版本与完整轨迹）
- GET  /batches/{batch_id}/trajectory  完整调校轨迹与事件流
"""

from __future__ import annotations

import json
from datetime import datetime, timezone

from fastapi import FastAPI, Response
from fastapi.responses import JSONResponse

from . import batches, geometry, lacing, load_sheets, optimizer, storage, svg
from .errors import WheelError
from .schemas import OptimizeSpec, WheelSpec

storage.init_db()

app = FastAPI(
    title="Wheel Lacing API",
    version="1.2.0",
    description="自行车轮组编轮计算：辐条长度、角度、张力比、穿线图、库存组合优化、调校批次与服役载荷校核",
)

app.include_router(batches.router)
app.include_router(load_sheets.router)


@app.exception_handler(WheelError)
def _wheel_error_handler(_request, exc: WheelError):
    return JSONResponse(status_code=exc.status, content=exc.payload())


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _build_snapshot(spec: WheelSpec, plan_id: str, version: int, created_at: str) -> dict:
    layout = geometry.resolve_layout(spec)
    mapping = lacing.build_mapping(spec, layout)
    n_side = geometry.validate_counts(spec, layout)
    geo = geometry.compute_geometry(spec, mapping["mapping"], n_side, layout)
    svg_text = svg.render_svg(spec, mapping["mapping"], geo, mapping["first_spoke"]["rim_hole"], layout)
    return {
        "plan_id": plan_id,
        "version": version,
        "created_at": created_at,
        "formula_version": geometry.FORMULA_VERSION,
        "formulas": geometry.FORMULAS,
        "inputs": spec.model_dump(),
        "geometry": geo,
        "lacing": mapping,
        "svg": svg_text,
    }


def _get_plan_or_404(plan_id: str):
    row = storage.plan_row(plan_id)
    if row is None:
        raise WheelError("PLAN_NOT_FOUND", f"方案 {plan_id} 不存在", {"plan_id": plan_id}, status=404)
    return row


def _snapshot_response(text: str, status_code: int = 200) -> Response:
    return Response(content=text, media_type="application/json", status_code=status_code)


@app.get("/health")
def health():
    return {"status": "ok"}


@app.post("/plans", status_code=201)
def create_plan(spec: WheelSpec):
    # 先完成全部校验与计算，再原子写入方案与版本 1：失败请求不留下空方案
    plan_id = storage.new_plan_id()
    snapshot = _build_snapshot(spec, plan_id, 1, _now())
    text = storage.canonical(snapshot)
    storage.insert_plan_with_version(plan_id, spec.name, 1, text)
    return _snapshot_response(text, status_code=201)


@app.get("/plans")
def list_plans():
    return {"plans": storage.list_plans()}


@app.get("/plans/{plan_id}")
def get_latest(plan_id: str):
    _get_plan_or_404(plan_id)
    text = storage.get_snapshot(plan_id)
    return _snapshot_response(text)


@app.get("/plans/{plan_id}/versions/{version}")
def get_version(plan_id: str, version: int):
    _get_plan_or_404(plan_id)
    text = storage.get_snapshot(plan_id, version)
    if text is None:
        raise WheelError(
            "VERSION_NOT_FOUND",
            f"方案 {plan_id} 的版本 {version} 不存在",
            {"plan_id": plan_id, "version": version},
            status=404,
        )
    return _snapshot_response(text)


@app.post("/plans/{plan_id}/versions", status_code=201)
def add_version(plan_id: str, spec: WheelSpec):
    _get_plan_or_404(plan_id)
    version = storage.next_version(plan_id)
    snapshot = _build_snapshot(spec, plan_id, version, _now())
    text = storage.canonical(snapshot)
    storage.insert_version(plan_id, version, text)
    return _snapshot_response(text, status_code=201)


@app.post("/plans/{plan_id}/optimize", status_code=201)
def optimize_plan(plan_id: str, opt: OptimizeSpec):
    _get_plan_or_404(plan_id)
    latest = storage.get_snapshot(plan_id)
    snapshot = json.loads(latest)
    result = optimizer.optimize(snapshot["geometry"], opt)
    snapshot["optimization"] = result
    snapshot["version"] = storage.next_version(plan_id)
    snapshot["created_at"] = _now()
    text = storage.canonical(snapshot)
    storage.insert_version(plan_id, snapshot["version"], text)
    return _snapshot_response(text, status_code=201)


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("wheel_api.app:app", host="127.0.0.1", port=8000, reload=False)
