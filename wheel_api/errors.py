"""领域错误：携带机器可读 code、中文原因与具体孔号等细节。"""

from __future__ import annotations


class WheelError(Exception):
    """轮组计算/校验错误。

    code:    稳定的机器可读错误码（如 HOLE_COUNT_MISMATCH）
    message: 人类可读原因（中文）
    details: 具体孔号、可行上限等结构化细节
    status:  HTTP 状态码
    """

    def __init__(self, code: str, message: str, details: dict | None = None, status: int = 400):
        self.code = code
        self.message = message
        self.details = details or {}
        self.status = status
        super().__init__(message)

    def payload(self) -> dict:
        return {"error": {"code": self.code, "message": self.message, "details": self.details}}
