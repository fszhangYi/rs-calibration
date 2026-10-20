from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, Optional, TypeVar, Union


@dataclass
class AppError(Exception):
    code: str
    message: str
    detail: Optional[str] = None

    def __str__(self) -> str:
        if self.detail:
            return f"[{self.code}] {self.message} ({self.detail})"
        return f"[{self.code}] {self.message}"


T = TypeVar("T")


@dataclass
class Ok(Generic[T]):
    value: T


@dataclass
class Err:
    error: AppError


Result = Union[Ok[T], Err]
