from typing import Literal
from pydantic import BaseModel, ConfigDict


class ObjectSpec(BaseModel):
    model_config = ConfigDict(extra="forbid")
    id: str
    source: str


class DestinationSpec(BaseModel):
    model_config = ConfigDict(extra="forbid")
    type: Literal["tray"]
    color: Literal["red", "green", "blue"]
    location: str


class TaskPlan(BaseModel):
    model_config = ConfigDict(extra="forbid")
    task_id: str
    action: Literal["pick_and_place"]
    object: ObjectSpec
    destination: DestinationSpec


class UnknownTask(BaseModel):
    model_config = ConfigDict(extra="forbid")
    action: Literal["unknown"]
    reason: str