from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel, Field


class LabGuidanceBase(BaseModel):
    title: str = Field(..., min_length=1, max_length=200)
    description: str
    type: str = Field(..., pattern="^(hardware|cloud|simulation)$")  # "hardware", "cloud", "simulation"
    difficulty: str = Field(..., pattern="^(beginner|intermediate|advanced)$")  # "beginner", "intermediate", "advanced"
    duration: int = Field(..., ge=1, le=999)  # Estimated completion time in minutes
    instructions: str
    requirements: List[str] = []
    learningObjectives: List[str] = []


class LabGuidanceCreate(LabGuidanceBase):
    pass


class LabGuidanceUpdate(BaseModel):
    title: Optional[str] = Field(None, min_length=1, max_length=200)
    description: Optional[str] = None
    type: Optional[str] = Field(None, pattern="^(hardware|cloud|simulation)$")
    difficulty: Optional[str] = Field(None, pattern="^(beginner|intermediate|advanced)$")
    duration: Optional[int] = Field(None, ge=1, le=999)
    instructions: Optional[str] = None
    requirements: Optional[List[str]] = None
    learningObjectives: Optional[List[str]] = None


class LabGuidance(LabGuidanceBase):
    id: str
    createdAt: datetime
    updatedAt: datetime

    class Config:
        from_attributes = True