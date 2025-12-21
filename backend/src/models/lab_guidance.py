from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime
import uuid

class LabGuidance(BaseModel):
    id: str = str(uuid.uuid4())
    title: str
    slug: str
    content: str
    lab_type: str  # "cloud" or "hardware"
    difficulty_level: str  # "beginner", "intermediate", "advanced"
    estimated_duration: int  # in minutes
    prerequisites: List[str] = []
    related_topics: List[str] = []
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()