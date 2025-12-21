from pydantic import BaseModel
from typing import Optional
from datetime import datetime
import uuid

class Translation(BaseModel):
    id: str = str(uuid.uuid4())
    source_content_id: str
    target_language: str
    translated_content: str
    status: str = "pending"  # "pending", "approved", "rejected"
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()
    reviewer_notes: Optional[str] = None