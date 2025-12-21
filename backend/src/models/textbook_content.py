from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime
import uuid

class TextbookContent(BaseModel):
    id: str = str(uuid.uuid4())
    title: str
    slug: str
    content: str
    language: str = "en"
    chapter_number: Optional[int] = None
    topic: str
    created_at: datetime = datetime.now()
    updated_at: datetime = datetime.now()
    version: int = 1