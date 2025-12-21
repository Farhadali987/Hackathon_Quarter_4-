from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime
import uuid

class RAGChatbot(BaseModel):
    id: str = str(uuid.uuid4())
    session_id: str = str(uuid.uuid4())
    user_id: Optional[str] = None
    query: str
    response: str
    source_documents: List[str] = []
    created_at: datetime = datetime.now()
    is_fallback: bool = False