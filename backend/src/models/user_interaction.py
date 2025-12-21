from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime
import uuid

class UserInteraction(BaseModel):
    id: str = str(uuid.uuid4())
    user_id: Optional[str] = None
    content_id: str
    interaction_type: str  # "view", "bookmark", "question", "feedback"
    metadata: Optional[Dict[str, Any]] = {}
    created_at: datetime = datetime.now()