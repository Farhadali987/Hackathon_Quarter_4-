from typing import List, Optional
from src.models.textbook_content import TextbookContent, TextbookContentCreate, TextbookContentUpdate
from src.models.user_profile import User
from src.utils import NotFoundError


class TextbookContentService:
    # In-memory storage for demonstration purposes
    # In a real application, this would connect to a database
    _storage: List[TextbookContent] = []
    
    @classmethod
    async def create_content(cls, content_create: TextbookContentCreate) -> TextbookContent:
        """Create a new textbook content item"""
        from datetime import datetime
        import uuid
        
        new_content = TextbookContent(
            id=str(uuid.uuid4()),
            title=content_create.title,
            content=content_create.content,
            contentType=content_create.contentType,
            topic=content_create.topic,
            level=content_create.level,
            language=content_create.language,
            parentId=content_create.parentId,
            order=content_create.order,
            createdAt=datetime.utcnow(),
            updatedAt=datetime.utcnow(),
            version=1
        )
        
        cls._storage.append(new_content)
        return new_content
    
    @classmethod
    async def get_content_by_id(cls, content_id: str) -> Optional[TextbookContent]:
        """Get a textbook content item by its ID"""
        for content in cls._storage:
            if content.id == content_id:
                return content
        return None
    
    @classmethod
    async def get_content_list(
        cls, 
        topic: Optional[str] = None, 
        level: Optional[str] = None, 
        language: Optional[str] = None, 
        content_type: Optional[str] = None,
        parent_id: Optional[str] = None
    ) -> List[TextbookContent]:
        """Get a list of textbook content items with optional filters"""
        results = cls._storage
        
        if topic:
            results = [c for c in results if c.topic.lower() == topic.lower()]
        if level:
            results = [c for c in results if c.level.lower() == level.lower()]
        if language:
            results = [c for c in results if c.language.lower() == language.lower()]
        if content_type:
            results = [c for c in results if c.contentType.lower() == content_type.lower()]
        if parent_id:
            results = [c for c in results if c.parentId == parent_id]
        
        return results
    
    @classmethod
    async def update_content(cls, content_id: str, content_update: TextbookContentUpdate) -> Optional[TextbookContent]:
        """Update a textbook content item"""
        from datetime import datetime
        
        for i, content in enumerate(cls._storage):
            if content.id == content_id:
                update_data = content_update.dict(exclude_unset=True)
                updated_content = content.copy(update=update_data)
                updated_content.updatedAt = datetime.utcnow()
                updated_content.version += 1
                cls._storage[i] = updated_content
                return updated_content
        return None
    
    @classmethod
    async def delete_content(cls, content_id: str) -> bool:
        """Delete a textbook content item"""
        for i, content in enumerate(cls._storage):
            if content.id == content_id:
                del cls._storage[i]
                return True
        return False
    
    @classmethod
    async def get_content_by_topic_and_level(cls, topic: str, level: str) -> List[TextbookContent]:
        """Get content filtered by topic and level"""
        results = []
        for content in cls._storage:
            if (content.topic.lower() == topic.lower() and 
                content.level.lower() == level.lower()):
                results.append(content)
        return results
    
    @classmethod
    async def get_child_content(cls, parent_id: str) -> List[TextbookContent]:
        """Get all child content items for a given parent"""
        return [c for c in cls._storage if c.parentId == parent_id]


# Initialize with some sample content
async def initialize_sample_content():
    from src.models.textbook_content import TextbookContentCreate
    
    sample_content = [
        TextbookContentCreate(
            title="Introduction to Physical AI",
            content="Physical AI is a field that combines artificial intelligence with physical systems...",
            contentType="chapter",
            topic="Physical AI",
            level="beginner",
            language="en",
            parentId=None,
            order=1
        ),
        TextbookContentCreate(
            title="What is Humanoid Robotics?",
            content="Humanoid robotics is the branch of robotics that studies and develops robots with human-like features...",
            contentType="chapter",
            topic="Humanoid Robotics",
            level="beginner",
            language="en",
            parentId=None,
            order=2
        )
    ]
    
    for content in sample_content:
        await TextbookContentService.create_content(content)