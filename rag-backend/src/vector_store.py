import asyncio
from typing import List, Optional, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from src.config import settings
import uuid


class VectorStore:
    def __init__(self, host: str = settings.QDRANT_HOST, port: int = settings.QDRANT_PORT):
        self.client = QdrantClient(host=host, port=port)
        self.collection_name = "textbook_content"
        self._initialize_collection()
    
    def _initialize_collection(self):
        """Initialize the Qdrant collection if it doesn't exist"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),  # Assuming 1536-dim embeddings
            )
    
    async def add_embedding(self, content_id: str, embedding: List[float], text: str, metadata: Dict[str, Any] = None):
        """Add an embedding to the vector store"""
        if metadata is None:
            metadata = {}
        
        point = PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={
                "content_id": content_id,
                "text": text,
                "metadata": metadata
            }
        )
        
        self.client.upsert(
            collection_name=self.collection_name,
            points=[point]
        )
    
    async def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """Search for similar embeddings"""
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit
        )
        
        return [
            {
                "content_id": result.payload.get("content_id"),
                "text": result.payload.get("text"),
                "metadata": result.payload.get("metadata", {}),
                "score": result.score
            }
            for result in results
        ]
    
    async def delete_content(self, content_id: str):
        """Delete all embeddings associated with a content ID"""
        # Find all points with this content_id
        results = self.client.scroll(
            collection_name=self.collection_name,
            scroll_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="payload.content_id",
                        match=models.MatchValue(value=content_id)
                    )
                ]
            ),
            limit=10000  # Assuming not more than 10k chunks per content
        )
        
        point_ids = [record.id for record in results[0]]
        if point_ids:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=point_ids)
            )
    
    async def get_content_chunks(self, content_id: str) -> List[Dict[str, Any]]:
        """Get all chunks for a specific content ID"""
        results = self.client.scroll(
            collection_name=self.collection_name,
            scroll_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="payload.content_id",
                        match=models.MatchValue(value=content_id)
                    )
                ]
            ),
            limit=10000
        )
        
        return [
            {
                "id": str(record.id),
                "text": record.payload.get("text"),
                "metadata": record.payload.get("metadata", {})
            }
            for record in results[0]
        ]


# Global instance
vector_store = VectorStore()