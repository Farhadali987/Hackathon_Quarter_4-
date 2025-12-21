from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
import uuid
from ..config import settings

class VectorStore:
    def __init__(self):
        if settings.qdrant_api_key:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key
            )
        else:
            self.client = QdrantClient(url=settings.qdrant_url)
        
        # Create collection if it doesn't exist
        self.collection_name = "textbook_content"
        self._create_collection()
    
    def _create_collection(self):
        """Create the collection for storing textbook content vectors if it doesn't exist"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),  # Assuming OpenAI embeddings
            )
    
    def add_vectors(self, vectors: List[Dict[str, Any]], texts: List[str], metadatas: List[Dict[str, Any]]):
        """Add vectors to the collection"""
        points = []
        for i, (vector, text, metadata) in enumerate(zip(vectors, texts, metadatas)):
            points.append(
                models.PointStruct(
                    id=str(uuid.uuid4()),
                    vector=vector,
                    payload={
                        "text": text,
                        "metadata": metadata
                    }
                )
            )
        
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )
    
    def search(self, query_vector: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """Search for similar vectors"""
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=top_k
        )
        
        return [
            {
                "text": result.payload["text"],
                "metadata": result.payload["metadata"],
                "score": result.score
            }
            for result in results
        ]
    
    def delete_collection(self):
        """Delete the collection (useful for testing)"""
        self.client.delete_collection(self.collection_name)