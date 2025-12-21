from typing import List, Dict, Any
from .vector_store import VectorStore
from .embedding_engine import EmbeddingEngine
from .content_ingestion import ContentIngestion

class RAGAgent:
    def __init__(self):
        self.vector_store = VectorStore()
        self.embedding_engine = EmbeddingEngine()
        self.content_ingestion = ContentIngestion()
    
    def ingest_content(self, content_dir: str = "frontend/docs"):
        """Ingest content from markdown files into vector store"""
        # Get textbook content
        textbook_contents = self.content_ingestion.read_textbook_content()
        
        # Get lab content
        lab_contents = self.content_ingestion.read_lab_content()
        
        # Combine all content
        all_contents = textbook_contents + lab_contents
        
        # Create embeddings and store in vector database
        texts = [item['text'] for item in all_contents]
        embeddings = self.embedding_engine.create_embeddings(texts)
        
        # Prepare metadata for each content item
        metadatas = []
        for item in all_contents:
            metadata = {
                'title': item['title'],
                'file_path': item['file_path'],
                'type': 'textbook' if 'topic' in item else 'lab',
            }
            if 'topic' in item:
                metadata['topic'] = item['topic']
            if 'lab_type' in item:
                metadata['lab_type'] = item['lab_type']
            
            metadatas.append(metadata)
        
        # Add vectors to the store
        if embeddings:
            self.vector_store.add_vectors(embeddings, texts, metadatas)
        
        return len(all_contents)
    
    def query(self, question: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """Query the RAG system with a question"""
        # Create embedding for the question
        question_embedding = self.embedding_engine.create_embedding(question)
        
        if not question_embedding:
            return []
        
        # Search in vector store
        results = self.vector_store.search(question_embedding, top_k)
        
        return results