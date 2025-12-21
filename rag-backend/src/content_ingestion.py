import asyncio
import os
from typing import List, Dict, Any
from pathlib import Path
import markdown
from bs4 import BeautifulSoup
import logging

from src.embedding_engine import embedding_engine
from src.vector_store import vector_store


class ContentIngestionPipeline:
    def __init__(self, textbook_dir: str = "frontend/docs/textbook"):
        self.textbook_dir = Path(textbook_dir)
        self.logger = logging.getLogger(__name__)
    
    async def process_textbook_content(self):
        """
        Process all textbook content and populate the Qdrant vector store
        """
        self.logger.info(f"Starting to process textbook content from {self.textbook_dir}")
        
        # Get all markdown files in the textbook directory
        md_files = list(self.textbook_dir.rglob("*.md"))
        
        for md_file in md_files:
            await self._process_single_file(md_file)
        
        self.logger.info(f"Completed processing {len(md_files)} textbook files")
    
    async def _process_single_file(self, file_path: Path):
        """
        Process a single markdown file and store its content in the vector store
        """
        self.logger.info(f"Processing file: {file_path}")
        
        try:
            # Read the markdown file
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Parse the markdown to extract content
            metadata, text_content = self._parse_markdown(content, file_path)
            
            # Split content into chunks
            chunks = self._split_content_into_chunks(text_content)
            
            # Process each chunk
            for i, chunk in enumerate(chunks):
                chunk_id = f"{file_path.stem}_chunk_{i}"
                
                # Create metadata for this chunk
                chunk_metadata = metadata.copy()
                chunk_metadata["chunk_index"] = i
                chunk_metadata["total_chunks"] = len(chunks)
                chunk_metadata["file_path"] = str(file_path.relative_to(self.textbook_dir))
                
                # Create embeddings for the chunk
                embeddings = embedding_engine.create_embeddings([chunk])
                
                # Store in vector database
                await vector_store.add_embedding(
                    content_id=chunk_id,
                    embedding=embeddings[0],
                    text=chunk,
                    metadata=chunk_metadata
                )
                
                self.logger.debug(f"Stored chunk {i} of {file_path.name}")
        
        except Exception as e:
            self.logger.error(f"Error processing file {file_path}: {str(e)}")
            raise
    
    def _parse_markdown(self, content: str, file_path: Path) -> tuple[Dict[str, Any], str]:
        """
        Parse markdown content to extract metadata and text content
        """
        # Split content by lines to separate frontmatter
        lines = content.split('\n')
        
        # Check if there's frontmatter (indicated by --- at start and end)
        metadata = {}
        content_start = 0
        
        if lines and lines[0].strip() == '---':
            # Find the end of frontmatter
            for i, line in enumerate(lines[1:], 1):
                if line.strip() == '---':
                    # Parse the frontmatter
                    frontmatter = '\n'.join(lines[1:i])
                    for line in frontmatter.split('\n'):
                        if ':' in line:
                            key, value = line.split(':', 1)
                            metadata[key.strip()] = value.strip().strip('"\'')
                    
                    content_start = i + 1
                    break
        
        # Get the actual content (without frontmatter)
        text_content = '\n'.join(lines[content_start:])
        
        # Add file-specific metadata
        metadata["file_name"] = file_path.name
        metadata["file_stem"] = file_path.stem
        metadata["relative_path"] = str(file_path.relative_to(self.textbook_dir))
        metadata["content_type"] = "textbook"
        
        # Convert markdown to plain text for better indexing
        html = markdown.markdown(text_content)
        soup = BeautifulSoup(html, 'html.parser')
        text_content = soup.get_text()
        
        return metadata, text_content
    
    def _split_content_into_chunks(self, content: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
        """
        Split content into overlapping chunks
        """
        if len(content) <= chunk_size:
            return [content]
        
        chunks = []
        start = 0
        
        while start < len(content):
            end = start + chunk_size
            
            # Try to break at sentence boundary if possible
            if end < len(content):
                # Look for a sentence ending near the chunk boundary
                sentence_end = content.rfind('. ', start + chunk_size // 2, start + chunk_size)
                if sentence_end != -1 and sentence_end > start:
                    end = sentence_end + 2  # Include the period and space
                else:
                    # If no sentence ending found, just cut at chunk_size
                    end = start + chunk_size
            
            chunk = content[start:end].strip()
            if chunk:  # Only add non-empty chunks
                chunks.append(chunk)
            
            # Move start with overlap
            start = end - overlap if end < len(content) else end
        
        return chunks


# Function to run the ingestion pipeline
async def run_content_ingestion():
    """
    Run the content ingestion pipeline to populate the vector store
    """
    pipeline = ContentIngestionPipeline()
    await pipeline.process_textbook_content()
    print("Content ingestion completed successfully!")


# If this file is run directly, execute the ingestion
if __name__ == "__main__":
    asyncio.run(run_content_ingestion())