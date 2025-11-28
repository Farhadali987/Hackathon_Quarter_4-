import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models
from services import get_embedding
import glob
import tiktoken

# Load environment variables from .env file
load_dotenv()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

COLLECTION_NAME = "textbook_collection"

def chunk_text(text: str, max_tokens: int = 512):
    """
    Splits a text into chunks of a maximum number of tokens.
    """
    tokenizer = tiktoken.get_encoding("cl100k_base")
    tokens = tokenizer.encode(text)
    chunks = []
    for i in range(0, len(tokens), max_tokens):
        chunk_tokens = tokens[i:i + max_tokens]
        chunk_text = tokenizer.decode(chunk_tokens)
        chunks.append(chunk_text)
    return chunks

def ingest_documents():
    """
    Ingests all markdown documents from the 'docs/textbook' directory into Qdrant.
    """
    if not os.getenv("QDRANT_URL") or not os.getenv("QDRANT_API_KEY"):
        print("Qdrant is not configured. Please set QDRANT_URL and QDRANT_API_KEY in your .env file.")
        return

    # Create collection if it doesn't exist
    try:
        qdrant_client.recreate_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
        )
        print(f"Collection '{COLLECTION_NAME}' created.")
    except Exception as e:
        if "already exists" in str(e):
            print(f"Collection '{COLLECTION_NAME}' already exists.")
        else:
            print(f"Error creating collection: {e}")
            return

    # Get all markdown files
    doc_files = glob.glob("../docs/textbook/*.md")
    
    points = []
    point_id = 0

    for file_path in doc_files:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        chunks = chunk_text(content)

        for chunk in chunks:
            embedding = get_embedding(chunk)
            points.append(models.PointStruct(
                id=point_id,
                vector=embedding,
                payload={"text": chunk, "source": os.path.basename(file_path)}
            ))
            point_id += 1

    # Upsert points to Qdrant
    if points:
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points,
            wait=True,
        )
        print(f"Ingested {len(points)} document chunks into Qdrant.")

if __name__ == "__main__":
    ingest_documents()
