import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from openai import OpenAI

# Load environment variables from .env file
load_dotenv()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

# Initialize OpenAI client
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

def get_embedding(text: str, model="text-embedding-3-small"):
   text = text.replace("\n", " ")
   return openai_client.embeddings.create(input=[text], model=model).data[0].embedding

async def retrieve_context(query: str, collection_name = "textbook_collection"):
    """
    Retrieves context from Qdrant based on the user's query.
    """
    if not os.getenv("QDRANT_URL") or not os.getenv("QDRANT_API_KEY"):
        return "Qdrant is not configured. Please set QDRANT_URL and QDRANT_API_KEY in your .env file."

    try:
        embedded_query = get_embedding(query)

        search_results = qdrant_client.search(
            collection_name=collection_name,
            query_vector=embedded_query,
            limit=5,
        )

        context = ""
        for result in search_results:
            context += result.payload['text'] + "\n\n"
        
        return context

    except Exception as e:
        print(f"Error retrieving context from Qdrant: {e}")
        return "Error retrieving context from the knowledge base."


async def generate_response(query: str, context: str):
    """
    Generates a response using the LLM with the provided query and context.
    """
    if not os.getenv("OPENAI_API_KEY"):
        return "OpenAI is not configured. Please set OPENAI_API_KEY in your .env file."

    try:
        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are a helpful assistant for the 'Physical AI & Humanoid Robotics' textbook. Answer the user's question based on the provided context."},
                {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {query}"}
            ]
        )
        return response.choices[0].message.content

    except Exception as e:
        print(f"Error generating response from OpenAI: {e}")
        return "Error generating response from the language model."

async def process_chat_query(query: str, selected_text: str | None = None):
    """
    Processes a chat query by retrieving context and generating a response.
    """
    context = ""
    if selected_text:
        context = selected_text
    else:
        context = await retrieve_context(query)

    response = await generate_response(query, context)
    return response
