from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn
from services import process_chat_query

app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG Chatbot",
    description="A RAG chatbot for the Physical AI & Humanoid Robotics textbook.",
    version="0.1.0",
)

class ChatRequest(BaseModel):
    query: str
    user_id: str | None = None
    selected_text: str | None = None

class ChatResponse(BaseModel):
    response: str

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    response_text = await process_chat_query(request.query, request.selected_text)
    return ChatResponse(response=response_text)

@app.get("/")
async def root():
    return {"message": "Welcome to the RAG Chatbot API"}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
