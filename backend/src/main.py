from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.api import chatbot, textbook, auth, labs, translation
from src.config import settings


def create_app() -> FastAPI:
    app = FastAPI(
        title="AI Textbook for Physical AI & Humanoid Robotics API",
        description="Backend API for the AI-native textbook project",
        version="1.0.0",
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.ALLOWED_ORIGINS,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include API routers
    app.include_router(chatbot.router, prefix="/api", tags=["chatbot"])
    app.include_router(textbook.router, prefix="/api", tags=["textbook"])
    app.include_router(auth.router, prefix="/api", tags=["auth"])
    app.include_router(labs.router, prefix="/api", tags=["labs"])
    app.include_router(translation.router, prefix="/api", tags=["translation"])

    @app.get("/")
    def read_root():
        return {"message": "AI Textbook for Physical AI & Humanoid Robotics API"}

    @app.get("/health")
    def health_check():
        return {"status": "healthy"}

    return app


app = create_app()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)