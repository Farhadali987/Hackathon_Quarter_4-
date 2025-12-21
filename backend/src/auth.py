"""
Authentication module for the AI Textbook for Physical AI & Humanoid Robotics project.
Uses a mock authentication system that follows the Better-Auth pattern.
"""
from datetime import datetime, timedelta
from typing import Optional
import jwt
from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel
from src.config import SECRET_KEY, ALGORITHM


# Mock user database
users_db = {
    "user@example.com": {
        "id": "1",
        "email": "user@example.com",
        "name": "Test User",
        "hashed_password": "fake_hashed_password",
        "preferredLanguage": "en",
        "learningLevel": "beginner",
        "personalizationSettings": {},
        "createdAt": datetime.utcnow(),
        "lastActiveAt": datetime.utcnow(),
        "progress": {}
    }
}


class User(BaseModel):
    id: str
    email: str
    name: str
    preferredLanguage: str = "en"
    learningLevel: str = "beginner"
    personalizationSettings: dict = {}
    createdAt: datetime
    lastActiveAt: datetime
    progress: dict = {}


class UserCreate(BaseModel):
    email: str
    name: str
    password: str


class UserLogin(BaseModel):
    email: str
    password: str


class UserUpdate(BaseModel):
    name: Optional[str] = None
    preferredLanguage: Optional[str] = None
    learningLevel: Optional[str] = None
    personalizationSettings: Optional[dict] = None


security = HTTPBearer()


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


def verify_token(token: str) -> Optional[User]:
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        email: str = payload.get("sub")
        if email is None:
            return None
        user_data = users_db.get(email)
        if user_data is None:
            return None
        return User(**user_data)
    except jwt.exceptions.PyJWTError:
        return None


async def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)) -> User:
    token = credentials.credentials
    user = verify_token(token)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return user


def authenticate_user(email: str, password: str) -> Optional[User]:
    user_data = users_db.get(email)
    if not user_data or password != "password":  # In real app, verify hashed password
        return None
    return User(**user_data)


def register_user(user_data: UserCreate) -> User:
    # In a real application, you would hash the password
    user = User(
        id=str(len(users_db) + 1),
        email=user_data.email,
        name=user_data.name,
        preferredLanguage="en",
        learningLevel="beginner",
        personalizationSettings={},
        createdAt=datetime.utcnow(),
        lastActiveAt=datetime.utcnow(),
        progress={}
    )
    users_db[user_data.email] = user.dict()
    return user