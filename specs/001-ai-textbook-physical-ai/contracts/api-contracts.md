# API Contracts: AI Textbook for Physical AI & Humanoid Robotics

## Chatbot Service API

### POST /api/chatbot/query
**Purpose**: Submit a query to the RAG chatbot and receive a response based on textbook content

**Request**:
```json
{
  "query": "string (required, user's question)",
  "session_id": "string (optional, for conversation tracking)",
  "user_id": "string (optional, user identifier)",
  "language": "string (optional, default: 'en')"
}
```

**Response (200 OK)**:
```json
{
  "response": "string (chatbot's answer)",
  "session_id": "string (conversation identifier)",
  "source_documents": [
    {
      "id": "string (textbook content ID)",
      "title": "string (content title)",
      "url": "string (content URL)"
    }
  ],
  "is_fallback": "boolean (true if couldn't answer from indexed content)"
}
```

**Error Responses**:
- 400: Invalid request format
- 429: Rate limit exceeded
- 500: Internal server error

### GET /api/chatbot/history/{session_id}
**Purpose**: Retrieve conversation history for a specific session

**Response (200 OK)**:
```json
{
  "session_id": "string",
  "history": [
    {
      "query": "string",
      "response": "string",
      "timestamp": "datetime",
      "source_documents": ["array of document IDs"]
    }
  ]
}
```

## Textbook Content API

### GET /api/textbook/content
**Purpose**: Retrieve available textbook content with filtering options

**Query Parameters**:
- `topic` (optional): Filter by topic (e.g., "ROS 2", "Gazebo")
- `language` (optional): Filter by language (default: "en")
- `limit` (optional): Number of results (default: 20, max: 100)
- `offset` (optional): Offset for pagination (default: 0)

**Response (200 OK)**:
```json
{
  "items": [
    {
      "id": "string",
      "title": "string",
      "slug": "string",
      "topic": "string",
      "chapter_number": "integer",
      "language": "string",
      "created_at": "datetime",
      "updated_at": "datetime"
    }
  ],
  "total": "integer",
  "limit": "integer",
  "offset": "integer"
}
```

### GET /api/textbook/content/{slug}
**Purpose**: Retrieve specific textbook content by slug

**Response (200 OK)**:
```json
{
  "id": "string",
  "title": "string",
  "slug": "string",
  "content": "string (markdown content)",
  "topic": "string",
  "chapter_number": "integer",
  "language": "string",
  "version": "integer",
  "created_at": "datetime",
  "updated_at": "datetime"
}
```

### GET /api/textbook/content/{slug}/related
**Purpose**: Retrieve content related to the specified content

**Response (200 OK)**:
```json
{
  "items": [
    {
      "id": "string",
      "title": "string",
      "slug": "string",
      "topic": "string",
      "relevance_score": "number"
    }
  ]
}
```

## Lab Guidance API

### GET /api/labs
**Purpose**: Retrieve available lab guidance with filtering options

**Query Parameters**:
- `lab_type` (optional): "cloud" or "hardware"
- `difficulty_level` (optional): "beginner", "intermediate", "advanced"
- `topic` (optional): Related textbook topic
- `limit` (optional): Number of results (default: 20, max: 100)
- `offset` (optional): Offset for pagination (default: 0)

**Response (200 OK)**:
```json
{
  "items": [
    {
      "id": "string",
      "title": "string",
      "slug": "string",
      "lab_type": "string",
      "difficulty_level": "string",
      "estimated_duration": "integer",
      "related_topics": ["array of strings"],
      "created_at": "datetime",
      "updated_at": "datetime"
    }
  ],
  "total": "integer",
  "limit": "integer",
  "offset": "integer"
}
```

### GET /api/labs/{slug}
**Purpose**: Retrieve specific lab guidance by slug

**Response (200 OK)**:
```json
{
  "id": "string",
  "title": "string",
  "slug": "string",
  "content": "string (markdown content)",
  "lab_type": "string",
  "difficulty_level": "string",
  "estimated_duration": "integer",
  "prerequisites": ["array of strings"],
  "related_topics": ["array of strings"],
  "created_at": "datetime",
  "updated_at": "datetime"
}
```

## Translation API

### GET /api/translation/{content_type}/{content_id}
**Purpose**: Retrieve translation for specific content

**Path Parameters**:
- `content_type`: "textbook" or "lab"
- `content_id`: ID of the content to translate

**Query Parameters**:
- `target_language`: Required, target language code (e.g., "ur")

**Response (200 OK)**:
```json
{
  "content_id": "string",
  "content_type": "string",
  "target_language": "string",
  "translated_content": "string (translated content)",
  "status": "string (translation status)",
  "created_at": "datetime",
  "updated_at": "datetime"
}
```

### POST /api/translation/suggest
**Purpose**: Submit a translation suggestion

**Request**:
```json
{
  "content_id": "string (ID of source content)",
  "content_type": "string (textbook/lab)",
  "target_language": "string (e.g., ur)",
  "suggested_translation": "string (suggested translation content)",
  "submitter_notes": "string (optional, additional context)"
}
```

**Response (201 Created)**:
```json
{
  "id": "string (translation suggestion ID)",
  "status": "string (pending)",
  "created_at": "datetime"
}
```

## User Profile API

### GET /api/user/profile
**Purpose**: Retrieve user profile information

**Response (200 OK)**:
```json
{
  "id": "string",
  "username": "string",
  "email": "string",
  "preferred_language": "string",
  "personalization_settings": "object",
  "created_at": "datetime",
  "updated_at": "datetime"
}
```

### PUT /api/user/profile
**Purpose**: Update user profile information

**Request**:
```json
{
  "username": "string (optional)",
  "preferred_language": "string (optional)",
  "personalization_settings": "object (optional)"
}
```

**Response (200 OK)**:
```json
{
  "id": "string",
  "username": "string",
  "email": "string",
  "preferred_language": "string",
  "personalization_settings": "object",
  "updated_at": "datetime"
}
```

### GET /api/user/progress
**Purpose**: Retrieve user's learning progress

**Response (200 OK)**:
```json
{
  "learning_progress": "object (user's progress tracking)",
  "completed_content": [
    {
      "content_id": "string",
      "title": "string",
      "completed_at": "datetime"
    }
  ],
  "current_learning_path": "array of content IDs"
}
```

## Authentication API

### POST /api/auth/login
**Purpose**: Authenticate user and return session token

**Request**:
```json
{
  "email": "string",
  "password": "string"
}
```

**Response (200 OK)**:
```json
{
  "user_id": "string",
  "token": "string",
  "expires_at": "datetime"
}
```

### POST /api/auth/register
**Purpose**: Register a new user

**Request**:
```json
{
  "username": "string",
  "email": "string",
  "password": "string"
}
```

**Response (201 Created)**:
```json
{
  "user_id": "string",
  "token": "string",
  "expires_at": "datetime"
}
```

### POST /api/auth/logout
**Purpose**: End user session

**Response (200 OK)**:
```json
{
  "message": "Successfully logged out"
}
```