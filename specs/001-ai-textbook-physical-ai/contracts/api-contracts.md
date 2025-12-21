# API Contracts: AI Textbook for Physical AI & Humanoid Robotics

## Chatbot API

### POST /api/chatbot/query
**Description**: Submit a query to the RAG chatbot and receive a response based on textbook content

**Request**:
```json
{
  "query": "string (required): The user's question/query",
  "sessionId": "string (optional): Session identifier for conversation tracking",
  "userId": "string (optional): User identifier if authenticated"
}
```

**Response** (200 OK):
```json
{
  "response": "string: The AI-generated response based on textbook content",
  "sessionId": "string: Session identifier for conversation tracking",
  "sourceChunks": "array of strings: IDs of content chunks used to generate response",
  "timestamp": "string: ISO 8601 timestamp of the response"
}
```

**Error Responses**:
- 400: Invalid request format
- 429: Rate limit exceeded
- 500: Internal server error

---

## Textbook Content API

### GET /api/textbook/content
**Description**: Retrieve textbook content by various filters

**Query Parameters**:
- `topic` (optional): Filter by topic (e.g., "ROS 2", "Gazebo")
- `level` (optional): Filter by difficulty level ("beginner", "intermediate", "advanced")
- `language` (optional): Filter by language ("en", "ur")
- `contentType` (optional): Filter by content type ("chapter", "section", "lesson")
- `parentId` (optional): Get content that is a child of the specified parent

**Response** (200 OK):
```json
{
  "content": [
    {
      "id": "string: Unique identifier for the content piece",
      "title": "string: Title of the content piece",
      "content": "string: The actual educational content in markdown format",
      "contentType": "string: Type of content",
      "topic": "string: The topic covered",
      "level": "string: Difficulty level",
      "language": "string: Language code",
      "parentId": "string: ID of parent content",
      "order": "integer: Order of this content within its parent"
    }
  ]
}
```

### GET /api/textbook/content/{id}
**Description**: Retrieve specific textbook content by ID

**Response** (200 OK):
```json
{
  "id": "string: Unique identifier for the content piece",
  "title": "string: Title of the content piece",
  "content": "string: The actual educational content in markdown format",
  "contentType": "string: Type of content",
  "topic": "string: The topic covered",
  "level": "string: Difficulty level",
  "language": "string: Language code",
  "parentId": "string: ID of parent content",
  "order": "integer: Order of this content within its parent",
  "createdAt": "string: ISO 8601 timestamp",
  "updatedAt": "string: ISO 8601 timestamp"
}
```

**Error Responses**:
- 404: Content not found
- 500: Internal server error

---

## User Profile API

### GET /api/user/profile
**Description**: Retrieve current user's profile information

**Response** (200 OK):
```json
{
  "id": "string: Unique user identifier",
  "email": "string: User's email address",
  "name": "string: User's full name",
  "preferredLanguage": "string: Preferred language code",
  "learningLevel": "string: Current learning level",
  "personalizationSettings": "json: User's personalization preferences",
  "createdAt": "string: ISO 8601 timestamp",
  "lastActiveAt": "string: ISO 8601 timestamp",
  "progress": "json: Learning progress tracking"
}
```

### PUT /api/user/profile
**Description**: Update user's profile information

**Request**:
```json
{
  "name": "string (optional): User's full name",
  "preferredLanguage": "string (optional): Preferred language code",
  "learningLevel": "string (optional): Current learning level",
  "personalizationSettings": "json (optional): User's personalization preferences"
}
```

**Response** (200 OK):
```json
{
  "id": "string: Unique user identifier",
  "email": "string: User's email address",
  "name": "string: User's full name",
  "preferredLanguage": "string: Preferred language code",
  "learningLevel": "string: Current learning level",
  "personalizationSettings": "json: User's personalization preferences",
  "createdAt": "string: ISO 8601 timestamp",
  "lastActiveAt": "string: ISO 8601 timestamp",
  "progress": "json: Learning progress tracking"
}
```

---

## Lab Guidance API

### GET /api/labs
**Description**: Retrieve lab guidance by various filters

**Query Parameters**:
- `type` (optional): Filter by lab type ("hardware", "cloud", "simulation")
- `difficulty` (optional): Filter by difficulty level ("beginner", "intermediate", "advanced")
- `topic` (optional): Filter by related topic

**Response** (200 OK):
```json
{
  "labs": [
    {
      "id": "string: Unique identifier for the lab guidance",
      "title": "string: Title of the lab exercise",
      "description": "string: Detailed description of the lab",
      "type": "string: Type of lab",
      "difficulty": "string: Difficulty level",
      "duration": "integer: Estimated completion time in minutes",
      "instructions": "string: Step-by-step instructions",
      "requirements": "array of strings: Equipment/software requirements",
      "learningObjectives": "array of strings: What the user will learn"
    }
  ]
}
```

### GET /api/labs/{id}
**Description**: Retrieve specific lab guidance by ID

**Response** (200 OK):
```json
{
  "id": "string: Unique identifier for the lab guidance",
  "title": "string: Title of the lab exercise",
  "description": "string: Detailed description of the lab",
  "type": "string: Type of lab",
  "difficulty": "string: Difficulty level",
  "duration": "integer: Estimated completion time in minutes",
  "instructions": "string: Step-by-step instructions",
  "requirements": "array of strings: Equipment/software requirements",
  "learningObjectives": "array of strings: What the user will learn",
  "createdAt": "string: ISO 8601 timestamp",
  "updatedAt": "string: ISO 8601 timestamp"
}
```

---

## Translation API

### GET /api/translation/available
**Description**: Retrieve list of available languages for translation

**Response** (200 OK):
```json
{
  "languages": [
    {
      "code": "string: Language code (e.g., 'en', 'ur')",
      "name": "string: Full language name (e.g., 'English', 'Urdu')",
      "isSupported": "boolean: Whether translation is fully supported for this language"
    }
  ]
}
```