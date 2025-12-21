# Data Model: AI Textbook for Physical AI & Humanoid Robotics

## Entity: TextbookContent

### Fields:
- `id` (string, primary key): Unique identifier for the content piece
- `title` (string): Title of the content piece (chapter, section, etc.)
- `content` (text): The actual educational content in markdown format
- `contentType` (string): Type of content (e.g., "chapter", "section", "lesson", "exercise")
- `topic` (string): The topic covered (e.g., "ROS 2", "Gazebo", "NVIDIA Isaac", "VLA")
- `level` (string): Difficulty level (e.g., "beginner", "intermediate", "advanced")
- `language` (string): Language code (e.g., "en", "ur")
- `parentId` (string, nullable): ID of parent content (for hierarchical structure)
- `order` (integer): Order of this content within its parent
- `createdAt` (datetime): Creation timestamp
- `updatedAt` (datetime): Last update timestamp
- `version` (integer): Version number for content tracking

### Relationships:
- One-to-many with itself (via parentId) for hierarchical content structure
- Many-to-one with UserProfile (via access tracking)

### Validation Rules:
- `title` must be 1-200 characters
- `content` must be 10-50000 characters
- `contentType` must be one of predefined values
- `language` must be a supported language code
- `level` must be one of "beginner", "intermediate", "advanced"

## Entity: RAGChatbot

### Fields:
- `id` (string, primary key): Unique identifier for the chatbot instance
- `sessionId` (string): Session identifier for conversation tracking
- `userId` (string, nullable): User identifier if authenticated
- `inputQuery` (text): The user's question/query
- `response` (text): The AI-generated response
- `sourceChunks` (array of strings): IDs of content chunks used to generate response
- `createdAt` (datetime): Timestamp of the interaction
- `feedback` (integer, nullable): User feedback rating (1-5) or thumbs up/down
- `feedbackComment` (text, nullable): Additional feedback from user

### Relationships:
- Many-to-one with UserProfile (via userId)
- Many-to-many with TextbookContent (via sourceChunks)

### Validation Rules:
- `inputQuery` must be 1-1000 characters
- `response` must be 1-5000 characters
- `sourceChunks` must reference valid TextbookContent IDs
- `feedback` must be null or 1-5 if provided

## Entity: UserProfile

### Fields:
- `id` (string, primary key): Unique user identifier
- `email` (string, unique): User's email address
- `name` (string): User's full name
- `preferredLanguage` (string): Preferred language code (e.g., "en", "ur")
- `learningLevel` (string): Current learning level (e.g., "beginner", "intermediate")
- `personalizationSettings` (json): User's personalization preferences
- `createdAt` (datetime): Account creation timestamp
- `lastActiveAt` (datetime): Last activity timestamp
- `progress` (json): Learning progress tracking

### Relationships:
- One-to-many with RAGChatbot (via userId)
- One-to-many with LabGuidance (via userId) for tracking lab completions

### Validation Rules:
- `email` must be a valid email format
- `preferredLanguage` must be a supported language code
- `learningLevel` must be one of "beginner", "intermediate", "advanced"

## Entity: LabGuidance

### Fields:
- `id` (string, primary key): Unique identifier for the lab guidance
- `title` (string): Title of the lab exercise
- `description` (text): Detailed description of the lab
- `type` (string): Type of lab ("hardware", "cloud", "simulation")
- `difficulty` (string): Difficulty level ("beginner", "intermediate", "advanced")
- `duration` (integer): Estimated completion time in minutes
- `instructions` (text): Step-by-step instructions
- `requirements` (array of strings): Equipment/software requirements
- `learningObjectives` (array of strings): What the user will learn
- `createdAt` (datetime): Creation timestamp
- `updatedAt` (datetime): Last update timestamp

### Relationships:
- Many-to-many with UserProfile (for tracking who has completed which labs)
- Many-to-one with TextbookContent (for related content)

### Validation Rules:
- `title` must be 1-200 characters
- `type` must be one of "hardware", "cloud", "simulation"
- `difficulty` must be one of "beginner", "intermediate", "advanced"
- `duration` must be 1-999 minutes

## Additional Data Structures

### Content Embedding
- `contentId` (string): Reference to TextbookContent ID
- `embedding` (array of floats): Vector representation of content for RAG
- `chunkText` (text): The text chunk that was embedded
- `chunkIndex` (integer): Index of this chunk within the original content

### Validation Rules:
- `contentId` must reference a valid TextbookContent
- `embedding` must have the correct dimensionality for the vector database
- `chunkIndex` must be non-negative