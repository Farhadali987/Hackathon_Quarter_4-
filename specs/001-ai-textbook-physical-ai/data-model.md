# Data Model: AI Textbook for Physical AI & Humanoid Robotics

## TextbookContent
- **id**: UUID (Primary Key)
- **title**: String (Required)
- **slug**: String (Required, URL-friendly identifier)
- **content**: String (Required, Markdown format)
- **language**: String (Required, default: "en")
- **chapter_number**: Integer (Optional, for ordering)
- **topic**: String (Required, e.g., "ROS 2", "Gazebo", "NVIDIA Isaac")
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)
- **version**: Integer (Required, for tracking updates)

## RAGChatbot
- **id**: UUID (Primary Key)
- **session_id**: UUID (Required, for conversation tracking)
- **user_id**: UUID (Foreign Key to UserProfile, Optional for anonymous users)
- **query**: String (Required, user's question)
- **response**: String (Required, chatbot's answer)
- **source_documents**: Array of UUIDs (References to TextbookContent)
- **created_at**: DateTime (Required)
- **is_fallback**: Boolean (True if chatbot couldn't answer from indexed content)

## UserProfile
- **id**: UUID (Primary Key)
- **username**: String (Required)
- **email**: String (Required, Unique)
- **preferred_language**: String (Required, default: "en")
- **personalization_settings**: JSON (Optional, user's learning preferences)
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)
- **learning_progress**: JSON (Optional, track user's progress through content)

## LabGuidance
- **id**: UUID (Primary Key)
- **title**: String (Required)
- **slug**: String (Required, URL-friendly identifier)
- **content**: String (Required, Markdown format)
- **lab_type**: String (Required, "cloud" or "hardware")
- **difficulty_level**: String (Required, "beginner", "intermediate", "advanced")
- **estimated_duration**: Integer (Required, in minutes)
- **prerequisites**: Array of Strings (Optional, required tools/knowledge)
- **related_topics**: Array of Strings (Optional, related textbook topics)
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)

## Translation
- **id**: UUID (Primary Key)
- **source_content_id**: UUID (Required, reference to TextbookContent or LabGuidance)
- **target_language**: String (Required)
- **translated_content**: String (Required, translated content)
- **status**: String (Required, "pending", "approved", "rejected")
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)
- **reviewer_notes**: String (Optional)

## UserInteraction
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key to UserProfile, Optional for anonymous users)
- **content_id**: UUID (Foreign Key to TextbookContent)
- **interaction_type**: String (Required, "view", "bookmark", "question", "feedback")
- **metadata**: JSON (Optional, additional interaction data)
- **created_at**: DateTime (Required)

## Relationships
- UserProfile * → 1 UserInteraction (user has many interactions)
- TextbookContent * → 1 UserInteraction (content has many interactions)
- TextbookContent 1 → * Translation (content has many translations)
- LabGuidance 1 → * Translation (lab guidance has many translations)
- UserProfile 1 → * RAGChatbot (user has many chatbot sessions)
- TextbookContent * → * RAGChatbot (content referenced by chatbot responses)

## Validation Rules
- TextbookContent.title must be between 5-200 characters
- TextbookContent.slug must be unique within the same language
- UserProfile.email must be a valid email address and unique
- LabGuidance.estimated_duration must be positive
- Translation.status must be one of the allowed values
- RAGChatbot.source_documents must reference valid TextbookContent items