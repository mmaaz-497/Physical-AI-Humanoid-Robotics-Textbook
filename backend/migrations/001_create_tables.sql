-- Migration: Create chat_logs and book_index tables
-- Database: Neon Serverless Postgres

-- Create chat_logs table
CREATE TABLE IF NOT EXISTS chat_logs (
    id SERIAL PRIMARY KEY,
    question TEXT NOT NULL,
    answer TEXT NOT NULL,
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP NOT NULL,
    retrieval_metadata JSONB,
    model_used VARCHAR(50) NOT NULL,
    session_id VARCHAR(255)
);

-- Create index on timestamp for efficient queries
CREATE INDEX idx_chat_logs_timestamp ON chat_logs(timestamp DESC);

-- Create index on session_id for session-based queries
CREATE INDEX idx_chat_logs_session_id ON chat_logs(session_id) WHERE session_id IS NOT NULL;

-- Create book_index table
CREATE TABLE IF NOT EXISTS book_index (
    id SERIAL PRIMARY KEY,
    qdrant_id VARCHAR(255) UNIQUE NOT NULL,
    chapter_title VARCHAR(255) NOT NULL,
    file_path VARCHAR(500) NOT NULL,
    section_heading VARCHAR(255)
);

-- Create index on qdrant_id for fast lookups
CREATE INDEX idx_book_index_qdrant_id ON book_index(qdrant_id);

-- Create index on chapter_title for browsing
CREATE INDEX idx_book_index_chapter ON book_index(chapter_title);

-- Insert some sample book_index entries (example - replace with actual data)
-- These map Qdrant vector IDs to book chapters
INSERT INTO book_index (qdrant_id, chapter_title, file_path, section_heading) VALUES
    ('chunk_001', 'Introduction to Physical AI', '/docs/introduction/what-is-physical-ai', 'What is Physical AI?'),
    ('chunk_002', 'Module 1: ROS 2 Fundamentals', '/docs/ros2/introduction', 'Introduction to ROS 2'),
    ('chunk_003', 'Module 2: Digital Twin Systems', '/docs/digital-twin/introduction', 'Digital Twin Overview')
ON CONFLICT (qdrant_id) DO NOTHING;

-- Display table info
SELECT 'chat_logs table created' AS status;
SELECT 'book_index table created' AS status;
