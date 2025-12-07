"""
Upload Docusaurus Book Content to Qdrant Vector Database (Python Version)

This script uses Qdrant FastEmbed for local embedding generation.
No external API calls needed - embeddings are generated locally!

Requirements:
- pip install qdrant-client fastembed python-dotenv
"""

import os
import re
from pathlib import Path
from typing import List, Dict
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from fastembed import TextEmbedding

# Load environment variables
load_dotenv()

# Configuration
QDRANT_URL = os.getenv('QDRANT_URL')
QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')
COLLECTION_NAME = os.getenv('QDRANT_COLLECTION_NAME', 'physical_ai_book')
CHUNK_SIZE = int(os.getenv('CHUNK_SIZE', '1000'))
CHUNK_OVERLAP = int(os.getenv('CHUNK_OVERLAP', '200'))

# Validate environment variables
if not QDRANT_URL or not QDRANT_API_KEY:
    print("❌ Error: Missing required environment variables")
    print("Please set QDRANT_URL and QDRANT_API_KEY in your .env file")
    exit(1)


def extract_chapter_info(file_path: str) -> Dict[str, any]:
    """Extract chapter number and name from file path."""
    match = re.search(r'docs[/\\](\d+)-([^/\\]+)[/\\]', file_path)
    if match:
        return {
            'number': int(match.group(1)),
            'name': match.group(2).replace('-', ' ')
        }
    return {'number': 0, 'name': 'unknown'}


def extract_title(content: str) -> str:
    """Extract title from first h1 heading."""
    match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
    return match.group(1).strip() if match else 'Untitled'


def clean_content(content: str) -> str:
    """Remove MDX/JSX syntax and clean markdown."""
    # Remove frontmatter
    content = re.sub(r'^---[\s\S]*?---\n', '', content, flags=re.MULTILINE)
    # Remove import statements
    content = re.sub(r'^import\s+.+$', '', content, flags=re.MULTILINE)
    # Remove JSX components
    content = re.sub(r'<[A-Z][^>]*>', '', content)
    content = re.sub(r'</[A-Z][^>]*>', '', content)
    # Remove HTML comments
    content = re.sub(r'<!--[\s\S]*?-->', '', content)
    # Normalize whitespace
    content = re.sub(r'\n{3,}', '\n\n', content)
    return content.strip()


def chunk_text(text: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> List[str]:
    """Chunk text into fixed-size pieces with overlap."""
    chunks = []
    start = 0

    while start < len(text):
        end = min(start + chunk_size, len(text))
        chunk = text[start:end].strip()

        if chunk:
            chunks.append(chunk)

        start = end - overlap
        if start >= len(text) - overlap:
            break

    return chunks


def find_mdx_files(docs_dir: Path) -> List[Path]:
    """Find all .mdx and .md files in docs directory."""
    return list(docs_dir.rglob('*.mdx')) + list(docs_dir.rglob('*.md'))


def main():
    print("Physical AI Book -> Qdrant Upload Script (Python + FastEmbed)\n")

    # Initialize Qdrant client with increased timeout
    print(f"Connecting to Qdrant at {QDRANT_URL}...")
    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
        timeout=60,  # Increase timeout to 60 seconds
    )

    # Initialize FastEmbed (local embedding model)
    print("Loading FastEmbed model (BAAI/bge-small-en-v1.5)...")
    embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")

    # Get vector size from model
    test_embedding = list(embedding_model.embed(["test"]))[0]
    vector_size = len(test_embedding)
    print(f"[OK] Embedding model loaded (dimension: {vector_size})\n")

    # Check if collection exists
    collections = client.get_collections().collections
    collection_exists = any(c.name == COLLECTION_NAME for c in collections)

    if collection_exists:
        print(f"[INFO] Collection '{COLLECTION_NAME}' already exists - will append new data\n")
    else:
        # Create collection
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=vector_size,
                distance=Distance.COSINE
            )
        )
        print(f"[OK] Created collection '{COLLECTION_NAME}'\n")

    # Find all MDX files
    docs_dir = Path(__file__).parent / 'docs'
    print(f"[SCAN] Scanning {docs_dir} for content...")
    mdx_files = find_mdx_files(docs_dir)
    print(f"[OK] Found {len(mdx_files)} files\n")

    # Process files
    all_points = []
    point_id = 0

    for file_path in mdx_files:
        relative_path = file_path.relative_to(Path(__file__).parent)
        content = file_path.read_text(encoding='utf-8')

        chapter = extract_chapter_info(str(relative_path))
        title = extract_title(content)
        cleaned_content = clean_content(content)
        chunks = chunk_text(cleaned_content)

        print(f"[FILE] {relative_path}")
        print(f"   Chapter {chapter['number']}: {chapter['name']}")
        print(f"   Title: {title}")
        print(f"   Chunks: {len(chunks)}")

        # Generate embeddings for all chunks
        print(f"   Generating embeddings...", end=' ')
        embeddings = list(embedding_model.embed(chunks))
        print("[OK]")

        # Create points
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            point = PointStruct(
                id=point_id,
                vector=embedding.tolist(),
                payload={
                    'text': chunk,
                    'file_path': str(relative_path),
                    'chapter_number': chapter['number'],
                    'chapter_name': chapter['name'],
                    'document_title': title,
                    'chunk_index': i,
                    'total_chunks': len(chunks),
                }
            )
            all_points.append(point)
            point_id += 1

        print()

    # Upload to Qdrant in batches
    print(f"[TOTAL] Total chunks: {len(all_points)}")
    print(f"[UPLOAD] Uploading to Qdrant...\n")

    batch_size = 50  # Reduced from 100 to 50 for better reliability
    for i in range(0, len(all_points), batch_size):
        batch = all_points[i:i + batch_size]
        batch_num = i // batch_size + 1
        total_batches = (len(all_points) + batch_size - 1) // batch_size

        print(f"[BATCH] Uploading batch {batch_num}/{total_batches} ({len(batch)} points)...", end=' ')

        # Retry logic for network timeouts
        max_retries = 3
        for retry in range(max_retries):
            try:
                client.upsert(
                    collection_name=COLLECTION_NAME,
                    points=batch
                )
                print("[OK]")
                break
            except Exception as e:
                if retry < max_retries - 1:
                    print(f"[RETRY {retry + 1}/{max_retries}]", end=' ')
                else:
                    print(f"[FAILED] {str(e)}")
                    raise

    print("\n[SUCCESS] Upload completed successfully!\n")
    print(f"[SUMMARY]")
    print(f"   - Files processed: {len(mdx_files)}")
    print(f"   - Total chunks: {len(all_points)}")
    print(f"   - Collection: {COLLECTION_NAME}")
    print(f"   - Vector dimension: {vector_size}")
    print(f"\n[DONE] Your book is now searchable in Qdrant!")
    print(f"\nTest a search:")
    print(f"   from qdrant_client import QdrantClient")
    print(f"   client = QdrantClient(url='{QDRANT_URL}', api_key='...')")
    print(f"   results = client.search(collection_name='{COLLECTION_NAME}', query_vector=..., limit=5)")


if __name__ == '__main__':
    main()
