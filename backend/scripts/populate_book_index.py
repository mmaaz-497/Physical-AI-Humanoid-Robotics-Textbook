"""
Script to populate the book_index table with Qdrant ID to chapter mappings.
Run this after setting up the database and Qdrant collection.
"""

import sys
import os

# Add parent directory to path to import src modules
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from qdrant_client import QdrantClient
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from src.config import settings
from src.models.database import BookIndex, Base

def populate_book_index():
    """
    Fetch all points from Qdrant and populate book_index table with mappings.
    """
    print("Connecting to Qdrant...")
    qdrant_client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
    )

    print("Connecting to Neon database...")
    engine = create_engine(settings.NEON_DATABASE_URL)
    SessionLocal = sessionmaker(bind=engine)
    db = SessionLocal()

    try:
        # Get collection info
        collection_info = qdrant_client.get_collection(settings.QDRANT_COLLECTION)
        print(f"Collection '{settings.QDRANT_COLLECTION}' has {collection_info.points_count} points")

        # Scroll through all points in the collection
        print("Fetching points from Qdrant...")
        offset = None
        total_inserted = 0
        batch_size = 100

        while True:
            # Scroll through points
            points, offset = qdrant_client.scroll(
                collection_name=settings.QDRANT_COLLECTION,
                limit=batch_size,
                offset=offset,
                with_payload=True,
                with_vectors=False,
            )

            if not points:
                break

            # Insert into book_index
            for point in points:
                qdrant_id = str(point.id)
                payload = point.payload

                # Extract metadata from payload
                chapter_title = payload.get("chapter", "Unknown Chapter")
                file_path = payload.get("file_path", "/docs/unknown")
                section_heading = payload.get("section", None)

                # Check if already exists
                existing = db.query(BookIndex).filter(BookIndex.qdrant_id == qdrant_id).first()

                if not existing:
                    book_entry = BookIndex(
                        qdrant_id=qdrant_id,
                        chapter_title=chapter_title,
                        file_path=file_path,
                        section_heading=section_heading,
                    )
                    db.add(book_entry)
                    total_inserted += 1

            db.commit()
            print(f"Processed {len(points)} points. Total inserted: {total_inserted}")

            # Check if we've reached the end
            if offset is None:
                break

        print(f"\n✅ Successfully populated book_index with {total_inserted} new entries")

        # Display summary
        total_count = db.query(BookIndex).count()
        print(f"Total entries in book_index: {total_count}")

    except Exception as e:
        print(f"❌ Error: {str(e)}")
        db.rollback()
        raise

    finally:
        db.close()


if __name__ == "__main__":
    print("=" * 60)
    print("Populating book_index table from Qdrant")
    print("=" * 60)
    populate_book_index()
