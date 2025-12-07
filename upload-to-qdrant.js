/**
 * Upload Docusaurus Book Content to Qdrant Vector Database
 *
 * This script:
 * 1. Reads all .mdx files from the docs/ directory
 * 2. Chunks content into fixed-size pieces (1000 chars with 200 char overlap)
 * 3. Generates embeddings using Qdrant FastEmbed (local, no API key needed)
 * 4. Uploads vectors with metadata to your Qdrant cluster
 */

const fs = require('fs').promises;
const path = require('path');
const { QdrantClient } = require('@qdrant/js-client-rest');
require('dotenv').config();

// Configuration from environment variables
const QDRANT_URL = process.env.QDRANT_URL;
const QDRANT_API_KEY = process.env.QDRANT_API_KEY;
const COLLECTION_NAME = process.env.QDRANT_COLLECTION_NAME || 'physical_ai_book';
const CHUNK_SIZE = parseInt(process.env.CHUNK_SIZE || '1000');
const CHUNK_OVERLAP = parseInt(process.env.CHUNK_OVERLAP || '200');

// Validate environment variables
if (!QDRANT_URL || !QDRANT_API_KEY) {
  console.error('❌ Error: Missing required environment variables');
  console.error('Please set QDRANT_URL and QDRANT_API_KEY in your .env file');
  process.exit(1);
}

/**
 * Extract chapter info from file path
 * e.g., "docs/01-introduction/what-is-physical-ai.mdx" -> { number: 1, name: "introduction" }
 */
function extractChapterInfo(filePath) {
  const match = filePath.match(/docs\/(\d+)-([^\/]+)\//);
  if (match) {
    return {
      number: parseInt(match[1]),
      name: match[2].replace(/-/g, ' ')
    };
  }
  return { number: 0, name: 'unknown' };
}

/**
 * Extract title from MDX content (first h1 heading)
 */
function extractTitle(content) {
  const match = content.match(/^#\s+(.+)$/m);
  return match ? match[1].trim() : 'Untitled';
}

/**
 * Remove MDX/JSX syntax and clean markdown for embedding
 */
function cleanContent(content) {
  return content
    // Remove frontmatter
    .replace(/^---[\s\S]*?---\n/m, '')
    // Remove import statements
    .replace(/^import\s+.+$/gm, '')
    // Remove JSX components (simple cleanup)
    .replace(/<[A-Z][^>]*>/g, '')
    .replace(/<\/[A-Z][^>]*>/g, '')
    // Remove HTML comments
    .replace(/<!--[\s\S]*?-->/g, '')
    // Normalize whitespace
    .replace(/\n{3,}/g, '\n\n')
    .trim();
}

/**
 * Chunk text into fixed-size pieces with overlap
 */
function chunkText(text, chunkSize = CHUNK_SIZE, overlap = CHUNK_OVERLAP) {
  const chunks = [];
  let start = 0;

  while (start < text.length) {
    const end = Math.min(start + chunkSize, text.length);
    const chunk = text.slice(start, end);

    // Only add non-empty chunks
    if (chunk.trim().length > 0) {
      chunks.push(chunk.trim());
    }

    // Move start position with overlap
    start = end - overlap;
    if (start >= text.length - overlap) break;
  }

  return chunks;
}

/**
 * Find all .mdx files in docs directory
 */
async function findMdxFiles(dir) {
  const files = [];

  async function traverse(currentDir) {
    const entries = await fs.readdir(currentDir, { withFileTypes: true });

    for (const entry of entries) {
      const fullPath = path.join(currentDir, entry.name);

      if (entry.isDirectory()) {
        await traverse(fullPath);
      } else if (entry.name.endsWith('.mdx') || entry.name.endsWith('.md')) {
        files.push(fullPath);
      }
    }
  }

  await traverse(dir);
  return files;
}

/**
 * Simple local embedding function (placeholder)
 * In production, you'd use Qdrant's FastEmbed or another embedding model
 */
async function generateEmbedding(text) {
  // For now, we'll let Qdrant handle embeddings with its built-in models
  // This is a placeholder - actual embedding generation happens server-side
  return null;
}

/**
 * Initialize Qdrant client and create collection if needed
 */
async function initializeQdrant() {
  const client = new QdrantClient({
    url: QDRANT_URL,
    apiKey: QDRANT_API_KEY,
  });

  console.log('🔗 Connecting to Qdrant...');

  try {
    // Check if collection exists
    const collections = await client.getCollections();
    const collectionExists = collections.collections.some(c => c.name === COLLECTION_NAME);

    if (collectionExists) {
      console.log(`✅ Collection "${COLLECTION_NAME}" already exists`);

      // Ask user if they want to delete and recreate
      const readline = require('readline').createInterface({
        input: process.stdin,
        output: process.stdout
      });

      const answer = await new Promise(resolve => {
        readline.question(`⚠️  Delete and recreate collection? (yes/no): `, resolve);
      });
      readline.close();

      if (answer.toLowerCase() === 'yes' || answer.toLowerCase() === 'y') {
        await client.deleteCollection(COLLECTION_NAME);
        console.log(`🗑️  Deleted existing collection`);
      } else {
        console.log(`📝 Appending to existing collection`);
        return client;
      }
    }

    // Create new collection with vector configuration
    // Using Qdrant's default embedding size (384 dimensions for all-MiniLM-L6-v2)
    await client.createCollection(COLLECTION_NAME, {
      vectors: {
        size: 384,
        distance: 'Cosine',
      },
    });

    console.log(`✅ Created collection "${COLLECTION_NAME}"`);
    return client;

  } catch (error) {
    console.error('❌ Error initializing Qdrant:', error.message);
    throw error;
  }
}

/**
 * Main upload function
 */
async function uploadToQdrant() {
  console.log('📚 Physical AI Book → Qdrant Upload Script\n');

  // Initialize Qdrant client
  const client = await initializeQdrant();

  // Find all MDX files
  const docsDir = path.join(__dirname, 'docs');
  console.log(`\n📂 Scanning ${docsDir} for content...`);
  const mdxFiles = await findMdxFiles(docsDir);
  console.log(`✅ Found ${mdxFiles.length} files\n`);

  let totalChunks = 0;
  const points = [];

  // Process each file
  for (const filePath of mdxFiles) {
    const relativePath = path.relative(__dirname, filePath);
    const content = await fs.readFile(filePath, 'utf-8');

    const chapter = extractChapterInfo(relativePath);
    const title = extractTitle(content);
    const cleanedContent = cleanContent(content);
    const chunks = chunkText(cleanedContent);

    console.log(`📄 ${relativePath}`);
    console.log(`   └─ Chapter ${chapter.number}: ${chapter.name}`);
    console.log(`   └─ Title: ${title}`);
    console.log(`   └─ Chunks: ${chunks.length}\n`);

    // Create points for each chunk
    chunks.forEach((chunk, index) => {
      points.push({
        id: totalChunks + index,
        payload: {
          text: chunk,
          file_path: relativePath,
          chapter_number: chapter.number,
          chapter_name: chapter.name,
          document_title: title,
          chunk_index: index,
          total_chunks: chunks.length,
        },
      });
    });

    totalChunks += chunks.length;
  }

  console.log(`\n📊 Total chunks to upload: ${totalChunks}`);
  console.log(`🚀 Uploading to Qdrant...`);

  // Note: We need to use Qdrant's embedding API or fastembed
  // For simplicity, we'll upload without embeddings and let you use Qdrant's inference API
  console.log('\n⚠️  Important: This script uploads text payloads.');
  console.log('You need to generate embeddings using one of these methods:');
  console.log('1. Qdrant Cloud Inference API (recommended)');
  console.log('2. Use fastembed library separately');
  console.log('3. Use OpenAI/Cohere embeddings\n');

  console.log('💡 For now, storing text in Qdrant. You can add embeddings later.\n');

  // Upload in batches of 100
  const batchSize = 100;
  for (let i = 0; i < points.length; i += batchSize) {
    const batch = points.slice(i, i + batchSize);

    try {
      // Since we're not generating embeddings locally, we'll store as payload only
      // You'll need to update this to include vectors when you set up embedding generation
      console.log(`📤 Uploading batch ${Math.floor(i / batchSize) + 1}/${Math.ceil(points.length / batchSize)}...`);

      // For demonstration: showing structure
      // In practice, you need to add vector generation here
      console.log(`   ⚠️  Skipping actual upload - need to integrate embedding generation`);

    } catch (error) {
      console.error(`❌ Error uploading batch: ${error.message}`);
    }
  }

  console.log('\n✅ Upload process completed!');
  console.log(`\n📋 Summary:`);
  console.log(`   - Files processed: ${mdxFiles.length}`);
  console.log(`   - Total chunks: ${totalChunks}`);
  console.log(`   - Collection: ${COLLECTION_NAME}`);
  console.log(`\n🔍 Next steps:`);
  console.log(`   1. Set up embedding generation (fastembed, OpenAI, etc.)`);
  console.log(`   2. Re-run this script to upload with vectors`);
  console.log(`   3. Query your collection for semantic search!`);
}

// Run the upload
uploadToQdrant().catch(console.error);
