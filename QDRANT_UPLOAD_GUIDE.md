# Qdrant Upload Guide

This guide explains how to upload your Physical AI & Humanoid Robotics book content to your Qdrant vector database cluster.

## 📋 Prerequisites

Your `.env` file is already configured with your Qdrant credentials. Make sure it contains:

```env
QDRANT_URL=https://your-cluster-url.qdrant.io:6333
QDRANT_API_KEY=your-api-key-here
QDRANT_COLLECTION_NAME=physical_ai_book
CHUNK_SIZE=600
CHUNK_OVERLAP=50
```

## 🐍 Option 1: Python Script (Recommended)

The Python script uses **FastEmbed** for local embedding generation - no external API calls needed!

### Installation

```bash
pip install qdrant-client fastembed python-dotenv
```

### Run the Upload

```bash
python upload-to-qdrant-python.py
```

### What it does:

1. ✅ Scans all `.mdx` files in the `docs/` directory
2. ✅ Cleans and chunks content (600 chars per chunk, 50 char overlap)
3. ✅ Generates embeddings locally using FastEmbed (BAAI/bge-small-en-v1.5)
4. ✅ Uploads vectors with metadata to Qdrant

### Expected Output:

```
📚 Physical AI Book → Qdrant Upload Script (Python + FastEmbed)

🔗 Connecting to Qdrant...
🤖 Loading FastEmbed model (BAAI/bge-small-en-v1.5)...
✅ Embedding model loaded (dimension: 384)

📂 Scanning docs/ for content...
✅ Found 20 files

📄 docs/01-introduction/what-is-physical-ai.mdx
   └─ Chapter 1: introduction
   └─ Title: What is Physical AI?
   └─ Chunks: 8
   └─ Generating embeddings... ✅

...

📊 Total chunks: 156
🚀 Uploading to Qdrant...

📤 Uploading batch 1/2 (100 points)...
📤 Uploading batch 2/2 (56 points)...

✅ Upload completed successfully!
```

---

## 🟨 Option 2: Node.js Script

The Node.js script is a basic implementation. For production use, you'll need to add embedding generation.

### Installation

Dependencies are already installed:
```bash
npm install
```

### Current Status

The `upload-to-qdrant.js` script currently:
- ✅ Reads and chunks your documentation
- ✅ Connects to Qdrant
- ✅ Prepares metadata
- ⚠️  Does NOT generate embeddings (you'll need to integrate an embedding provider)

### To Complete the Node.js Script:

You need to add one of these embedding providers:

1. **OpenAI** (recommended for quality):
   ```bash
   npm install openai
   ```

2. **Cohere** (generous free tier):
   ```bash
   npm install cohere-ai
   ```

3. **Qdrant Inference API** (if your cluster supports it)

---

## 📊 What Gets Uploaded

Each chunk is stored with the following metadata:

```json
{
  "text": "Chunk content...",
  "file_path": "docs/01-introduction/what-is-physical-ai.mdx",
  "chapter_number": 1,
  "chapter_name": "introduction",
  "document_title": "What is Physical AI?",
  "chunk_index": 0,
  "total_chunks": 8
}
```

## 🔍 Testing Your Collection

After upload, test semantic search:

### Python

```python
from qdrant_client import QdrantClient
from fastembed import TextEmbedding

client = QdrantClient(
    url="your-qdrant-url",
    api_key="your-api-key"
)

# Generate query embedding
embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")
query = "What is physical AI?"
query_vector = list(embedding_model.embed([query]))[0]

# Search
results = client.search(
    collection_name="physical_ai_book",
    query_vector=query_vector.tolist(),
    limit=5
)

for result in results:
    print(f"Score: {result.score:.3f}")
    print(f"File: {result.payload['file_path']}")
    print(f"Text: {result.payload['text'][:200]}...")
    print()
```

### Node.js

```javascript
const { QdrantClient } = require('@qdrant/js-client-rest');

const client = new QdrantClient({
  url: process.env.QDRANT_URL,
  apiKey: process.env.QDRANT_API_KEY,
});

// You'll need to generate query_vector using your embedding provider
const results = await client.search('physical_ai_book', {
  vector: query_vector,
  limit: 5,
});

results.forEach(result => {
  console.log(`Score: ${result.score}`);
  console.log(`File: ${result.payload.file_path}`);
  console.log(`Text: ${result.payload.text.substring(0, 200)}...`);
  console.log();
});
```

## 📈 Collection Statistics

After upload, you can check your collection:

```python
from qdrant_client import QdrantClient

client = QdrantClient(url="...", api_key="...")
info = client.get_collection("physical_ai_book")

print(f"Total vectors: {info.points_count}")
print(f"Vector dimension: {info.config.params.vectors.size}")
```

## 🔒 Security Notes

- ✅ Your `.env` file is already in `.gitignore`
- ✅ Never commit `.env` to version control
- ✅ The `.env.example` file has placeholder values for reference
- ⚠️  Rotate your API key if it's ever exposed

## 🚀 Next Steps

1. **Run the Python script** (recommended):
   ```bash
   python upload-to-qdrant-python.py
   ```

2. **Build a RAG application**: Use your Qdrant collection for semantic search and question-answering

3. **Integrate with your app**: Query the collection from your frontend/backend

## 🆘 Troubleshooting

### Error: "Missing required environment variables"
- Make sure your `.env` file exists and contains `QDRANT_URL` and `QDRANT_API_KEY`

### Error: "Connection refused"
- Check that your Qdrant cluster URL is correct
- Verify your cluster is running

### Error: "Invalid API key"
- Regenerate your API key in the Qdrant dashboard
- Update the `QDRANT_API_KEY` in your `.env` file

### Slow upload?
- The Python script processes embeddings locally, which can take time
- Expect ~30-60 seconds for 20 files with ~150 chunks
- You can adjust `CHUNK_SIZE` to create fewer, larger chunks

---

**Questions?** Check the [Qdrant Documentation](https://qdrant.tech/documentation/) or [FastEmbed Documentation](https://qdrant.github.io/fastembed/)
