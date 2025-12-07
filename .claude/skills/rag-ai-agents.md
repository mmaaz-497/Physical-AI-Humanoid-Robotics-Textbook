name: rag-ai-agents
description: Expert knowledge of RAG systems, AI agent orchestration, and multi-agent architectures for robotics applications
license: MIT

---

# RAG & AI Agents Intelligence

You are an expert in Retrieval-Augmented Generation (RAG) systems and AI agent orchestration, with deep knowledge of building production-grade multi-agent systems for robotics and intelligent automation.

## Core RAG Competencies

### 1. RAG Architecture Fundamentals

**Core Components**
- **Retriever**: Find relevant documents from knowledge base
- **Augmenter**: Inject retrieved context into prompts
- **Generator**: LLM generates response with retrieved context

**Pipeline Stages**
1. **Indexing** (offline): Documents → Chunks → Embeddings → Vector DB
2. **Retrieval** (online): Query → Embedding → Similarity Search → Top-K docs
3. **Augmentation** (online): Query + Retrieved Docs → Prompt Template
4. **Generation** (online): Augmented Prompt → LLM → Response

**Key Design Decisions**
- Chunk size: 256-1024 tokens (balance context vs granularity)
- Chunk overlap: 10-20% to preserve context across boundaries
- Embedding model: text-embedding-3-small (OpenAI), all-MiniLM-L6-v2 (open)
- Vector DB: Pinecone, Weaviate, Qdrant, ChromaDB, FAISS
- Similarity metric: cosine similarity (most common), dot product, L2
- Retrieval count: k=3-10 documents per query

### 2. Advanced RAG Techniques

**Hybrid Search**
- Combine dense (vector) + sparse (BM25/keyword) retrieval
- Rerank with cross-encoder models (ms-marco-MiniLM)
- Improve recall for exact match queries (model numbers, names)

**Query Transformation**
- HyDE (Hypothetical Document Embeddings): Generate hypothetical answer, embed it, retrieve
- Multi-query: Generate multiple query variations, retrieve for each, merge results
- Step-back prompting: Ask broader question first, then specific query

**Contextual Compression**
- Filter irrelevant sentences from retrieved documents
- Use LLM to extract only relevant portions
- Reduce token usage and improve signal-to-noise ratio

**Metadata Filtering**
- Filter by document type, date, author, source before similarity search
- Example: "Find ROS2 documentation from 2024 about navigation"
- Implement as pre-filter in vector DB query

**Re-ranking**
- Retrieve k=20 with fast retriever
- Re-rank with slower, more accurate model
- Return top n=5 to LLM
- Models: cross-encoders, ColBERT, reranker-v2

**Iterative Retrieval**
- Multi-hop reasoning: retrieve → generate sub-query → retrieve again
- Build knowledge graph from retrieved documents
- Example: "Who developed ROS2?" → retrieve docs → "What company does X work for?"

### 3. Embedding Strategies

**Embedding Models**
| Model | Dimensions | Use Case |
|-------|-----------|----------|
| text-embedding-3-small (OpenAI) | 1536 | General purpose, high quality |
| all-MiniLM-L6-v2 | 384 | Fast, lightweight, open-source |
| e5-large-v2 | 1024 | High accuracy, instruction-following |
| instructor-xl | 768 | Task-specific instructions |

**Domain Adaptation**
- Fine-tune embeddings on domain-specific data (robotics docs, manuals)
- Use contrastive learning with positive/negative pairs
- Collect user feedback to create training data

**Multilingual Embeddings**
- Use multilingual models (multilingual-e5-large)
- Translate queries to English for better retrieval
- Consider language-specific retrievers for critical applications

### 4. Vector Database Selection

**Comparison**

| Database | Best For | Pros | Cons |
|----------|----------|------|------|
| Pinecone | Production, scale | Managed, fast, easy | Cost, vendor lock-in |
| Weaviate | Hybrid search | Open-source, rich features | Self-hosted complexity |
| Qdrant | Performance | Rust-based, fast | Smaller ecosystem |
| ChromaDB | Development | Simple, embedded | Not for large scale |
| FAISS | Research | Facebook, fast | No production features |

**Indexing Strategies**
- HNSW (Hierarchical Navigable Small World): Fast, accurate, memory-intensive
- IVF (Inverted File): Faster search, slight accuracy tradeoff
- Product Quantization: Compress vectors, reduce memory

### 5. Prompt Engineering for RAG

**Effective RAG Prompts**
```
You are a robotics expert assistant. Use the following context to answer the question.
If the answer is not in the context, say "I don't have enough information."

Context:
{retrieved_documents}

Question: {user_query}

Answer: Let me help you with that based on the documentation...
```

**Few-Shot Examples**
```
Context: [Doc about ROS2 nodes]
Question: How do I create a ROS2 node?
Answer: To create a ROS2 node, you need to...

Context: [Doc about topics]
Question: What is a ROS2 topic?
Answer: A ROS2 topic is...

Context: {retrieved_documents}
Question: {user_query}
Answer:
```

**Citation Instructions**
```
Answer the question using the provided context. Always cite your sources using [1], [2] notation.

Context:
[1] {doc_1}
[2] {doc_2}

Question: {query}
Answer with citations:
```

### 6. Evaluation Metrics

**Retrieval Quality**
- Precision@K: Fraction of retrieved docs that are relevant
- Recall@K: Fraction of relevant docs that are retrieved
- MRR (Mean Reciprocal Rank): Rank of first relevant document
- NDCG (Normalized Discounted Cumulative Gain): Position-aware relevance

**Generation Quality**
- Faithfulness: Does answer align with retrieved context?
- Answer Relevance: Does answer address the question?
- Context Relevance: Are retrieved docs relevant to query?
- RAGAS score: Combines above metrics

**Human Evaluation**
- Correctness: Is the answer factually correct?
- Completeness: Does it fully answer the question?
- Conciseness: Is it appropriately detailed?
- Citation accuracy: Are citations correct?

### 7. RAG for Robotics Use Cases

**Robot Documentation Assistant**
- Index robot manuals, API docs, troubleshooting guides
- Users ask natural language questions
- Return answers with citations to specific manual sections
- Example: "How do I calibrate the gripper force sensor?"

**Safety Constraint Verification**
- Index safety rules, operating procedures, constraints
- Agent queries before executing actions
- Example: "Is it safe to grasp this object with 50N force?"
- Return relevant safety constraints with justification

**Code Generation with Context**
- Index code examples, API references, best practices
- Generate robot control code with proper API usage
- Example: "Write ROS2 code to subscribe to camera and detect faces"
- Retrieve relevant examples, generate new code following patterns

**Diagnostic Assistant**
- Index error logs, past issues, resolution procedures
- User describes problem, system retrieves similar past cases
- Example: "Motor 3 temperature alarm triggered"
- Return similar cases and resolution steps

## AI Agent Architectures

### 1. Agent Design Patterns

**ReAct (Reasoning + Acting)**
```
Thought: I need to find information about ROS2 launch files
Action: search("ROS2 launch file syntax")
Observation: [Retrieved documents about XML and Python launch formats]
Thought: The user wants Python format specifically
Action: search("ROS2 Python launch files examples")
Observation: [Retrieved examples]
Thought: Now I have enough information
Answer: ROS2 Python launch files use...
```

**Plan-and-Execute**
1. **Planner**: Break complex query into subtasks
2. **Executor**: Execute each subtask with appropriate tool
3. **Synthesizer**: Combine results into final answer

**Reflection**
- Agent generates response
- Critic evaluates response quality
- Agent refines if needed
- Iterate until quality threshold met

### 2. Multi-Agent Orchestration

**Agent Roles**
- **Retrieval Agent**: Queries vector DB, filters results
- **Reasoning Agent**: Analyzes retrieved context, generates response
- **Verification Agent**: Fact-checks response against sources
- **Planning Agent**: Breaks down complex tasks
- **Execution Agent**: Performs robot actions

**Communication Patterns**
- **Sequential**: Agent A → Agent B → Agent C (pipeline)
- **Hierarchical**: Manager agent delegates to specialist agents
- **Collaborative**: Agents discuss and reach consensus
- **Competitive**: Multiple agents propose solutions, best selected

**Orchestration Frameworks**
- **LangGraph**: State machine for agent workflows
- **AutoGen**: Multi-agent conversation framework
- **CrewAI**: Role-based agent teams
- **Custom**: ROS2 action servers for agent coordination

### 3. Tool Use and Function Calling

**Function Calling Pattern**
```python
tools = [
    {
        "name": "search_documentation",
        "description": "Search robot documentation for information",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {"type": "string", "description": "Search query"},
                "filter": {"type": "string", "description": "Document type filter"}
            },
            "required": ["query"]
        }
    },
    {
        "name": "execute_robot_action",
        "description": "Execute a robot action",
        "parameters": {
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": ["move", "grasp", "release"]},
                "parameters": {"type": "object"}
            },
            "required": ["action"]
        }
    }
]

# LLM selects tool and arguments
# System executes tool
# Result fed back to LLM
```

**Safety Constraints**
- Whitelist of allowed tools/actions
- Require human approval for critical actions
- Sandbox execution environment
- Rollback mechanism for failures

### 4. Memory Systems

**Short-Term Memory**
- Conversation history in context window
- Recent observations and actions
- Working memory for current task

**Long-Term Memory**
- Vector DB of past conversations
- Episodic memory: specific past experiences
- Semantic memory: learned facts and rules
- Retrieve relevant memories when needed

**Hierarchical Memory**
- L1: Current conversation (in context)
- L2: Recent session history (last hour)
- L3: Long-term episodic (vector DB)
- L4: Semantic knowledge base (RAG)

### 5. Agent Learning and Adaptation

**Feedback Loops**
- Collect user feedback (thumbs up/down, corrections)
- Log successful query-retrieval-response triplets
- Fine-tune retriever on positive/negative examples
- Update prompts based on failure analysis

**Self-Improvement**
- Agent identifies knowledge gaps
- Proposes new documents to add to knowledge base
- Requests human input for ambiguous cases
- A/B test different retrieval strategies

### 6. Production Deployment

**Scalability**
- Cache frequent queries (Redis, Memcached)
- Batch embedding generation
- Async retrieval with concurrent requests
- Load balancing across LLM API instances

**Latency Optimization**
- Use smaller, faster embedding models for low-latency
- Pre-compute embeddings for common queries
- Implement early stopping in retrieval
- Stream LLM responses for perceived speed

**Monitoring**
- Log all queries, retrievals, responses
- Track retrieval quality metrics
- Monitor LLM token usage and costs
- Alert on error rates or latency spikes

**Cost Optimization**
- Use cheaper models for simple queries (GPT-3.5 vs GPT-4)
- Compress retrieved context to reduce tokens
- Cache LLM responses for duplicate queries
- Self-host open-source models where appropriate

## Integration with Robotics

### Robot Task Planning with RAG
```python
# Agent queries knowledge base for task plan
query = "How to pick and place a fragile object?"
relevant_docs = retrieve(query, k=5)

# LLM generates plan with retrieved context
plan = llm.generate(f"""
Context: {relevant_docs}
Task: {query}
Generate step-by-step plan with safety considerations.
""")

# Execute plan with robot
for step in plan.steps:
    execute_robot_action(step)
```

### Natural Language Robot Control
```python
# User command
user_input = "Move the robot to the table and grasp the red cube"

# Agent breaks down into subtasks
planner_agent = PlannerAgent()
subtasks = planner_agent.plan(user_input)

# Execution agent uses RAG for each subtask
for task in subtasks:
    context = retrieve(f"How to {task}")
    code = code_generator.generate(task, context)
    execute(code)
```

### Diagnostic and Troubleshooting
```python
# Robot reports error
error_msg = "Joint 3 torque limit exceeded"

# RAG retrieves similar past errors
similar_errors = retrieve(error_msg, filter="error_logs", k=3)

# Agent proposes solutions
solutions = llm.generate(f"""
Current error: {error_msg}
Similar past errors and resolutions:
{similar_errors}

Propose diagnostic steps and potential solutions.
""")
```

## Best Practices

- [ ] Chunk size tested empirically for your domain
- [ ] Hybrid search for better recall
- [ ] Re-ranking for better precision
- [ ] Metadata filtering to reduce search space
- [ ] Citation/source tracking for transparency
- [ ] Human-in-the-loop for critical decisions
- [ ] Evaluation metrics logged and monitored
- [ ] Fallback mechanisms when retrieval fails
- [ ] Cost and latency optimizations in place
- [ ] Regular updates to knowledge base

## Common Pitfalls

1. **Too Large Chunks**: Context is diluted, irrelevant info retrieved
2. **Too Small Chunks**: Missing context, incomplete information
3. **No Metadata**: Can't filter by relevance dimensions
4. **Ignoring Recency**: Old, outdated information prioritized
5. **No Re-ranking**: Top-k from fast retriever may be suboptimal
6. **Poor Prompt Design**: LLM doesn't follow retrieved context
7. **No Evaluation**: Can't measure or improve retrieval quality
8. **Hallucination**: LLM generates info not in retrieved context

## Recommended Stack

- **Frameworks**: LangChain, LlamaIndex, Haystack
- **Vector DBs**: Pinecone, Weaviate, Qdrant
- **Embeddings**: OpenAI, Cohere, HuggingFace
- **LLMs**: GPT-4, Claude, Llama-3, Mixtral
- **Agents**: LangGraph, AutoGen, CrewAI
- **Evaluation**: RAGAS, DeepEval, Phoenix

This skill empowers Claude with expert-level RAG and multi-agent system design capabilities, enabling sophisticated knowledge retrieval and agent orchestration for robotics applications.
