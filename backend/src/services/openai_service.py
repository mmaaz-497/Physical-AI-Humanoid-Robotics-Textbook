"""
OpenAI service for answer generation with strict grounding constraints.
"""

import logging
from typing import List
from openai import OpenAI, RateLimitError
import time

from src.config import settings
from src.models.retrieval import RetrievedChunk
from src.utils.exceptions import OpenAIRateLimitError

logger = logging.getLogger(__name__)

# System prompt enforcing zero hallucinations
GROUNDING_SYSTEM_PROMPT = """You are a helpful assistant answering questions about the Physical AI & Humanoid Robotics textbook.

CRITICAL RULES:
1. Answer ONLY using the provided context below. Do NOT use external knowledge.
2. If the context does not contain the answer, respond with: "I couldn't find information on this topic in the textbook."
3. Cite your sources by referencing the chapter name in your answer (e.g., "According to Module 1: ROS 2 Fundamentals, ...").
4. Be concise and accurate.
5. If the question is completely unrelated to the textbook content (e.g., current events, general knowledge), respond with: "I can only answer questions based on the Physical AI & Humanoid Robotics textbook content."

Context:
{context}

Question: {question}

Answer:"""


class OpenAIService:
    """Service for generating answers using OpenAI GPT models."""

    def __init__(self):
        """Initialize OpenAI client."""
        self.client = OpenAI(api_key=settings.OPENAI_API_KEY)
        self.model = settings.OPENAI_MODEL
        logger.info(f"Initialized OpenAI service with model: {self.model}")

    def generate_answer(
        self,
        question: str,
        context_chunks: List[RetrievedChunk],
    ) -> str:
        """
        Generate answer from retrieved context chunks with strict grounding.

        Args:
            question: The user's question
            context_chunks: List of retrieved chunks from Qdrant

        Returns:
            Generated answer string

        Raises:
            OpenAIRateLimitError: If rate limit is exceeded
        """
        # Format context from chunks
        context_text = self._format_context(context_chunks)

        # Build prompt
        prompt = GROUNDING_SYSTEM_PROMPT.format(
            context=context_text,
            question=question,
        )

        return self._call_openai_with_retry(prompt)

    def generate_answer_from_selection(
        self,
        question: str,
        selection_text: str,
    ) -> str:
        """
        Generate answer using only highlighted text (bypasses Qdrant retrieval).

        Args:
            question: The user's question
            selection_text: Highlighted text from the page

        Returns:
            Generated answer string

        Raises:
            OpenAIRateLimitError: If rate limit is exceeded
        """
        # Use selection text as context
        prompt = GROUNDING_SYSTEM_PROMPT.format(
            context=selection_text,
            question=question,
        )

        logger.info("Generating answer from selection text (bypassing Qdrant)")
        return self._call_openai_with_retry(prompt)

    def _format_context(self, chunks: List[RetrievedChunk]) -> str:
        """
        Format retrieved chunks into context string.

        Args:
            chunks: List of retrieved chunks

        Returns:
            Formatted context string
        """
        if not chunks:
            return "No relevant context found."

        context_parts = []
        for i, chunk in enumerate(chunks, 1):
            chapter = chunk.metadata.get("chapter", "Unknown Chapter")
            context_parts.append(f"[Source {i} - {chapter}]\n{chunk.chunk_text}\n")

        return "\n".join(context_parts)

    def _call_openai_with_retry(self, prompt: str) -> str:
        """
        Call OpenAI API with retry logic for rate limits.

        Args:
            prompt: Complete prompt including system instructions and context

        Returns:
            Generated answer

        Raises:
            OpenAIRateLimitError: If rate limit persists after retries
        """
        max_retries = 3
        retry_delay = 1.0

        for attempt in range(max_retries):
            try:
                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "user", "content": prompt}
                    ],
                    temperature=0.3,  # Low temperature for consistency
                    max_tokens=500,
                )

                answer = response.choices[0].message.content.strip()

                # Log token usage
                usage = response.usage
                logger.info(
                    f"OpenAI API call successful. "
                    f"Tokens: {usage.prompt_tokens} prompt + {usage.completion_tokens} completion = {usage.total_tokens} total"
                )

                return answer

            except RateLimitError as e:
                if attempt < max_retries - 1:
                    wait_time = retry_delay * (2 ** attempt)
                    logger.warning(
                        f"Rate limit hit (attempt {attempt + 1}/{max_retries}). "
                        f"Waiting {wait_time}s..."
                    )
                    time.sleep(wait_time)
                else:
                    logger.error(f"Rate limit exceeded after {max_retries} attempts")
                    raise OpenAIRateLimitError(
                        "OpenAI API rate limit exceeded. Please try again later.",
                        retry_after=60,
                    )

            except Exception as e:
                logger.error(f"OpenAI API error: {str(e)}")
                raise

        return "I encountered an error generating the answer. Please try again."

    def test_connection(self) -> bool:
        """
        Test OpenAI API connection.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            # Make a minimal API call to test
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": "Test"}],
                max_tokens=5,
            )
            logger.info("OpenAI API connection test successful")
            return True
        except Exception as e:
            logger.error(f"OpenAI API connection test failed: {str(e)}")
            return False
