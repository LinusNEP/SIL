import os
import time
import requests
from openai import OpenAI
import rospy

def _retry_with_backoff(fn, max_retries=2, base_delay=1.0, provider_name="LLM"):
    last_error = None
    for attempt in range(1 + max_retries):
        try:
            return fn()
        except Exception as e:
            last_error = e
            if attempt < max_retries:
                delay = base_delay * (2 ** attempt)
                rospy.logwarn(
                    f"[{provider_name}] Attempt {attempt + 1} failed: {e}  "
                    f"— retrying in {delay:.1f}s"
                )
                time.sleep(delay)
    raise last_error

class BaseLLMProvider:
    max_retries = 2
    retry_base_delay = 1.0

    def chat(self, system_message: str, user_prompt: str) -> str:
        raise NotImplementedError

class OpenAIProvider(BaseLLMProvider):
    def __init__(self, api_key, model_name, max_tokens, temperature,
                 endpoint=None, timeout=30):
        kwargs = {"api_key": api_key}
        if endpoint:
            kwargs["base_url"] = endpoint
        kwargs["timeout"] = timeout
        self.client = OpenAI(**kwargs)
        self.model_name = model_name
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.timeout = timeout

    def chat(self, system_message, user_prompt):
        def _call():
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=self.max_tokens,
                temperature=self.temperature
            )
            content = response.choices[0].message.content
            return content.strip() if content else ""

        try:
            return _retry_with_backoff(_call, max_retries=self.max_retries, base_delay=self.retry_base_delay, provider_name="OpenAI")
        except Exception as e:
            rospy.logerr(f"OpenAI API error: {e}")
            return "I'm sorry, I couldn't process that request."


class DeepSeekProvider(BaseLLMProvider):
    DEFAULT_ENDPOINT = "https://api.deepseek.com/v1/chat/completions"

    def __init__(self, api_key, model_name, max_tokens, temperature,
                 endpoint=None, timeout=30):
        self.api_key = api_key
        self.model_name = model_name
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.endpoint = endpoint or self.DEFAULT_ENDPOINT
        self.timeout = timeout

    def chat(self, system_message, user_prompt):
        def _call():
            headers = {
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }
            payload = {
                "model": self.model_name,
                "messages": [
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": user_prompt}
                ],
                "max_tokens": self.max_tokens,
                "temperature": self.temperature
            }
            response = requests.post(
                self.endpoint, headers=headers, json=payload,
                timeout=self.timeout
            )
            response.raise_for_status()
            data = response.json()
            content = data["choices"][0]["message"]["content"]
            return content.strip() if content else ""

        try:
            return _retry_with_backoff(_call, max_retries=self.max_retries, base_delay=self.retry_base_delay, provider_name="DeepSeek")
        except Exception as e:
            rospy.logerr(f"DeepSeek API error: {e}")
            return "I'm sorry, I couldn't process that request."


class LlamaCppProvider(BaseLLMProvider):
    DEFAULT_ENDPOINT = "http://localhost:8000/completion"

    def __init__(self, api_key_unused, model_name_unused, max_tokens, temperature,
                 endpoint=None, timeout=30):
        self.endpoint = endpoint or self.DEFAULT_ENDPOINT
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.timeout = timeout

    def chat(self, system_message, user_prompt):
        def _call():
            payload = {
                "prompt": f"{system_message}\n{user_prompt}",
                "n_predict": self.max_tokens,
                "temperature": self.temperature,
            }
            response = requests.post(
                self.endpoint, json=payload, timeout=self.timeout
            )
            response.raise_for_status()
            data = response.json()
            content = data.get("content", "")
            return content.strip() if content else ""

        try:
            return _retry_with_backoff(_call, max_retries=self.max_retries, base_delay=self.retry_base_delay, provider_name="Llama.cpp")
        except Exception as e:
            rospy.logerr(f"Llama.cpp API error: {e}")
            return "I'm sorry, I couldn't process that request."


class ClaudeProvider(BaseLLMProvider):
    DEFAULT_ENDPOINT = "https://api.anthropic.com/v1/messages"

    def __init__(self, api_key, model_name, max_tokens, temperature,
                 endpoint=None, timeout=30):
        self.api_key = api_key
        self.model_name = model_name
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.endpoint = endpoint or self.DEFAULT_ENDPOINT
        self.timeout = timeout

    def chat(self, system_message, user_prompt):
        def _call():
            headers = {
                "x-api-key": self.api_key,
                "anthropic-version": "2023-06-01",
                "Content-Type": "application/json"
            }
            payload = {
                "model": self.model_name,
                "max_tokens": self.max_tokens,
                "temperature": self.temperature,
                "system": system_message,
                "messages": [
                    {"role": "user", "content": user_prompt}
                ]
            }
            response = requests.post(
                self.endpoint, headers=headers, json=payload,
                timeout=self.timeout
            )
            response.raise_for_status()
            data = response.json()
            content = data["content"][0]["text"]
            return content.strip() if content else ""

        try:
            return _retry_with_backoff(_call, max_retries=self.max_retries, base_delay=self.retry_base_delay, provider_name="Claude")
        except Exception as e:
            rospy.logerr(f"Claude API error: {e}")
            return "I'm sorry, I couldn't process that request."


class GeminiProvider(BaseLLMProvider):
    DEFAULT_ENDPOINT = "https://generativelanguage.googleapis.com/v1beta/models"

    def __init__(self, api_key, model_name, max_tokens, temperature,
                 endpoint=None, timeout=30):
        self.api_key = api_key
        self.model_name = model_name or "gemini-pro"
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.endpoint = endpoint or self.DEFAULT_ENDPOINT
        self.timeout = timeout

    def chat(self, system_message, user_prompt):
        def _call():
            url = (
                f"{self.endpoint}/{self.model_name}:generateContent"
                f"?key={self.api_key}"
            )
            payload = {
                "contents": [{
                    "parts": [
                        {"text": system_message},
                        {"text": user_prompt}
                    ]
                }],
                "generationConfig": {
                    "maxOutputTokens": self.max_tokens,
                    "temperature": self.temperature
                }
            }
            response = requests.post(url, json=payload, timeout=self.timeout)
            response.raise_for_status()
            data = response.json()
            content = data["candidates"][0]["content"]["parts"][0]["text"]
            return content.strip() if content else ""

        try:
            return _retry_with_backoff(_call, max_retries=self.max_retries, base_delay=self.retry_base_delay, provider_name="Gemini")
        except Exception as e:
            rospy.logerr(f"Gemini API error: {e}")
            return "I'm sorry, I couldn't process that request."

class LLMProviderFactory:
    provider_classes = {
        "openai": OpenAIProvider,
        "deepseek": DeepSeekProvider,
        "llama.cpp": LlamaCppProvider,
        "claude": ClaudeProvider,
        "gemini": GeminiProvider,
    }

    @classmethod
    def create_provider(cls, provider_name, api_key, model_name, max_tokens,
                        temperature, endpoint=None, timeout=30,
                        max_retries=2, retry_base_delay=1.0):
        if provider_name not in cls.provider_classes:
            raise ValueError(f"Unsupported LLM provider: {provider_name}")
        provider_class = cls.provider_classes[provider_name]
        provider = provider_class(
            api_key, model_name, max_tokens, temperature,
            endpoint=endpoint or None,
            timeout=timeout
        )
        provider.max_retries = max_retries
        provider.retry_base_delay = retry_base_delay
        return provider
