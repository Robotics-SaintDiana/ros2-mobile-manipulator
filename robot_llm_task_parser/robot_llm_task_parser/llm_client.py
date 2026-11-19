import os
from typing import Union
from openai import OpenAI
from pydantic import ValidationError
from robot_llm_task_parser.schemas import TaskPlan, UnknownTask


class LLMError(RuntimeError):
    pass


class LLMClient:
    def __init__(self, base_url: str, model: str, api_key_env: str,
                 system_prompt: str, temperature: float = 0.0,
                 timeout: float = 15.0, max_attempts: int = 3):
        # 从 .env 文件中获取 LLM 的 API KEY
        api_key = os.getenv(api_key_env)
        if not api_key:
            raise LLMError(f"环境变量 {api_key_env} 未设置")

        # 根据传入的参数，利用 openai 官方库创建客户端
        self.client = OpenAI(base_url=base_url, api_key=api_key)
        self.model = model
        self.system_prompt = system_prompt
        self.temperature = temperature
        self.timeout = timeout
        self.max_attempts = max_attempts

    # 把传入的自然语言转换成结构化对象
    def parse(self, command: str) -> Union[TaskPlan, UnknownTask]:
        # 构造发给大模型的初始对话消息列表，包括给系统的提示词以及用户的指令
        messages = [
            {"role": "system", "content": self.system_prompt},
            {"role": "user", "content": command},
        ]

        # 最多尝试调用大模型 max_attempts 次
        last_error = None
        for _ in range(self.max_attempts):
            try:
                # 调用模型
                resp = self.client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    temperature=self.temperature,
                    response_format={"type": "json_object"},  # 强制模型的输出格式为 json
                    timeout=self.timeout,
                )
            except Exception as e:
                raise LLMError(f"LLM 请求失败: {e}") from e

            # 输出 LLM 的返回结果方便调试
            raw = resp.choices[0].message.content
            print(f"[raw] {raw}")   

            # 优先尝试解析成 TaskPlan，失败后再尝试解析成 UnknownTask
            # 如果两者都解析失败，把错误输出追加到对话历史让模型自我纠错
            try:
                return TaskPlan.model_validate_json(raw)
            except ValidationError as e:
                try:
                    return UnknownTask.model_validate_json(raw)
                except ValidationError:
                    last_error = e
                    messages.append({"role": "assistant", "content": raw})
                    messages.append({
                        "role": "user",
                        "content": f"上面的 JSON 不合法：{e}。只输出合法 JSON。",
                    })

        raise LLMError(f"{self.max_attempts} 次输出都不合法: {last_error}")