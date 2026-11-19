# 确定 LLM 返回的 json 消息的固定模板
SCHEMA_TEXT = """{
  "task_id": "string",
  "action": "pick_and_place",
  "object": {
    "id": "string",
    "source": "pick_station"
  },
  "destination": {
    "type": "tray",
    "color": "red | green | blue",
    "location": "place_station"
  }
}"""

SYSTEM_TEMPLATE = """你是移动操作机器人的任务解析器。只输出 JSON，不要解释，不要 markdown 代码块。

输出必须严格符合下面的结构，字段名、层级、枚举值都要一致：
{schema}

可用的 waypoint：
{waypoints}

可用的托盘颜色：{colors}

规则：
1. action 只能是 "pick_and_place"。
2. object.source 固定为 "pick_station"。
3. destination.type 固定为 "tray"。
4. destination.location 固定为 "place_station"。
5. destination.color 只能是上面列出的颜色。
6. 无法确定时输出 {{"action":"unknown","reason":"..."}}。

示例：
用户：把红色积木放到绿色盘子里
输出：{example}
"""

# 辅助函数 --- 从导航点中选出包含有 x, y, yaw 信息的有效导航点
def _stations(waypoints: dict) -> dict:
    return {
        name: info
        for name, info in waypoints.items()
        if isinstance(info, dict) and "x" in info and "y" in info and "yaw" in info
    }


def build_system_prompt(waypoints: dict, colors: list[str]) -> str:
    # 获取传入的有效导航点
    stations = _stations(waypoints)
    lines = [f"- {name}：{info.get('description', name)}" for name, info in stations.items()]

    # 找有效导航点中带有 pick 和 place 的，找不到就用默认值
    pick = next((n for n in stations if "pick" in n), "pick_station")
    place = next((n for n in stations if "place" in n), "place_station")

    # 动态生成一个提示词模板，让 LLM 更容易模仿
    example = (
        '{"task_id":"pick_irregular_object_to_green_tray",'
        '"action":"pick_and_place",'
        '"object":{"id":"irregular_object","source":"%s"},'
        '"destination":{"type":"tray","color":"green","location":"%s"}}'
    ) % (pick, place)

    return SYSTEM_TEMPLATE.format(
        schema=SCHEMA_TEXT,
        waypoints="\n".join(lines),
        colors=", ".join(colors),
        example=example,
    )