#!/usr/bin/env python
"""Skill registry for SIL.
"""
from __future__ import annotations
import re
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

@dataclass
class Param:
    name: str
    type: type = float
    default: Any = None
    description: str = ""

@dataclass
class Skill:
    name: str                                
    description: str                          
    call: Callable[[Any, Dict], bool]          
    params: List[Param] = field(default_factory=list)
    aliases: List[str] = field(default_factory=list)
    enabled: bool = True

    def signature(self) -> str:
        inner = ", ".join(p.name for p in self.params)
        return f"{self.name}({inner})" if inner else self.name


def _ok(value) -> bool:
    return True if value is None else bool(value)


def normalize_coordinates(value) -> Dict[str, float]:
    x = y = z = 0.0
    if isinstance(value, dict):
        x = float(value.get("x", 0.0) or 0.0)
        y = float(value.get("y", 0.0) or 0.0)
        z = float(value.get("z", 0.0) or 0.0)
    elif isinstance(value, (list, tuple)):
        nums = []
        for v in value:
            try:
                nums.append(float(v))
            except (TypeError, ValueError):
                pass
        x = nums[0] if len(nums) > 0 else 0.0
        y = nums[1] if len(nums) > 1 else 0.0
        z = nums[2] if len(nums) > 2 else 0.0
    elif isinstance(value, str):
        parsed = SkillRegistry._parse_sequence(value) if value.strip() else []
        return normalize_coordinates(parsed)
    return {"x": x, "y": y, "z": z}

class SkillRegistry:
    def __init__(self):
        self._skills: Dict[str, Skill] = {}
        self._alias_index: Dict[str, str] = {}

    def register(self, skill: Skill):
        self._skills[skill.name.upper()] = skill
        self._alias_index[skill.name.lower()] = skill.name.upper()
        for alias in skill.aliases:
            self._alias_index[alias.lower()] = skill.name.upper()

    def get(self, name: str) -> Optional[Skill]:
        if not name:
            return None
        key = name.upper()
        if key in self._skills and self._skills[key].enabled:
            return self._skills[key]
        canon = self._alias_index.get(name.lower())
        if canon and self._skills[canon].enabled:
            return self._skills[canon]
        return None

    def names(self) -> List[str]:
        return [n for n, s in self._skills.items() if s.enabled]

    def filter(self, enabled: Optional[List[str]] = None,
               disabled: Optional[List[str]] = None):
        if enabled:
            enabled_upper = {n.upper() for n in enabled}
            for name, skill in self._skills.items():
                skill.enabled = name in enabled_upper
        if disabled:
            for name in disabled:
                if name.upper() in self._skills:
                    self._skills[name.upper()].enabled = False

    def execute(self, executor, action: Dict) -> Optional[bool]:
        skill = self.get(action.get("action", ""))
        if skill is None:
            return None
        return _ok(skill.call(executor, action))
    
    def describe_for_prompt(self) -> str:
        lines = []
        for skill in self._skills.values():
            if not skill.enabled:
                continue
            if skill.params:
                args = ", ".join(
                    f"{p.name}" + (f"={p.default}" if p.default is not None else "")
                    for p in skill.params
                )
                lines.append(f"- {skill.name}({args}) — {skill.description}")
            else:
                lines.append(f"- {skill.name} — {skill.description}")
        return "\n".join(lines)

    _CALL_RE = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)\s*(?:\((.*)\))?\s*$", re.DOTALL)

    @staticmethod
    def _split_top_level(arg_str: str) -> List[str]:
        parts, depth, quote, buf = [], 0, None, []
        for ch in arg_str:
            if quote:
                buf.append(ch)
                if ch == quote:
                    quote = None
            elif ch in "'\"":
                quote = ch
                buf.append(ch)
            elif ch in "([{":
                depth += 1
                buf.append(ch)
            elif ch in ")]}":
                depth = max(0, depth - 1)
                buf.append(ch)
            elif ch == "," and depth == 0:
                parts.append("".join(buf))
                buf = []
            else:
                buf.append(ch)
        if buf:
            parts.append("".join(buf))
        return [p.strip() for p in parts if p.strip()]

    def parse_structured(self, text: str) -> Optional[Dict]:
        if not text:
            return None
        m = self._CALL_RE.match(text.strip())
        if not m:
            return None
        name, arg_str = m.group(1), m.group(2) or ""
        skill = self.get(name)
        if skill is None:
            return None

        action: Dict[str, Any] = {"action": skill.name}
        param_by_name = {p.name: p for p in skill.params}
        positional: List[Any] = []

        for token in self._split_top_level(arg_str):
            if "=" in token and re.match(r"^\s*[A-Za-z_][A-Za-z0-9_]*\s*=", token):
                key, raw = token.split("=", 1)
                key = key.strip()
                p = param_by_name.get(key)
                action[key] = self._coerce(raw.strip(), p.type if p else str)
            else:
                positional.append(token.strip())
        for value, p in zip(positional, skill.params):
            if p.name not in action:
                action[p.name] = self._coerce(value, p.type)
        for p in skill.params:
            if p.name not in action and p.default is not None:
                action[p.name] = p.default
        return action

    @staticmethod
    def _coerce(raw: str, typ: type):
        raw = raw.strip()
        if raw and raw[0] in "([{" and raw[-1] in ")]}":
            return SkillRegistry._parse_sequence(raw)
        raw = raw.strip("'\"")
        try:
            if typ is float:
                return float(raw)
            if typ is int:
                return int(float(raw))
            if typ is bool:
                return raw.lower() in ("1", "true", "yes", "on")
            if typ is dict:
                return SkillRegistry._parse_sequence(raw)
        except (TypeError, ValueError):
            return raw
        return raw

    @staticmethod
    def _parse_sequence(raw: str):
        inner = raw.strip().strip("()[]{}").strip()
        if not inner:
            return []
        items = [s.strip() for s in inner.split(",") if s.strip()]
        if all((":" in it or "=" in it) for it in items):
            out = {}
            for it in items:
                sep = ":" if ":" in it else "="
                k, v = it.split(sep, 1)
                try:
                    out[k.strip().strip("'\"")] = float(v.strip())
                except ValueError:
                    out[k.strip().strip("'\"")] = v.strip().strip("'\"")
            return out
        seq = []
        for it in items:
            try:
                seq.append(float(it))
            except ValueError:
                seq.append(it.strip("'\""))
        return seq

def build_registry(defaults: Optional[Dict] = None) -> SkillRegistry:
    d = defaults or {}
    fwd = d.get("forward", 1.0)
    back = d.get("backward", 1.0)
    turn = d.get("turn", 90.0)
    rot = d.get("rotate", 360.0)
    circ = d.get("circle_radius", 0.5)

    reg = SkillRegistry()
    reg.register(Skill(     
        "FORWARD", "Move forward `distance` metres",
        call=lambda ex, a: ex.move_linear(float(a.get("distance", fwd)), a.get("speed")),
        params=[Param("distance", float, fwd, "metres")],
        aliases=["move_forward", "go_forward"]))

    reg.register(Skill(
        "BACKWARD", "Move backward `distance` metres",
        call=lambda ex, a: ex.move_linear(-float(a.get("distance", back)), a.get("speed")),
        params=[Param("distance", float, back, "metres")],
        aliases=["move_backward", "go_back"]))

    reg.register(Skill(
        "TURN_LEFT", "Turn left `angle` degrees",
        call=lambda ex, a: ex.turn(float(a.get("angle", turn)), "left"),
        params=[Param("angle", float, turn, "degrees")]))

    reg.register(Skill(
        "TURN_RIGHT", "Turn right `angle` degrees",
        call=lambda ex, a: ex.turn(float(a.get("angle", turn)), "right"),
        params=[Param("angle", float, turn, "degrees")]))

    reg.register(Skill(
        "ROTATE", "Rotate in place by `angle` degrees",
        call=lambda ex, a: ex.rotate(float(a.get("angle", rot))),
        params=[Param("angle", float, rot, "degrees")]))

    reg.register(Skill(
        "NAVIGATE_TO_DESTINATION", "Navigate to a known named destination",
        call=lambda ex, a: ex.navigate_to(a.get("destination_name"), a.get("speed")),
        params=[Param("destination_name", str, None, "config destination slug")],
        aliases=["navigate_to", "go_to"]))

    reg.register(Skill(
        "GO_TO_COORDINATES", "Navigate to an (x, y, z) goal — e.g. GO_TO_COORDINATES(x=1.0, y=0.0, z=0.0)",
        call=lambda ex, a: ex.go_to_coordinates(
            normalize_coordinates(a.get("coordinates", a)), a.get("speed")),
        params=[Param("x", float, None, "metres"),
                Param("y", float, None, "metres"),
                Param("z", float, None, "metres")]))

    reg.register(Skill(
        "MOVE_TO_OBJECT", "Navigate to a detected object by concept name (open-vocab)",
        call=lambda ex, a: ex.move_to_object(a.get("object_name")),
        params=[Param("object_name", str, None, "free-text object concept")]))

    reg.register(Skill(
        "CIRCULAR_MOTION", "Drive an arc/circle of `radius` metres",
        call=lambda ex, a: ex.circular_motion(a),
        params=[Param("radius", float, circ, "metres"),
                Param("angle", float, 360.0, "degrees of arc")],
        aliases=["circle"]))

    reg.register(Skill(
        "SEND_IMAGE", "Capture and publish the current camera view",
        call=lambda ex, a: ex.send_image(),
        aliases=["take_photo"]))

    reg.register(Skill(
        "DESCRIBE_SURROUNDINGS", "Detect and describe visible objects",
        call=lambda ex, a: ex.describe_surroundings(),
        aliases=["look_around", "describe"]))

    reg.register(Skill(
        "REPORT_COORDINATES", "Report the robot's current position",
        call=lambda ex, a: _ok(ex.report_coordinates())))

    reg.register(Skill(
        "REPORT_ORIENTATION", "Report the robot's current orientation",
        call=lambda ex, a: _ok(ex.report_orientation())))

    reg.register(Skill(
        "STOP", "Stop all motion immediately",
        call=lambda ex, a: _ok(ex.stop_all()),
        aliases=["halt", "cancel"]))

    return reg
