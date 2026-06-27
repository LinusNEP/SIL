#!/usr/bin/env python

import re
import json
import rospy
from typing import Dict, List, Union, Optional
import rospkg
import os

class CommandParser:
    def __init__(self, action_executor):
        self.action_executor = action_executor
        self.patterns = self._load_patterns()
        self.linear_speed = rospy.get_param("speeds/default_linear_speed", 0.2)
        self.angular_speed = rospy.get_param("speeds/default_angular_speed", 0.5)
        self.maximum_speed = rospy.get_param("speeds/maximum_speed", 1.0)
        self.minimum_speed = rospy.get_param("speeds/minimum_speed", 0.2)

    def _load_patterns(self) -> Dict[str, str]:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('sil_ros')
        full_path = os.path.join(package_path, 'config', 'action_dictionary.json')
        with open(full_path, "r") as file:
            data = json.load(file)
            return data["Action_Dictionary"]["patterns"]

    def parse_input(self, llm_output: Dict) -> Dict:
        if llm_output['type'] == 'RESPONSE':
            return {'type': 'RESPONSE', 'content': llm_output['content']}
        elif llm_output['type'] == 'ACTIONS':
            actions = self.parse_actions(llm_output['content'])
            return {'type': 'ACTIONS', 'content': actions}
        else:
            return {'type': 'UNKNOWN', 'content': llm_output['content']}
        
    def parse_actions(self, llm_output: str) -> List[Dict]:
        actions = []
        lines = llm_output.split('\n')
        
        for line in lines:
            line = line.strip()
            if not line:
                continue
            action_match = re.match(r'Action\s*\d*:?\s*(.+)', line, re.IGNORECASE)
            if action_match:
                action_description = action_match.group(1).strip()
                parsed = self._parse_action_flexible(action_description)
                if parsed:
                    if isinstance(parsed, list):
                        actions.extend(parsed)
                    else:
                        actions.append(parsed)
                else:
                    rospy.loginfo(f"[Parser] Could not parse: {action_description}")
        
        if actions:
            rospy.loginfo(f"[Parser] Successfully parsed {len(actions)} actions")   
        return actions

    def _parse_action_flexible(self, description: str) -> Optional[Union[Dict, List]]:
        desc_lower = description.lower().strip()
        desc_cleaned = self._clean_description(desc_lower)
        if any(phrase in desc_cleaned for phrase in ['navigate to', 'go to', 'return to', 'head to']):
            destination = self._extract_destination(desc_cleaned)
            if destination:
                speed = None
                if any(word in desc_cleaned for word in ['fast', 'quickly', 'rapidly']):
                    speed = self.maximum_speed
                elif any(word in desc_cleaned for word in ['slow', 'slowly', 'carefully']):
                    speed = self.minimum_speed
                action = {'action': 'NAVIGATE_TO_DESTINATION', 'destination_name': destination}
                if speed:
                    action['speed'] = speed
                return action
        if any(phrase in desc_cleaned for phrase in ['move forward', 'go forward', 'forward']):
            distance = self._extract_number(desc_cleaned, 'meter|m', default=1.0)
            return {'action': 'FORWARD', 'distance': distance}
        
        if any(phrase in desc_cleaned for phrase in ['move backward', 'go backward', 'backward', 'back']):
            distance = self._extract_number(desc_cleaned, 'meter|m', default=1.0)
            return {'action': 'BACKWARD', 'distance': distance}

        if 'left' in desc_cleaned and any(word in desc_cleaned for word in ['turn', 'rotate']):
            angle = self._extract_number(desc_cleaned, 'degree|deg', default=90.0)
            return {'action': 'TURN_LEFT', 'angle': angle}
        
        if 'right' in desc_cleaned and any(word in desc_cleaned for word in ['turn', 'rotate']):
            angle = self._extract_number(desc_cleaned, 'degree|deg', default=90.0)
            return {'action': 'TURN_RIGHT', 'angle': angle}
        
        if 'rotate' in desc_cleaned:
            angle = self._extract_number(desc_cleaned, 'degree|deg', default=360.0)
            return {'action': 'ROTATE', 'angle': angle}

        if any(phrase in desc_cleaned for phrase in ['circle', 'circular', 'arc']):
            radius = self._extract_number(desc_cleaned, 'meter|m', default=1.0)
            angle = 360.0
            if 'half' in desc_cleaned or 'semi' in desc_cleaned:
                angle = 180.0
            elif 'quarter' in desc_cleaned:
                angle = 90.0
            
            return {
                'action': 'CIRCULAR_MOTION',
                'radius': radius,
                'angle': angle,
                'direction': 'clockwise'
            }
        
        if any(phrase in desc_cleaned for phrase in ['send image', 'take photo', 'picture', 'capture']):
            return {'action': 'SEND_IMAGE'}
        
        if any(phrase in desc_cleaned for phrase in ['describe', 'look around', 'what do you see']):
            return {'action': 'DESCRIBE_SURROUNDINGS'}
        
        if 'stop' in desc_cleaned:
            return {'action': 'STOP'}

    def _clean_description(self, description: str) -> str:
        filler_words = ['the', 'a', 'an', 'to', 'at', 'in', 'on', 'with']
        words = description.split()
        cleaned = ' '.join([w for w in words if w not in filler_words])
        return cleaned

    def _extract_destination(self, text: str) -> Optional[str]:
        resolver = getattr(self.action_executor, 'dest_resolver', None)
        phrase = re.sub(
            r'^(?:navigate to|go to|return to|head to|move to)\s+(?:the\s+)?',
            '', text).strip()

        if resolver is not None and phrase:
            slug, how = resolver.resolve(phrase)
            if slug:
                rospy.loginfo(f"[Parser] Resolved destination '{phrase}' -> '{slug}' ({how})")
                return slug
        patterns = [
            r'(?:navigate to|go to|return to|head to)\s+(?:the\s+)?(\w+(?:\s+\w+)?)',
            r'to\s+(?:the\s+)?(\w+(?:\s+\w+)?)\s*$'
        ]
        for pattern in patterns:
            match = re.search(pattern, text)
            if match:
                dest = match.group(1).strip()
                if resolver is not None:
                    slug, _ = resolver.resolve(dest)
                    if slug:
                        return slug
                return dest.replace(' ', '_')

        return None

    def _extract_number(self, text: str, unit_pattern: str = '', default: float = 1.0) -> float:
        pattern = rf'(\d+(?:\.\d+)?)\s*(?:{unit_pattern})?' if unit_pattern else r'(\d+(?:\.\d+)?)'
        match = re.search(pattern, text, re.IGNORECASE)
        if match:
            try:
                return float(match.group(1))
            except (ValueError, IndexError):
                pass
        return default
    
