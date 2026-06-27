#!/usr/bin/env python3

import glob
import os
import re
import sys
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(ROOT, "config", "sil_config.yaml")

def flatten(node, prefix=""):
    keys = set()
    if isinstance(node, dict):
        for key, value in node.items():
            full = f"{prefix}/{key}".strip("/")
            keys.add(full)
            keys |= flatten(value, full)
    return keys

def main() -> int:
    with open(CONFIG) as handle:
        cfg_keys = flatten(yaml.safe_load(handle))

    pattern = re.compile(r"get_param\(\s*[\"']([^\"']+)[\"']")
    used = {}
    sources = glob.glob(os.path.join(ROOT, "src", "sil_ros", "*.py"))
    sources += glob.glob(os.path.join(ROOT, "scripts", "*.py"))
    for path in sources:
        with open(path) as handle:
            for line in handle:
                for match in pattern.findall(line):
                    key = match.strip("/")
                    if key:
                        used.setdefault(key, set()).add(os.path.basename(path))

    missing = {k: v for k, v in used.items() if k not in cfg_keys}

    print(f"config keys: {len(cfg_keys)} | referenced keys: {len(used)}")
    if missing:
        print("\nMISSING from config/sil_config.yaml:")
        for key, files in sorted(missing.items()):
            print(f"  {key}  <- {', '.join(sorted(files))}")
        return 1
    print("All get_param keys resolve against the config. OK")
    return 0

if __name__ == "__main__":
    sys.exit(main())
