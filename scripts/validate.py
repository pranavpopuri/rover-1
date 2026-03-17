#!/usr/bin/env python3
"""
Validation script for rover2 ROS2 packages.
Checks Python syntax, XML well-formedness, and YAML validity.

Usage:
    python3 scripts/validate.py
"""

import os
import sys
import py_compile
import xml.etree.ElementTree as ET
from pathlib import Path

# Colors for terminal output
GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
RESET = '\033[0m'


def check_python_syntax(filepath: Path) -> bool:
    """Check Python file for syntax errors."""
    try:
        py_compile.compile(str(filepath), doraise=True)
        return True
    except py_compile.PyCompileError as e:
        print(f"{RED}FAIL{RESET}: {filepath}")
        print(f"       {e}")
        return False


def check_xml_wellformed(filepath: Path) -> bool:
    """Check XML file is well-formed."""
    try:
        ET.parse(filepath)
        return True
    except ET.ParseError as e:
        print(f"{RED}FAIL{RESET}: {filepath}")
        print(f"       {e}")
        return False


def check_yaml_syntax(filepath: Path) -> bool:
    """Check YAML file syntax (basic check)."""
    try:
        import yaml
        with open(filepath, 'r') as f:
            yaml.safe_load(f)
        return True
    except ImportError:
        # yaml not installed, skip check
        print(f"{YELLOW}SKIP{RESET}: {filepath} (PyYAML not installed)")
        return True
    except Exception as e:
        print(f"{RED}FAIL{RESET}: {filepath}")
        print(f"       {e}")
        return False


def find_files(base_path: Path, extension: str) -> list:
    """Find all files with given extension."""
    return list(base_path.rglob(f"*{extension}"))


def main():
    # Get project root (parent of scripts directory)
    script_dir = Path(__file__).parent
    project_root = script_dir.parent

    print("=" * 60)
    print("Rover2 ROS2 Package Validation")
    print("=" * 60)

    all_passed = True
    total_files = 0
    passed_files = 0

    # Check Python files
    print(f"\n{YELLOW}Checking Python files...{RESET}")
    python_files = find_files(project_root / "src", ".py")
    for f in python_files:
        total_files += 1
        if check_python_syntax(f):
            print(f"{GREEN}OK{RESET}: {f.relative_to(project_root)}")
            passed_files += 1
        else:
            all_passed = False

    # Check XML files (package.xml, xacro, urdf, sdf, launch.xml)
    print(f"\n{YELLOW}Checking XML files...{RESET}")
    xml_extensions = [".xml", ".xacro", ".urdf", ".sdf"]
    for ext in xml_extensions:
        xml_files = find_files(project_root / "src", ext)
        for f in xml_files:
            total_files += 1
            if check_xml_wellformed(f):
                print(f"{GREEN}OK{RESET}: {f.relative_to(project_root)}")
                passed_files += 1
            else:
                all_passed = False

    # Check YAML files
    print(f"\n{YELLOW}Checking YAML files...{RESET}")
    yaml_files = find_files(project_root / "src", ".yaml")
    yaml_files.extend(find_files(project_root / "src", ".yml"))
    for f in yaml_files:
        total_files += 1
        if check_yaml_syntax(f):
            print(f"{GREEN}OK{RESET}: {f.relative_to(project_root)}")
            passed_files += 1
        else:
            all_passed = False

    # Check RViz config (also YAML-ish)
    print(f"\n{YELLOW}Checking RViz config files...{RESET}")
    rviz_files = find_files(project_root / "src", ".rviz")
    for f in rviz_files:
        total_files += 1
        if check_yaml_syntax(f):
            print(f"{GREEN}OK{RESET}: {f.relative_to(project_root)}")
            passed_files += 1
        else:
            all_passed = False

    # Summary
    print("\n" + "=" * 60)
    if all_passed:
        print(f"{GREEN}All {total_files} files passed validation!{RESET}")
        return 0
    else:
        print(f"{RED}Validation failed: {passed_files}/{total_files} files passed{RESET}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
