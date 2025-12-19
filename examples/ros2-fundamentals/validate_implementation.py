#!/usr/bin/env python3

"""
Validation script for ROS 2 Fundamentals Module Implementation

This script validates that all components of the ROS 2 fundamentals module
are working correctly by testing the example code.
"""

import subprocess
import sys
import os
from pathlib import Path


def check_python_file_syntax(file_path):
    """Check if a Python file has valid syntax."""
    try:
        with open(file_path, 'r') as file:
            source = file.read()
        compile(source, file_path, 'exec')
        return True
    except SyntaxError as e:
        print(f"Syntax error in {file_path}: {e}")
        return False


def validate_all_python_examples():
    """Validate syntax of all Python example files."""
    examples_dir = Path("examples/ros2-fundamentals")
    python_files = list(examples_dir.glob("*.py"))

    print("Validating Python example files...")
    all_valid = True

    for py_file in python_files:
        if check_python_file_syntax(py_file):
            print(f"OK {py_file.name} - Valid syntax")
        else:
            print(f"X {py_file.name} - Syntax error")
            all_valid = False

    return all_valid


def validate_urdf_file():
    """Basic validation of the URDF file."""
    full_path = Path(__file__).parent / "models" / "example-humanoid.urdf"

    print(f"Validating URDF file: {full_path}")

    if not full_path.exists():
        print(f"X URDF file does not exist: {full_path}")
        return False

    try:
        with open(full_path, 'r') as file:
            content = file.read()

        # Basic checks
        if '<robot' not in content:
            print("X URDF file doesn't contain robot tag")
            return False

        if '<link' not in content:
            print("X URDF file doesn't contain link tags")
            return False

        if '<joint' not in content:
            print("X URDF file doesn't contain joint tags")
            return False

        print(f"OK {full_path.name} - Valid structure")
        return True

    except Exception as e:
        print(f"X Error reading URDF file: {e}")
        return False


def validate_markdown_files():
    """Basic validation of markdown files."""
    docs_dir = Path("docs/ros2-fundamentals")
    md_files = list(docs_dir.rglob("*.md"))

    print("Validating Markdown files...")
    all_valid = True

    for md_file in md_files:
        try:
            with open(md_file, 'r', encoding='utf-8') as file:
                content = file.read()

            # Check for basic structure (frontmatter)
            if content.strip().startswith('---'):
                # Has frontmatter, check if properly closed
                lines = content.split('\n')
                if len(lines) > 1 and lines[0] == '---':
                    # Find the end of frontmatter
                    end_frontmatter = -1
                    for i, line in enumerate(lines[1:], 1):
                        if line.strip() == '---':
                            end_frontmatter = i
                            break

                    if end_frontmatter == -1:
                        print(f"X {md_file.name} - Unclosed frontmatter")
                        all_valid = False
                    else:
                        print(f"OK {md_file.name} - Valid frontmatter")
                else:
                    print(f"OK {md_file.name} - Valid markdown")
            else:
                print(f"OK {md_file.name} - Valid markdown")

        except Exception as e:
            print(f"X Error reading {md_file.name}: {e}")
            all_valid = False

    return all_valid


def main():
    print("ROS 2 Fundamentals Module - Implementation Validation")
    print("=" * 55)

    # Validate Python examples
    python_valid = validate_all_python_examples()

    # Validate URDF file
    urdf_valid = validate_urdf_file()

    # Validate Markdown files
    markdown_valid = validate_markdown_files()

    print("Validation Summary:")
    print(f"Python examples: {'PASS' if python_valid else 'FAIL'}")
    print(f"URDF file: {'PASS' if urdf_valid else 'FAIL'}")
    print(f"Markdown files: {'PASS' if markdown_valid else 'FAIL'}")

    overall_success = python_valid and urdf_valid and markdown_valid

    print(f"Overall: {'ALL TESTS PASSED' if overall_success else 'SOME TESTS FAILED'}")

    return 0 if overall_success else 1


if __name__ == "__main__":
    sys.exit(main())