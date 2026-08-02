#!/usr/bin/env python3
"""Local CI/CD Test Runner for MANTA Project.
Run this script locally on your PC before pushing code to GitHub:
    python run_tests.py
"""

import sys
import subprocess
import os

def main():
    print("==================================================")
    print("        MANTA Local CI/CD Test Suite Runner       ")
    print("==================================================")

    project_root = os.path.dirname(os.path.abspath(__file__))
    os.chdir(project_root)

    cmd = [sys.executable, "-m", "pytest", "tests", "-v", "-s"]

    print(f"Executing: {' '.join(cmd)}\n")
    result = subprocess.run(cmd)

    if result.returncode == 0:
        print("\n==================================================")
        print("  SUCCESS: All CI/CD tests PASSED cleanly!      ")
        print("==================================================")
    else:
        print("\n==================================================")
        print("  FAILED: One or more CI/CD tests FAILED!        ")
        print("==================================================")

    sys.exit(result.returncode)

if __name__ == "__main__":
    main()
