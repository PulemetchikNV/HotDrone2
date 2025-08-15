#!/usr/bin/env bash
echo "--- Cleaning Python cache files ---"
find . -type f -name "*.pyc" -delete
find . -type d -name "__pycache__" -exec rm -r {} +
echo "--- Cleanup complete ---"
