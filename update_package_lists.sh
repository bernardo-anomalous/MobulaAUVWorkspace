#!/bin/bash

# Navigate to the workspace root
cd "$(dirname "$0")"

# Regenerate pip and apt package lists
echo "[INFO] Generating pip_requirements.txt..."
pip3 freeze > pip_requirements.txt

echo "[INFO] Generating apt_package_list.txt..."
dpkg --get-selections > apt_package_list.txt

# Stage the updated files
git add pip_requirements.txt apt_package_list.txt

echo "[INFO] Package lists updated and staged for commit."
echo "Ready to commit with: git commit -m 'Update package lists'"
