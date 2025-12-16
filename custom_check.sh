#!/usr/bin/env bash
#
# Copyright (C) 2025 Xiaomi Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

TARGET_DIR="."
LOG_FILE="vendor/infineon/logfile"
EXCLUDE_DIRS=("vendor/infineon/chips/aurix/illd" "vendor/infineon/chips/aurix/scr" "vendor/infineon/chips/aurix/mcal" "vendor/infineon/boards/aurix/common_code/mcal")
TARGET_DIRS=("vendor/infineon/chips")
SKIP_ERROR=("Mixed case identifier found")

CHECKPATCH_SCRIPT="nuttx/tools/checkpatch.sh"
if [ ! -f "$CHECKPATCH_SCRIPT" ]; then
  echo "Error: checkpatch.sh not found"
  exit 1
fi

EXCLUDE_ARGS=""
for dir in "${EXCLUDE_DIRS[@]}"; do
  dir=${dir%/}
  for subdir in $(find "$dir" -type d); do
    EXCLUDE_ARGS+="-path $subdir -prune -o "
  done
done

EXCLUDE_ARGS=${EXCLUDE_ARGS%" -o "}

FILES=""
if [ -z "$EXCLUDE_ARGS" ]; then
  FILES=$(find "${TARGET_DIRS[@]}" -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" -o -name "*.py" -o -name "*.rs" -o -name "CMakeLists.txt" -o -name "*.cmake" \) -print)
else
  FILES=$(find "${TARGET_DIRS[@]}" \( $EXCLUDE_ARGS \) -prune -o -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" -o -name "*.py" -o -name "*.rs" -o -name "CMakeLists.txt" -o -name "*.cmake" \) -print)
fi

fail=0
for file in $FILES; do
  echo "Checking: $file"
  if [ -f "$file" ]; then
    "$CHECKPATCH_SCRIPT" -f -c -u "$file" &>> "$LOG_FILE"
  fi

  if [ $? -ne 0 ]; then
    fail=1
  fi
done

ERROR_LINES=$(grep -i "error" "$LOG_FILE")

for pattern in $SKIP_ERROR; do
    ERROR_LINES=$(echo "$ERROR_LINES" | grep -v "$pattern")
done

if [ -n "$ERROR_LINES" ]; then
    echo "Checkpatch failed with unexpected issues:"
    echo "$ERROR_LINES"
    rm -f "$LOG_FILE"
    exit 1
else
    echo "Checkpatch passed"
    rm -f "$LOG_FILE"
    exit 0
fi
