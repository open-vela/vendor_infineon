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

_vendor_prebuild_hook(){
    local TOP_DIR=$(gettop)
    local SMARTCODE_PATH=${TOP_DIR}/prebuilts/tasking/linux/SmartCode/ctc/bin
    echo "Add taskling path: $SMARTCODE_PATH"
    export PATH=$SMARTCODE_PATH:$PATH
    export VELA_EXTRA_FLAGS=""
}

chips=("tc4d7_evb" "tc4d9_evb")
cores_common=("bl" "core0" "core1" "core2" "core3" "core4" "core5" "corecs")

for chip in "${chips[@]}"; do
    for core in "${cores_common[@]}"; do
        add_vendor_prebuild_hook "[infineon]-[${chip}]-[${core}]" "_vendor_prebuild_hook"
    done
done
