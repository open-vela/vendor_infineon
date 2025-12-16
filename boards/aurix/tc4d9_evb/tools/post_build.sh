#!/bin/bash

readonly BASEDIR="$1"
readonly PRODUCT_PATH="$2"
readonly REMAP_PY="$BASEDIR/prebuilts/tools/micar/build_tool/hex_converter/hex_remap.py"

# List of hex file base names to process (includes bootloader and core applications)
names=(vela_bl vela_core0 vela_core0_user vela_core1 vela_core1_user
       vela_core2 vela_core2_user vela_core3 vela_core3_user
       vela_core4 vela_core4_user vela_core5 vela_core5_user vela_corecs)

# Define address mapping configurations for TC4D Bank A (hexadecimal format)
# Each entry follows: [source_start_address source_end_address target_start_address]
declare -a MAP_TC4D_A=(
    "0x80000000 0x80200000 0x80000000"
    "0x80200000 0x80400000 0x80400000"
    "0x80400000 0x80500000 0x80800000"
    "0x80500000 0x80700000 0x80A00000"
    "0x80700000 0x80900000 0x80E00000"
    "0x80900000 0x80A00000 0x81200000"
    "0x84000000 0x84040000 0x84000000"
    "0xA0000000 0xA0200000 0x80000000"
    "0xA0200000 0xA0400000 0x80400000"
    "0xA0400000 0xA0500000 0x80800000"
    "0xA0500000 0xA0700000 0x80A00000"
    "0xA0700000 0xA0900000 0x80E00000"
    "0xA0900000 0xA0A00000 0x81200000"
    "0xA4000000 0xA4040000 0x84000000"
)

# Define address mapping configurations for TC4D Bank B (hexadecimal format)
# Each entry follows: [source_start_address source_end_address target_start_address]
declare -a MAP_TC4D_B=(
    "0x80000000 0x80200000 0x80200000"
    "0x80200000 0x80400000 0x80600000"
    "0x80400000 0x80500000 0x80900000"
    "0x80500000 0x80700000 0x80C00000"
    "0x80700000 0x80900000 0x81000000"
    "0x80900000 0x80A00000 0x81300000"
    "0x84000000 0x84040000 0x84040000"
    "0xA0000000 0xA0200000 0x80200000"
    "0xA0200000 0xA0400000 0x80600000"
    "0xA0400000 0xA0500000 0x80900000"
    "0xA0500000 0xA0700000 0x80C00000"
    "0xA0700000 0xA0900000 0x81000000"
    "0xA0900000 0xA0A00000 0x81300000"
    "0xA4000000 0xA4040000 0x84040000"
)

# Convert bash array to JSON string array (fixed implementation)
# Parameters:
#   - arr: Bash array containing address mapping entries
# Returns:
#   - JSON formatted string array of mapping entries
function array_to_json() {
    local arr=("$@")
    local json="["
    local first=true

    for item in "${arr[@]}"; do
        IFS=' ' read -r -a values <<< "$item"

        if [ "$first" = true ]; then
            first=false
        else
            json+=","
        fi

        # Wrap each hex value in double quotes to ensure valid JSON syntax
        json+="[\"${values[0]}\",\"${values[1]}\",\"${values[2]}\"]"
    done

    json+="]"
    echo "$json"
}

# Verify that the hex remapping Python script exists
if [[ ! -f "$REMAP_PY" ]]; then
    echo "Error: hex_remap.py script not found at $REMAP_PY"
    exit 1
fi

# Convert address mapping configurations to JSON format strings
map_a_json=$(array_to_json "${MAP_TC4D_A[@]}")
map_b_json=$(array_to_json "${MAP_TC4D_B[@]}")

# Process each hex file in the names list
for key in "${names[@]}"; do
    for f in "${PRODUCT_PATH}/${key}.hex"; do
        # Only process if the source hex file exists
        if [[ -e "$f" ]]; then
            # Process BankA version - pass JSON mapping as parameter
            python3 "$REMAP_PY" "$f" "${PRODUCT_PATH}/${key}_bankA.hex" "$map_a_json"
            exit_code=$?
            if [ $exit_code -ne 0 ]; then
                echo "Error: Failed to generate ${key}_bankA hex file!"
                exit $exit_code
            fi

            # Process BankB version - pass JSON mapping as parameter
            python3 "$REMAP_PY" "$f" "${PRODUCT_PATH}/${key}_bankB.hex" "$map_b_json"
            exit_code=$?
            if [ $exit_code -ne 0 ]; then
                echo "Error: Failed to generate ${key}_bankB hex file!"
                exit $exit_code
            fi
        fi
    done
done