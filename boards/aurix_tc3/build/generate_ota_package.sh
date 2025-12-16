#!/bin/bash
TOP=$1
RELEASE_ID=$2
board=$3

Car="mann"
product=${board}
${TOP}/prebuilts/tools/micar/build_tool/create_a2l_mbf.sh -C ${Car} -p ${product} -b flat -c gcc -V ${RELEASE_ID} --images $TOP/${RELEASE_ID}/images/${board}
exit_code=$?
if [ $exit_code -ne 0 ]; then
    echo "Error: ${Car} ${product} create_a2l_mbf.sh failed with exit code $exit_code"
    exit $exit_code
fi
