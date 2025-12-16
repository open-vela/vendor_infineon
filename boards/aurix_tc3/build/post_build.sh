#!/bin/bash
TOP=$1
RELEASE_ID=$2
board=$3
config=$4
base_config=$5
CMAKE_CONFIG_DIR=$6

CUR_DIR=$(cd $(dirname $0); pwd)

copyPath="$TOP/${RELEASE_ID}/images/${board}/${config}"
if [ ! -d $copyPath ]; then
    mkdir -p $copyPath
fi

cp ${CMAKE_CONFIG_DIR}/vela_${base_config}.* ${copyPath}/ || true

post_build_script=$(dirname ${CUR_DIR})/${board}/tools/post_build.sh
if [ -f ${post_build_script} ];then
    ${post_build_script} ${TOP} ${copyPath}
fi

