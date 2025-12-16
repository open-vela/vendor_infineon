#!/bin/bash

set -x

top_path=$1
build_config_path=$2

if [[ "$build_config_path" =~ "aurix" ]]; then
    declare -A flatD=(
        ["evb"]="true"
    )

    if [[ -z ${flatD[$(basename ${build_config_path})]} ]]; then
        buildmode="protect"
    else
        buildmode="flat"
    fi
    product=$(echo $build_config_path | cut -d '/' -f 6)
    generate_script="./prebuilts/tools/micar/generator/generator.sh"
    if [ -e ${generate_script} ]; then
        ${generate_script} -C "mann" -p ${product} -b ${buildmode} -g "add"
        exit_code=$?
        if [ ${exit_code} -ne 0 ]; then
            echo "Generate code failed, exit code ${exit_code}"
            exit ${exit_code}
        fi
    fi

    cd ${top_path}
    echo "check defconfig:${build_config_path}"
    if [[ "$build_config_path" =~ "test" ]]; then
        echo "skip test core check"
    else
        if [ -f "./build/envsetup.sh" ]; then
            source ./build/envsetup.sh
        fi
        if ! ./nuttx/tools/refresh.sh --silent --cmake ${build_config_path} >.check_defconfig_warning; then
            fail=1
            echo "error: check defconfig warning: ${build_config_path}"
            cat .check_defconfig_warning
            exit 1
        fi
        if [ -f "${top_path}/nuttx/.config" ]; then
            rm -f ${top_path}/nuttx/.config
        fi

        if [ -f "${top_path}/nuttx/Make.defs" ]; then
            rm -f ${top_path}/nuttx/Make.defs
        fi
    fi
    cd -

    modify_script="./prebuilts/tools/micar/generator/modify_defconfigs.sh"
    if [ -e ${modify_script} ]; then
        ${modify_script} -C "mann" -p ${product} -b ${buildmode}
        exit_code=$?
        if [ ${exit_code} -ne 0 ]; then
            echo "Modify defconfigs failed, exit code ${exit_code}"
            exit ${exit_code}
        fi
    fi
fi
