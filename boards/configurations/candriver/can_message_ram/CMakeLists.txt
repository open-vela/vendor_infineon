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

if(CONFIG_AURIX_MCMCAN)
    # 仅添加头文件搜索路径，无需编译源文件
    set(INCDIRS
        ${CMAKE_CURRENT_LIST_DIR}/inc
    )

    # 将头文件目录添加到 NuttX 工程的全局包含路径中
    set_property(
        TARGET nuttx
        APPEND
        PROPERTY NUTTX_INCLUDE_DIRECTORIES ${INCDIRS}
    )
endif()
