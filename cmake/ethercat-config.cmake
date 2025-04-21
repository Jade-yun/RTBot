# ethercat-config.cmake

# 获取 ethercat-config.cmake 所在的目录
get_filename_component(_ethercat_config_dir "${CMAKE_CURRENT_LIST_FILE}" PATH)

# 构造相对路径
set(_ethercat_prefix "${_ethercat_config_dir}/../third_party/lib")

find_library(EtherCAT_LIBRARY
    NAMES ethercat
    PATHS "${_ethercat_prefix}/lib"
    NO_DEFAULT_PATH
)

find_path(EtherCAT_INCLUDE_DIR
    NAMES ecrt.h
    PATHS "${_ethercat_prefix}/include"
    NO_DEFAULT_PATH
)

mark_as_advanced(EtherCAT_LIBRARY EtherCAT_INCLUDE_DIR)

if(NOT TARGET EtherLab::ethercat)
    add_library(EtherLab::ethercat SHARED IMPORTED)
    set_target_properties(EtherLab::ethercat PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EtherCAT_INCLUDE_DIR}"
        IMPORTED_LOCATION "${EtherCAT_LIBRARY}"
    )
endif()
