# ethercat-config.cmake

#----------------------------------------------------------------------------
#
#  Copyright (C) 2021 Bjarne von Horn, Ingenieurgemeinschaft IgH
#
#  This file is part of the IgH EtherCAT Master.
#
#  The IgH EtherCAT Master is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License version 2, as
#  published by the Free Software Foundation.
#
#  The IgH EtherCAT Master is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
#  Public License for more details.
#
#  You should have received a copy of the GNU General Public License along
#  with the IgH EtherCAT Master; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#
#  vim: tw=78
#
#----------------------------------------------------------------------------

find_library(EtherCAT_LIBRARY
    NAMES ethercat
    PATHS ${CMAKE_SOURCE_DIR}//third_party/lib
)

find_path(EtherCAT_INCLUDE_DIR
    NAMES ecrt.h
    PATHS ${CMAKE_SOURCE_DIR}//third_party/include/etherlab
)

mark_as_advanced(EtherCAT_LIBRARY EtherCAT_INCLUDE_DIR)

if(NOT TARGET EtherLab::ethercat)
    add_library(EtherLab::ethercat SHARED IMPORTED)
    set_target_properties(EtherLab::ethercat PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EtherCAT_INCLUDE_DIR}"
        IMPORTED_LOCATION "${EtherCAT_LIBRARY}"
    )
endif()
