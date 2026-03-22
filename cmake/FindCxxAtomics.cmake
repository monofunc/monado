# Copyright 2023-2025, Collabora, Ltd.
#
# SPDX-License-Identifier: BSL-1.0
#
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE_1_0.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)
#
# Original Author:
# 2025 Rylie Pavlik <rylie.pavlik@collabora.com>

#[=======================================================================[.rst:
FindCxxAtomics
--------------

Find the library to link for atomics in C++, if any.
Components are the atomic typenames.

Targets
^^^^^^^

If successful, import targets are created in the following format:

``CxxAtomics::TYPENAME``

e.g.

``CxxAtomics::atomic_char``

``CxxAtomics::atomic_uint_fast64``


#]=======================================================================]

include(CheckCXXSourceCompiles)
include(FindPackageHandleStandardArgs)
# find_library(CxxAtomics_LIBRARY NAMES atomic libatomic.so.1)
set(CxxAtomics_LIBRARY atomic)

# Wraps check_cxx_source_compiles
function(_cxxatomics_check_compiles _typename _cachevar)
    set(cxx_atomic_check_code
        [[
#include <atomic>
static std::${_typename} generator{};
int main() {
    ++generator;
    return 0;
}
]])
    check_cxx_source_compiles("${cxx_atomic_check_code}" ${_cachevar})
endfunction()

# Set the outvar to true if you need to build with the lib, false if you do not.
function(_cxxatomics_find_needs_lib _typename _outvar)

    _cxxatomics_check_compiles(${_typename} CxxAtomics_${_typename}_NO_LIB)
    if(CxxAtomics_${_typename}_NO_LIB)
        set(${_outvar}
            FALSE
            PARENT_SCOPE)
        return()
    endif()

    set(${_outvar}
        TRUE
        PARENT_SCOPE)
endfunction()

function(_cxxatomics_check_typename _typename)
    set(CxxAtomics_${_typename}_FOUND FALSE)

    _cxxatomics_find_needs_lib(${_typename} CxxAtomics_${_typename}_NEEDS_LIB)
    if(CxxAtomics_${_typename}_NEEDS_LIB)

        if(CxxAtomics_LIBRARY)
            set(CMAKE_REQUIRED_LIBRARIES ${CxxAtomics_LIBRARY})
            _cxxatomics_check_compiles(${_typename}
                                       CxxAtomics_${_typename}_WITH_LIB)
            if(CxxAtomics_${_typename}_WITH_LIB)
                set(CxxAtomics_${_typename}_FOUND TRUE)
            endif()
        endif()

        if(CxxAtomics_${_typename}_FOUND AND NOT TARGET
                                             CxxAtomics::${_typename})
            add_library(CxxAtomics::${_typename} INTERFACE IMPORTED GLOBAL)
            target_link_libraries(CxxAtomics::${_typename}
                                  INTERFACE ${CxxAtomics_LIBRARY})
        endif()
    else()
        # no lib needed
        set(CxxAtomics_${_typename}_FOUND TRUE)

        if(NOT TARGET CxxAtomics::${_typename})
            # no extra requirements
            add_library(CxxAtomics::${_typename} INTERFACE IMPORTED GLOBAL)
        endif()
    endif()
    list(APPEND _cxxatomics_required_vars CxxAtomics_${_typename}_FOUND)
    set(CxxAtomics_${_typename}_FOUND
        "${CxxAtomics_${_typename}_FOUND}"
        PARENT_SCOPE)
    set(_cxxatomics_required_vars
        "${_cxxatomics_required_vars}"
        PARENT_SCOPE)
endfunction()

set(cxxatomics_possible_components
    atomic_flag
    atomic_ref
    atomic_bool
    atomic_char
    atomic_schar
    atomic_uchar
    atomic_short
    atomic_ushort
    atomic_int
    atomic_uint
    atomic_long
    atomic_ulong
    atomic_llong
    atomic_ullong
    atomic_char8_t
    atomic_char16_t
    atomic_char32_t
    atomic_wchar_t
    atomic_int8_t
    atomic_uint8_t
    atomic_int16_t
    atomic_uint16_t
    atomic_int32_t
    atomic_uint32_t
    atomic_int64_t
    atomic_uint64_t
    atomic_int_least8_t
    atomic_uint_least8_t
    atomic_int_least16_t
    atomic_uint_least16_t
    atomic_int_least32_t
    atomic_uint_least32_t
    atomic_int_least64_t
    atomic_uint_least64_t
    atomic_int_fast8_t
    atomic_uint_fast8_t
    atomic_int_fast16_t
    atomic_uint_fast16_t
    atomic_int_fast32_t
    atomic_uint_fast32_t
    atomic_int_fast64_t
    atomic_uint_fast64_t
    atomic_intptr_t
    atomic_uintptr_t
    atomic_size_t
    atomic_ptrdiff_t
    atomic_intmax_t
    atomic_uintmax_t
    atomic_signed_lock_free
    atomic_unsigned_lock_free)
set(_cxxatomics_required_vars)
if(NOT CxxAtomics_FIND_COMPONENTS)
    set(CxxAtomics_FIND_COMPONENTS
        atomic_int8_t
        atomic_uint8_t
        atomic_int16_t
        atomic_uint16_t
        atomic_int32_t
        atomic_uint32_t
        atomic_int64_t
        atomic_uint64_t)
endif()

foreach(typename ${CxxAtomics_FIND_COMPONENTS})
    _cxxatomics_check_typename(${typename})
endforeach()

find_package_handle_standard_args(
    CxxAtomics
    REQUIRED_VARS ${_cxxatomics_required_vars}
    HANDLE_COMPONENTS)
