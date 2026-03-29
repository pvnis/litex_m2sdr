cmake_minimum_required(VERSION 3.16)
project(SoapySDRLiteXM2SDR CXX C)

set(CMAKE_CXX_STANDARD 17)

option(USE_VFIO    "Uses VFIO user-mode driver for zero-copy low-latency streaming" OFF)

########################################################################
## LitePCIe discovery
########################################################################

find_path(LITEPCIE_KERNEL_INCLUDE_DIR litepcie.h
          REQUIRED
          HINTS
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
          PATH_SUFFIXES
            litex_m2sdr/kernel
            software/kernel
)
find_path(LITEPCIE_USER_INCLUDE_DIR liblitepcie.h
          REQUIRED
          HINTS
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
          PATH_SUFFIXES
            litex_m2sdr/liblitepcie
            software/user/liblitepcie
)
set(LITEPCIE_INCLUDE_DIR ${LITEPCIE_KERNEL_INCLUDE_DIR} ${LITEPCIE_USER_INCLUDE_DIR})
find_library(LITEPCIE_LIBRARY litepcie
             REQUIRED
             HINTS
               ${CMAKE_CURRENT_SOURCE_DIR}/../..
             PATH_SUFFIXES
               software/user/liblitepcie)

include_directories(${LITEPCIE_INCLUDE_DIR})

########################################################################
## USER discovery
########################################################################

find_path(USER_INCLUDE_DIR m2sdr_config.h
          REQUIRED
          HINTS
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
          PATH_SUFFIXES
            litex_m2sdr/libm2sdr
            software/user/libm2sdr
)

include_directories(${USER_INCLUDE_DIR})

########################################################################
## AD9361 discovery
########################################################################

find_path(AD9361_INCLUDE_DIR ad9361/ad9361.h
          REQUIRED
          HINTS
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
          PATH_SUFFIXES
            litex_m2sdr
            software/user
)

include_directories(${AD9361_INCLUDE_DIR})

########################################################################
## LIBM2SDR discovery
########################################################################

find_path(LIBM2SDR_INCLUDE_DIR libm2sdr.h
          REQUIRED
          HINTS
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
          PATH_SUFFIXES
            litex_m2sdr/libm2sdr
            software/user/libm2sdr
)

include_directories(${LIBM2SDR_INCLUDE_DIR})

find_library(LIBM2SDR_LIBRARY m2sdr
             REQUIRED
             HINTS
               ${CMAKE_CURRENT_SOURCE_DIR}/../..
             PATH_SUFFIXES
               litex_m2sdr/libm2sdr
               software/user/libm2sdr
)

########################################################################
## VFIO library discovery (optional)
########################################################################

if(USE_VFIO)
    find_path(LIBM2SDR_VFIO_INCLUDE_DIR m2sdr_vfio.h
              REQUIRED
              HINTS
                ${CMAKE_CURRENT_SOURCE_DIR}/../..
              PATH_SUFFIXES
                litex_m2sdr/libm2sdr_vfio
                software/user/libm2sdr_vfio
    )
    include_directories(${LIBM2SDR_VFIO_INCLUDE_DIR})

    find_library(LIBM2SDR_VFIO_LIBRARY m2sdr_vfio
                 REQUIRED
                 HINTS
                   ${CMAKE_CURRENT_SOURCE_DIR}/../..
                 PATH_SUFFIXES
                   litex_m2sdr/libm2sdr_vfio
                   software/user/libm2sdr_vfio
    )
endif()

########################################################################
## SoapySDR library
########################################################################

find_package(SoapySDR "0.2.1" REQUIRED)

set(LITEXM2SDR_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../software/user/ad9361/ad9361_api.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../software/user/ad9361/ad9361.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../software/user/ad9361/ad9361_conv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../software/user/ad9361/util.c
)

if(USE_VFIO)
    list(APPEND LITEXM2SDR_SOURCE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../software/user/libm2sdr/m2sdr_device.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../../software/user/libm2sdr/m2sdr_hal_vfio.c
    )
endif()

if(USE_VFIO)
    add_definitions(-DUSE_VFIO=1 -DUSE_LITEPCIE=0)
    set(VFIO_EXTRA_LIBS ${LIBM2SDR_VFIO_LIBRARY})
else()
    add_definitions(-DUSE_VFIO=0 -DUSE_LITEPCIE=1)
    set(VFIO_EXTRA_LIBS "")
endif()

SOAPY_SDR_MODULE_UTIL(
    TARGET SoapyLiteXM2SDR
    SOURCES LiteXM2SDRDevice.cpp LiteXM2SDRStreaming.cpp
    LiteXM2SDRRegistration.cpp
    ${LITEXM2SDR_SOURCE}
    LIBRARIES ${LIBM2SDR_LIBRARY} ${LITEPCIE_LIBRARY} ${VFIO_EXTRA_LIBS} m
)

# Suppress specific warnings
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(SoapyLiteXM2SDR PRIVATE
        -Wno-sign-compare
        -Wno-implicit-fallthrough
        -Wno-maybe-uninitialized
        -Wno-unused-parameter
    )
endif()
