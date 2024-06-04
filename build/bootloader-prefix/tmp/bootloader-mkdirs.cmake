# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.1.2/components/bootloader/subproject"
  "D:/WORK/PROJECT/ESP-IDF_PROJECTS/SHM-SENSOR-NODE/build/bootloader"
  "D:/WORK/PROJECT/ESP-IDF_PROJECTS/SHM-SENSOR-NODE/build/bootloader-prefix"
  "D:/WORK/PROJECT/ESP-IDF_PROJECTS/SHM-SENSOR-NODE/build/bootloader-prefix/tmp"
  "D:/WORK/PROJECT/ESP-IDF_PROJECTS/SHM-SENSOR-NODE/build/bootloader-prefix/src/bootloader-stamp"
  "D:/WORK/PROJECT/ESP-IDF_PROJECTS/SHM-SENSOR-NODE/build/bootloader-prefix/src"
  "D:/WORK/PROJECT/ESP-IDF_PROJECTS/SHM-SENSOR-NODE/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/WORK/PROJECT/ESP-IDF_PROJECTS/SHM-SENSOR-NODE/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/WORK/PROJECT/ESP-IDF_PROJECTS/SHM-SENSOR-NODE/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
