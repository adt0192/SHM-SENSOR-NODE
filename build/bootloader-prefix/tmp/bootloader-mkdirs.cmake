# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/andyd/esp-idf/esp-idf/components/bootloader/subproject"
  "D:/WORK/PROJECT/ESP-IDF_PROJECTS/TEST-ADXL355-TX-CS-AND-SAMPLE-REDIMENSION-ESP-IDF/build/bootloader"
  "D:/WORK/PROJECT/ESP-IDF_PROJECTS/TEST-ADXL355-TX-CS-AND-SAMPLE-REDIMENSION-ESP-IDF/build/bootloader-prefix"
  "D:/WORK/PROJECT/ESP-IDF_PROJECTS/TEST-ADXL355-TX-CS-AND-SAMPLE-REDIMENSION-ESP-IDF/build/bootloader-prefix/tmp"
  "D:/WORK/PROJECT/ESP-IDF_PROJECTS/TEST-ADXL355-TX-CS-AND-SAMPLE-REDIMENSION-ESP-IDF/build/bootloader-prefix/src/bootloader-stamp"
  "D:/WORK/PROJECT/ESP-IDF_PROJECTS/TEST-ADXL355-TX-CS-AND-SAMPLE-REDIMENSION-ESP-IDF/build/bootloader-prefix/src"
  "D:/WORK/PROJECT/ESP-IDF_PROJECTS/TEST-ADXL355-TX-CS-AND-SAMPLE-REDIMENSION-ESP-IDF/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/WORK/PROJECT/ESP-IDF_PROJECTS/TEST-ADXL355-TX-CS-AND-SAMPLE-REDIMENSION-ESP-IDF/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/WORK/PROJECT/ESP-IDF_PROJECTS/TEST-ADXL355-TX-CS-AND-SAMPLE-REDIMENSION-ESP-IDF/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
