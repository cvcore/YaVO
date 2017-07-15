include(ExternalProject)

ExternalProject_Add(ep_g2o
  GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o.git
  GIT_TAG master
  GIT_PROGESS 1
  PREFIX ${PROJECT_SOURCE_DIR}/ThirdParty/g2o
  UPDATE_COMMAND ""
  INSTALL_COMMAND ""
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DCMAKE_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/Dependencies/Install/g2o
  )
