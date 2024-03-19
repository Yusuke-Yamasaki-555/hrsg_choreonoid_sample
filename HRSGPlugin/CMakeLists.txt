option(BUILD_HRSG_PLUGIN ON)
if(NOT_BUILD_DEVELOPMENT)
  return()
endif()

choreonoid_add_plugin(CnoidHRSGPlugin HRSGPlugin.cpp)
target_link_libraries(CnoidHRSGPlugin PUBLIC CnoidBodyPlugin)
