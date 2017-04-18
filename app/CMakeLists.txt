# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Tanis Mar
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname objects3DModeler)

file(GLOB modules ${CMAKE_CURRENT_SOURCE_DIR}/modules/*.xml)
file(GLOB apps ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB temps ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml.template)
file(GLOB conf    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB cloudsReal ${CMAKE_CURRENT_SOURCE_DIR}/sampleClouds/real/*.ply
                     ${CMAKE_CURRENT_SOURCE_DIR}/sampleClouds/real/*.pcd
		             ${CMAKE_CURRENT_SOURCE_DIR}/sampleClouds/real/rec/*.ply)
                 
file(GLOB cloudsSim ${CMAKE_CURRENT_SOURCE_DIR}/sampleClouds/sim/*.ply
                    ${CMAKE_CURRENT_SOURCE_DIR}/sampleClouds/sim/*.pcd)

file(GLOB plugins ${CMAKE_CURRENT_SOURCE_DIR}/plugins/*.lua)
         
### create a single target that installs all applications at once
yarp_install(FILES ${modules} DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})
yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${temps} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${plugins}  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname}/${PROJECTNAME})


yarp_install(FILES ${apps} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
yarp_install( FILES ${cloudsReal} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname}/sampleClouds/real)
yarp_install( FILES ${cloudsSim} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname}/sampleClouds/sim)

