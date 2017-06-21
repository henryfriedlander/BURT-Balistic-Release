set(BARRETT_ETC_PATH /etc/barrett/)
set(BARRETT_REL_PATH /home/${OWNER}/.barrett)
set(BARRETT_REL_LEFT_PATH /home/${OWNER}/.barrett/proficio_left)
set(BARRETT_REL_RIGHT_PATH /home/${OWNER}/.barrett/proficio_right)

configure_file(${PROJECT_SOURCE_DIR}/configurations/calibration.conf.left 
               ${BARRETT_REL_LEFT_PATH}/calibration.conf)
configure_file(${PROJECT_SOURCE_DIR}/configurations/wam3.conf.left 
               ${BARRETT_REL_LEFT_PATH}/wam3.conf)
configure_file(${PROJECT_SOURCE_DIR}/configurations/gravitycal.conf.left 
               ${BARRETT_REL_LEFT_PATH}/calibration_data/wam3/gravitycal.conf)
configure_file(${PROJECT_SOURCE_DIR}/configurations/LeftConfig.txt
               ${BARRETT_REL_LEFT_PATH}/calibration_data/wam3/LeftConfig.txt)
configure_file(${PROJECT_SOURCE_DIR}/configurations/zerocal.conf.left 
               ${BARRETT_REL_LEFT_PATH}/calibration_data/wam3/zerocal.conf)
configure_file(${PROJECT_SOURCE_DIR}/configurations/default.conf 
               ${BARRETT_REL_LEFT_PATH}/default.conf)
configure_file(${PROJECT_SOURCE_DIR}/configurations/user_gravity_compensation_grid.conf.left 
               ${BARRETT_REL_LEFT_PATH}/user_gravity_compensation_grid.conf)

configure_file(${PROJECT_SOURCE_DIR}/configurations/calibration.conf.right 
               ${BARRETT_REL_RIGHT_PATH}/calibration.conf)
configure_file(${PROJECT_SOURCE_DIR}/configurations/wam3.conf.right 
               ${BARRETT_REL_RIGHT_PATH}/wam3.conf)
configure_file(${PROJECT_SOURCE_DIR}/configurations/gravitycal.conf.right 
               ${BARRETT_REL_RIGHT_PATH}/calibration_data/wam3/gravitycal.conf)
configure_file(${PROJECT_SOURCE_DIR}/configurations/RightConfig.txt
               ${BARRETT_REL_RIGHT_PATH}/calibration_data/wam3/RightConfig.txt)
configure_file(${PROJECT_SOURCE_DIR}/configurations/zerocal.conf.right 
               ${BARRETT_REL_RIGHT_PATH}/calibration_data/wam3/zerocal.conf)
configure_file(${PROJECT_SOURCE_DIR}/configurations/default.conf 
               ${BARRETT_REL_RIGHT_PATH}/default.conf)
configure_file(${PROJECT_SOURCE_DIR}/configurations/user_gravity_compensation_grid.conf.right
               ${BARRETT_REL_LEFT_PATH}/user_gravity_compensation_grid.conf)

# Make left config as the default configuration
install(DIRECTORY ${BARRETT_REL_LEFT_PATH}/
        DESTINATION ${BARRETT_REL_PATH}
        PATTERN ".conf"
        PATTERN ".txt"
        PATTERN ".svn" EXCLUDE
        PATTERN ".git" EXCLUDE
        PATTERN "~" EXCLUDE
)

install(FILES ${PROJECT_SOURCE_DIR}/.bash_aliases 
        DESTINATION /home/${OWNER})
install(CODE "message(STATUS \"Changing the owner of ${BARRETT_REL_PATH} to ${OWNER}...\")")
install(CODE "execute_process(COMMAND chown -R ${OWNER}:${OWNER} ${BARRETT_REL_PATH})")

install(CODE "message(STATUS \"Running ldconfig...\")")
install(CODE "execute_process(COMMAND ldconfig)")
install(CODE "message(STATUS \"Running bash...\")")
install(CODE "execute_process(COMMAND . ~/.bashrc)")
