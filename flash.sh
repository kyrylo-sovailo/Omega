#!/bin/sh

ENTRY_PACKAGES=turtlebot3_diff_drive_exercise

COLOR_RED='\033[0;31m'
COLOR_YELLOW='\033[0;33m'
COLOR_GREEN='\033[0;32m'
COLOR_DEFAULT='\033[0m'
printf_error()
{
    printf "${COLOR_RED}$1${COLOR_DEFAULT}\n"
}
printf_warning()
{
    printf "${COLOR_YELLOW}$1${COLOR_DEFAULT}\n"
}
printf_success()
{
    printf "${COLOR_GREEN}$1${COLOR_DEFAULT}\n"
}

if [ "$1" != "--root" -a "$1" != "--user-build" -a "$1" != "--user-copy" ]; then
    echo ENTRY STAGE
    if [ "$(whoami)" = "root" ]; then printf_error "Must not be run as root"; exit 1; fi
    if [ ! -f "$0" ]; then echo "${COLOR_ERROR}Script does not exist"; exit 1; fi
    if [ ! -e "$1" ]; then echo "${COLOR_ERROR}Device does not exist"; exit 1; fi
    if [   -n "$2" ]; then echo "${COLOR_ERROR}Too many parameters"; exit 1; fi
    export ENTRY_USER=$(whoami)
    export ENTRY_DIR=$(dirname $(readlink -f $0))
    export ENTRY_PATH=${PATH}
    export ENTRY_LIBRARY_PATH=${LD_LIBRARY_PATH}
    export ENTRY_DEVICE=$1
    sudo --preserve-env -- $0 --root


elif [ "$1" = "--root" ]; then
    echo ROOT STAGE
    if [ "$(whoami)" != "root" ]; then printf_error "Must be run as root"; exit 1; fi
    if [ -z "${ENTRY_USER}" -o -z "${ENTRY_DIR}" -o -z "${ENTRY_DEVICE}" ]; then printf_error "Environment invalid"; exit 1; fi

    printf_success "Check device correctness" #MILESTONE
    if [ $(blkid ${ENTRY_DEVICE} | grep PTUUID | wc -l) -ne 1 ]; then printf_error "Device invalid"; exit 1; fi
    if [ $(blkid ${ENTRY_DEVICE} | grep -o -e 'PTUUID="[a-zA-Z0-9-]*"' | cut -d \" -f 2 | wc -m) -gt 10 ]; then printf_error "Device PTUUID is unusually long"; exit 1; fi

    printf_success "Execute user build script" #MILESTONE
    sudo --user ${ENTRY_USER} --preserve-env -- $0 --user-build
    if [ $? -ne 0 ]; then printf_error "User build script failed"; exit 1; fi

    printf_success "Unmount partition" #MILESTONE
    umount ${ENTRY_DEVICE}1

    printf_success "Create partitions" #MILESTONE
    if [ ! -f commands.fdisk ]; then
        echo o>commands.fdisk
        echo n>>commands.fdisk
        echo p>>commands.fdisk
        echo 1>>commands.fdisk
        echo >>commands.fdisk
        echo >>commands.fdisk
        echo y>>commands.fdisk
        echo w>>commands.fdisk
    fi
    fdisk ${ENTRY_DEVICE} < commands.fdisk
    if [ $? -ne 0 ]; then printf_error "fdisk failed"; exit 1; fi
    sleep 1

    printf_success "Format partition" #MILESTONE
    mkfs.fat ${ENTRY_DEVICE}1 -n TURTLESTICK
    if [ $? -ne 0 ]; then printf_error "mkfs.fat failed"; exit 1; fi

    printf_success "Mount partition" #MILESTONE
    if [ ! -d "${ENTRY_DIR}/TURTLESTICK" ]; then
        mkdir "${ENTRY_DIR}/TURTLESTICK"
        if [ $? -ne 0 ]; then printf_error "mkdir failed"; exit 1; fi
        chown ${ENTRY_USER}:${ENTRY_USER} "${ENTRY_DIR}/TURTLESTICK"
        chmod u=rwx,g=rx,o=rx "${ENTRY_DIR}/TURTLESTICK"
    fi
    mount ${ENTRY_DEVICE}1 "${ENTRY_DIR}/TURTLESTICK" -o rw,nosuid,nodev,relatime,uid=${ENTRY_USER},gid=${ENTRY_USER},fmask=0,dmask=0,codepage=437,iocharset=ascii,shortname=mixed,showexec,utf8,flush,errors=remount-ro
    if [ $? -ne 0 ]; then printf_error "mount failed"; exit 1; fi

    printf_success "Execute user copy script" #MILESTONE
    sudo --user ${ENTRY_USER} --preserve-env -- $0 --user-copy
    if [ $? -ne 0 ]; then printf_error "User copy script failed"; fi

    printf_success "Unmount partition" #MILESTONE
    umount ${ENTRY_DEVICE}1


elif [ "$1" = "--user-build" ]; then
    echo USER BUILD SCRIPT
    if [ "$(whoami)" = "root" ]; then printf_error "Must not be run as root"; exit 1; fi
    if [ -z "${ENTRY_USER}" -o -z "${ENTRY_DIR}" -o -z "${ENTRY_DEVICE}" ]; then printf_error "Environment invalid"; exit 1; fi
    export PATH=${ENTRY_PATH}
    export LD_LIBRARY_PATH=${ENTRY_LIBRARY_PATH}

    printf_success "Switch to install profile" #MILESTONE
    cd ${ROS_WORKSPACE}
    catkin profile set install
    if [ $? -ne 0 ]; then printf_error "catkin profile set install failed"; exit 1; fi
    catkin config --install
    if [ $? -ne 0 ]; then printf_error "catkin config --install failed"; exit 1; fi

    printf_success "Build" #MILESTONE
    catkin clean --yes
    if [ $? -ne 0 ]; then printf_error "catkin clean failed"; exit 1; fi
    catkin build ${ENTRY_PACKAGES}
    if [ $? -ne 0 ]; then printf_error "catkin build failed"; exit 1; fi

    printf_success "Switch to default profile" #MILESTONE
    catkin config --no-install
    if [ $? -ne 0 ]; then printf_error "catkin config --no-install failed"; exit 1; fi
    catkin profile set default
    if [ $? -ne 0 ]; then printf_error "catkin profile set default failed"; exit 1; fi


elif [ "$1" = "--user-copy" ]; then
    echo USER COPY SCRIPT
    if [ "$(whoami)" = "root" ]; then printf_error "Must not be run as root"; exit 1; fi
    if [ -z "${ENTRY_USER}" -o -z "${ENTRY_DIR}" -o -z "${ENTRY_DEVICE}" ]; then printf_error "Environment invalid"; exit 1; fi
    export PATH=${ENTRY_PATH}
    export LD_LIBRARY_PATH=${ENTRY_LIBRARY_PATH}

    printf_success "Copy files" #MILESTONE
    cd "${ENTRY_DIR}/TURTLESTICK"
    FILES_DIR="$(rospack find turtlebot3_usb_launcher)/usb_stick_files"
    if [ ! -d "${FILES_DIR}" ]; then printf_error "usb_stick_files directory not found"; exit 1; fi
    cp -rp "${FILES_DIR}"/* .
    if [ $? -ne 0 ]; then printf_error "File copy failed"; exit 1; fi
    cp -r "${ROS_WORKSPACE}/../install_stick/." install/
    if [ $? -ne 0 ]; then printf_error "File copy failed"; exit 1; fi

    printf_success "Replace paths" #MILESTONE
    cd "${ENTRY_DIR}/TURTLESTICK/install"
    MATCH=$(readlink -f "${ROS_WORKSPACE}/../install_stick")
    REPLACE="/home/turtlebot/turtle/install"
    grep -r "${MATCH}" -l | grep .sh | tr '\n' ' ' | xargs sed -i "s@$MATCH@$REPLACE@g"

    printf_success "Delete launch files" #MILESTONE
    cd "${ENTRY_DIR}/TURTLESTICK"
    rm launch/*.launch

    printf_success "Copy start_robot.launch" #MILESTONE
    for PACKAGE in ${ENTRY_PACKAGES}
    do
        START_ROBOT_PATH=$(rospack find ${PACKAGE})/launch/start_robot.launch
        if [ -f "${START_ROBOT_PATH}" -a "${PACKAGE}" != "turtlebot3_behavior_exercise" ]; then
            PACKAGE_NEW_NAME=$(echo ${PACKAGE} | sed 's/turtlebot3_//g')
            cp "$START_ROBOT_PATH" "launch/${PACKAGE_NEW_NAME}.launch"
        fi
    done
else
    printf_error "Argument invalid"
fi