#!/bin/bash
NUM_JOBS=$( cat /proc/cpuinfo | grep "processor" | wc -l )
INSTALL_PREFIX="/usr/local/Ossian"
downloadFile(){
    if [ ! -f $2 ];
    then
        wget $1 -O $2
    else
        echo "$2 exists, skipping download"
    fi
}
doConfigure(){
    time $2
    if [ $? -eq 0 ] ; then
        echo "[$1] Configuration successful"
    else
        # Try to make again
        echo "[$1] Configuration failed " >&2
        echo "Please check the configuration being used"
        exit 1
    fi
}
doMake(){
    time make -j$NUM_JOBS
    if [ $? -eq 0 ] ; then
        echo "[$1] Make successful"
    else
        # Try to make again; Sometimes there are issues with the build
        # because of lack of resources or concurrency issues
        echo "[$1] Make did not build " >&2
        echo "Retrying ... "
        # Single thread this time
        make
        if [ $? -eq 0 ] ; then
            echo "[$1] Make successful"
        else
            # Try to make again
            echo "[$1] Make did not successfully build" >&2
            echo "Please fix issues and retry build"
            exit 1
        fi
    fi
}
doInstall(){
    echo "[$1] Installing ..."
    sudo make install
    sudo ldconfig
    if [ $? -eq 0 ] ; then
        echo "[$1] Installed in: $INSTALL_PREFIX"
    else
        echo "[$1] There was an issue with the final installation"
        exit 1
    fi
}