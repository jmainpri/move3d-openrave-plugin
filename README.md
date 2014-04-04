move3d-openrave-plugin
==============

Dependencies:

* Move3D : http://www.openrobots.org/wiki/move3d
* OpenRave : http://openrave.org/docs/latest_stable/


Install:

    cd ormove3d
    mkdir build
    cd build
    cmake ..
    make install
    
Set up your enviroment:

    export OPENRAVE_PLUGINS=$HOME/move3d-openrave-plugin/plugins:$OPENRAVE_PLUGINS
    
Run:
    
    cd examples
    python test_move3d_birrt_stones.py
    
