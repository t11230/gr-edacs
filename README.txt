== gr-edacs ==

Authors: Clayton Caron, Tayler Burns

Description:
    The purpose of this project is to provide an easy to use block for tracking 
    EDACS trunked radio communications. It can be used with software defined 
    radios such as Nuand's bladeRF or the RTL-SDR in conjuction with the 
    OsmoSDR block.

Dependencies:
    GNU Radio >= v3.8.0
    BOOST
    CPPUNIT
    SWIG
    gr-dsd (github.com/argilo/gr-dsd)

Installation:
    cd gr-edacs
    mkdir build
    cd build
    cmake ../
    make
    sudo make install
    sudo ldconfig
