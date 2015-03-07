# ROS doc

### Subscribed topics
#### image_input ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
Incoming raw image to apply filters on. Can be modified with the image_input parameter.

### Published topics
#### image_filtered ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
Output image after the filters have been applied. Can be modified with the ~image_output parameter.

### Services
~show_gui (std_srvs/Empty)<br/>
Shows the Seagoat GUI.

### Parameters
~gui (bool, default: False)<br/>
Whether or not to show the GUI of the application

~image_input (string, default: image_input) <br/>
Topic on which to listen for images

~image_output<br/>
Topic on which to send the filtered images

# Seagoat doc

This Vision Serveur is tested on OpenCV 2.4.2 and Python 2.7 on Fedora 17 and Ubuntu 12.04.

Requirements:
 - Python 2.7
 - PyQt4
 - Glade 3
 - OpenCV 2.4 w/ Python/Numpy bindings
 - Numpy
 - Scipy

Until OpenCV 2.4 is fully supported, the preferred way is to compile OpenCV manually: 
 - http://opencv.willowgarage.com/wiki/InstallGuide

==== INSTALLATION ====

A. Install dependencies

  Ubuntu :

    Execution only :
 sudo apt-get install python python-numpy python-scipy python-opencv python-protobuf protobuf

    Compilation proto : 
 sudo apt-get install protobuf-compiler

    Client qt :
 sudo apt-get install python-pyside python-qt4


  Fedora : 

    Execution only :
 sudo apt-get install python numpy scipy opencv-python protobuf-python protobuf

    Compilation proto :
 sudo apt-get install protobuf-compiler

    Client qt :
 sudo apt-get install python-pyside PyQt4

  Windows :
	Install the following dependencies :
 - Python: 	http://python.org/ftp/python/2.7.3/python-2.7.3.msi
 - Numpy: 	http://sourceforge.net/projects/numpy/files/NumPy/	# Choose the installer
 - Scipy:	http://sourceforge.net/projects/scipy/files/scipy/	# Choose the installer
 - PyQt4:	http://www.riverbankcomputing.co.uk/software/pyqt/download
 - PySide: 	http://qt-project.org/wiki/PySide_Binaries_Windows
 - PIL:		http://effbot.org/downloads/PIL-1.1.7.win32-py2.7.exe
 - OpenCV: 	http://www.lfd.uci.edu/~gohlke/pythonlibs/#opencv	# OpenCV installer for Windows.


  Fedora : 

    Execution only :
 sudo apt-get install python numpy scipy opencv-python protobuf-python protobuf

    Compilation proto :
 sudo apt-get install protobuf-compiler

    Client qt :
 sudo apt-get install python-pyside PyQt4

  Windows :
	Install the following dependencies :
 - Python: 	http://python.org/ftp/python/2.7.3/python-2.7.3.msi
 - Numpy: 	http://sourceforge.net/projects/numpy/files/NumPy/	# Choose the installer
 - Scipy:	http://sourceforge.net/projects/scipy/files/scipy/	# Choose the installer
 - PyQt4:	http://www.riverbankcomputing.co.uk/software/pyqt/download
 - PySide: 	http://qt-project.org/wiki/PySide_Binaries_Windows
 - PIL:		http://effbot.org/downloads/PIL-1.1.7.win32-py2.7.exe
 - OpenCV: 	http://www.lfd.uci.edu/~gohlke/pythonlibs/#opencv	# OpenCV installer for Windows.

B. Install OpenCV 2.4

 1. Install required OpenCV dependencies
   sudo apt-get install cmake cmake-gui gcc pkg-config libavformat-dev libswscale-dev

 2. Download the archive manually 
  From here: http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.2/OpenCV-2.4.2.tar.bz2/
  Go to directory containing downloaded file with a command line.
 
 3. Extract the archive
  tar -jxvf OpenCV-2.4.2.tar.bz2 && cd OpenCV-2.4.2

 4. Configure
  mkdir release
  cd release
  cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_PYTHON_SUPPORT=ON ..

 5. Compile
  make -j

 6. Do crazy stuff!

 More information is available here: http://opencv.willowgarage.com/wiki/InstallGuide

