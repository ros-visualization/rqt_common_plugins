^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_py_console
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.2.16 (2013-04-09 13:33)
-------------------------

0.2.15 (2013-04-09 00:02)
-------------------------
* maintainer added to many pkgs

0.2.14 (2013-03-14)
-------------------

0.2.13 (2013-03-11 22:14)
-------------------------

0.2.12 (2013-03-11 13:56)
-------------------------

0.2.11 (2013-03-08)
-------------------

0.2.10 (2013-01-22)
-------------------

0.2.9 (2013-01-17)
------------------

0.2.8 (2013-01-11)
------------------
* update repo urls

0.2.7 (2012-12-24)
------------------

0.2.6 (2012-12-23)
------------------

0.2.5 (2012-12-21 19:11)
------------------------
* pep8 style changes

0.2.4 (2012-12-21 01:13)
------------------------

0.2.3 (2012-12-21 00:24)
------------------------

0.2.2 (2012-12-20 18:29)
------------------------

0.2.1 (2012-12-20 17:47)
------------------------

0.2.0 (2012-12-20 17:39)
------------------------
* fix missing roslib
* catkinize shell scripts and removed load_manifest calls
* catkinizing
  catkinize: remaining uncatkinized packages
  catkinize: removed rqt stack
  catkinize: removing Makefiles
  catkinize: added version tags
  catkinize: fixed version numbers
  catkinize: added setup.py files
  catkinize: added setup.py files
  catkinize: fixed cmake lists
  catkinize: removed rosdeps
  catkinize: building
  catkinize: added setup.py files
  catkinize: move plugin.xmls
  catkinize: add qt_gui depend
  catkinize: added repositories
  fix deps
  add .gitignore
  fix export tag in rqt_gui_py
  add missing target_link_libraries
  catkinize: rqt_msg, rqt_bag, rqt_console, rqt_bag_plugins now work in install space
  catkinize: all python plugins run from their scripts in install space
  rqt_nav_view: Removed extra plugin install rule.
  disable graying out unavailable python plugins
  catkinize: c++ plugins working, rosruning working in install/build spaces
  catkinize: removed rosmake style .gitignore files
  catkinize: fixed the "s"
  add missing inc dirs

0.1.7 (2012-12-13 16:17)
------------------------

0.1.6 (2012-12-13 14:43:47)
---------------------------

0.1.5 (2012-12-08)
------------------
* rqt: conversion of scripts to new qt_gui standalone package argument system
* rqt: fix for standalone scripts to allow for standalone arguments
* rqt: script overhaul for running plugins in standalone mode
* add separate export and base class for ros-agnostic plugins (py_console, shell, web plugins) to not depend on rqt_gui
* fix finding py_console, shell and web plugins with roslib

0.1.4 (2012-11-20)
------------------

0.1.3 (2012-11-19)
------------------
* rqt_various: added or fixed support for appending iteration numbers (%d) to the titles to easily identify multiple plugin instances
* changed import style for Qt modules to use python_qt_binding directly
* adapted qt imports to new qt_binding_helper
* rqt_plot: merged with rqt_matplot and improved usability
* py_console: changed dependencies from rqt to qt_gui
* py_console: use the same rqt widget for all consoles to avoid new
  windows placement on switching
* py_console: small fix in restore settings
* py_console: make use of simple settings dialog
* py_console: added alternative console with autocompletion based on
  spyderlib
* rqt_py_console: added .gitignore file for bin/build directories
* removed console_text_edit and made py_console use the one
  in qt_gui_py_common
* added simple interactive python console plugin
