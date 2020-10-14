# $Id$
# 
# GMAT: General Mission Analysis Tool.
# 
# CMAKE script file for GMAT SaveCommand Plugin.
# (Adapted from the CInterface plugin CMake script)
# This file must be installed in the plugins/SaveCommandPlugin/src directory
#
# Author: Ravi Mathur
#  
# DO NOT MODIFY THIS FILE UNLESS YOU KNOW WHAT YOU ARE DOING!
#
Set (VERSION 1)

MESSAGE("==============================")
MESSAGE("GMAT VRInterface Plugin setup " ${VERSION}) 

SET(TargetName VRInterface)

# Add Preproccessor Definitions
ADD_DEFINITIONS("-DVRInterface_EXPORTS")

# ====================================================================
# list of directories containing source/header files
SET(PLUGIN_DIRS base/subscriber base/factory base/include base/plugin
							gui/guifactory 	gui/subscriber)

# ====================================================================
# source files
SET(PLUGIN_SRCS
	base/subscriber/VRInterface.cpp
	base/subscriber/DataManager.cpp
    base/factory/VRInterfaceFactory.cpp
    base/plugin/GmatPluginFunctions.cpp
)

# ====================================================================
# Setup common plugin definitions, targets, etc.
_SETUPPLUGIN(${TargetName} "${PLUGIN_DIRS}" "${PLUGIN_SRCS}" plugins)
