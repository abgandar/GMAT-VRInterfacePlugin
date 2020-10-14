//$Id$
//------------------------------------------------------------------------------
//                            GmatPluginFunctions
//------------------------------------------------------------------------------
// GMAT: General Mission Analysis Tool
/**
 * Definition for library code interfaces.
 */
//------------------------------------------------------------------------------
#ifndef GmatPluginFunctions_hpp
#define GmatPluginFunctions_hpp

#include "VRInterfaceDefs.hpp"
#include "Factory.hpp"

class MessageReceiver;


extern "C"	//entry point into plugin components here
{
   Integer    VRInterface_API GetFactoryCount();
   Factory    VRInterface_API *GetFactoryPointer(Integer index);
   void       VRInterface_API SetMessageReceiver(MessageReceiver* mr);
};

#endif /*GmatPluginFunctions_hpp*/