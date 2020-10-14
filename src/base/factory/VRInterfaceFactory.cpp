//$Id$
//------------------------------------------------------------------------------
//                          VRInterface Subscriber Factory
//------------------------------------------------------------------------------
// GMAT: General Mission Analysis Tool.
/**
 * Implementation of a factory used in a GMAT plugin
 *
 * Replace code in the indicated places for your Factory
 */
//------------------------------------------------------------------------------


#include "VRInterfaceFactory.hpp"
// #include "VRV.hpp"
#include "VRInterface.hpp"

// Here you add include statements for each class that can be instantiated in
// this factory.  The example here is a SampleClass object.



//------------------------------------------------------------------------------
// VRInterfaceFactory()
//------------------------------------------------------------------------------
/**
 * Constructor
 *
 * Replace the type Gmat::COMMAND in the base class call with the factory type 
 * you need.
 */
//------------------------------------------------------------------------------
//Pick from: SUBSCRIBER; XY_PLOT; ORBIT_VIEW
//these are what you get off the context menu

VRInterfaceFactory::VRInterfaceFactory() :
   Factory(Gmat::SUBSCRIBER)
{
   if (creatables.empty())
   {
	// Replace the SampleClass string here with your class name.  For multiple 
	// classes of the same core type, push back multiple names here
	creatables.push_back("VRInterface");
   }
}


//------------------------------------------------------------------------------
// ~VRInterfaceFactory()
//------------------------------------------------------------------------------
/**
 * Destructor (tilde = destroys function)
 */
//------------------------------------------------------------------------------
VRInterfaceFactory::~VRInterfaceFactory()
{
}

//------------------------------------------------------------------------------
// SubscriberFactory(StringArray createList)
//------------------------------------------------------------------------------
/**
 * This method creates an object of the class SubscriberFactory
 * (constructor).
 *
 * @param <createList> initial list of creatable objects for this factory.
 *
 */
 //------------------------------------------------------------------------------
VRInterfaceFactory::VRInterfaceFactory(StringArray createList) :
Factory(createList, Gmat::SUBSCRIBER)
{
}

//------------------------------------------------------------------------------
// SampleFactory(const VRInterfaceFactory& elf)
//------------------------------------------------------------------------------
/**
 * Copy constructor
 *
 * @param elf The factory copied here
 */
//------------------------------------------------------------------------------
VRInterfaceFactory::VRInterfaceFactory(const VRInterfaceFactory& elf) :
   Factory           (elf)
{
   if (creatables.empty())
   {
	 // Replace the SampleClass string here with your class name.  For multiple 
	 // classes of the same type, push back multiple names here
      creatables.push_back("VRInterface");
   }
}


//------------------------------------------------------------------------------
// VRInterfaceFactory& operator=(const VRInterfaceFactory& elf)
//------------------------------------------------------------------------------
/**
 * Assignment operator
 *
 * @param elf The factory copied to this one
 *
 * @return this instance, set to match elf
 */
//------------------------------------------------------------------------------
VRInterfaceFactory& VRInterfaceFactory::operator=(
      const VRInterfaceFactory& elf)
{
   if (this != &elf)
   {
      Factory::operator=(elf);

      if (creatables.empty())
      {
         // Replace the SampleClass string here with your class name.  For multiple 
         // classes of the same type, push back multiple names here
         creatables.push_back("VRInterface");
      }
   }

   return *this;
}

// The following code is a sample implementation.  This is where you'll 
// implement one of the Create methods commented out in the header file. 
////------------------------------------------------------------------------------
//// GmatCommand* CreateCommand(const std::string &ofType,
////       const std::string &withName)
////------------------------------------------------------------------------------


// * Creation method for GMAT commands
// *
// * @param ofType The subtype of the command
// * @param withName The new command's name
// *
// * @return A newly created GmatCommand (or NULL if this factory doesn't create
// *         the requested type)
// */
////------------------------------------------------------------------------------
//GmatCommand* SampleFactory::CreateCommand(const std::string &ofType,
//                                     const std::string &withName)
//{
//   // This is how you'd implement creation of a ConsolePrint command 
//   if (ofType == "ConsolePrint")
//      return new ConsolePrint();
//   // add more here .......
//
//   return NULL;   // doesn't match any type of Command known by this factory
//}

//------------------------------------------------------------------------------
//  CreateObject(const std::string &ofType, const std::string &withName)
//------------------------------------------------------------------------------
/**
 * This method creates and returns an object of the requested Subscriber class
 * in generic way.
 *
 * @param <ofType> the Subscriber object to create and return.
 * @param <withName> the name to give the newly-created Subscriber object.
 *
 */
 //------------------------------------------------------------------------------
GmatBase* VRInterfaceFactory::CreateObject(const std::string &ofType,
	const std::string &withName)
{
	return CreateSubscriber(ofType, withName);
}

//------------------------------------------------------------------------------
//  CreateSubscriber(const std::string &ofType, const std::string &withName,
//                   const std::string &fileName)
//------------------------------------------------------------------------------
/**
 * This method creates and returns an object of the requested Subscriber class
 *
 * @param <ofType>   the Subscriber object to create and return.
 * @param <withName> the name to give the newly-created Subscriber object.
 *
 */
 //------------------------------------------------------------------------------
Subscriber* VRInterfaceFactory::CreateSubscriber(const std::string &ofType,
																 const std::string &withName)
{
	if (ofType == "VRInterface")
	{
		//return new VRInterface(ofType, withName);
		// add more here .......
		VRInterface *obj = new VRInterface(ofType, withName); //remember: withName = "" // these need to match up with class implementation
		return obj;
	}

   return nullptr;   // doesn't match any type of Command known by this factory
}
