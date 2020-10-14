//$Id$
//------------------------------------------------------------------------------
//                          VRInterface Subscriber Factory
//------------------------------------------------------------------------------
// GMAT: General Mission Analysis Tool.
//
/**
 * Definition of the factory used to create subscribers in this
 * plugin
 */
//------------------------------------------------------------------------------


#ifndef VRInterfaceFactory_hpp
#define VRInterfaceFactory_hpp

#include "VRInterfaceDefs.hpp"
#include "Factory.hpp"


/**
 * Factory class that creates one class of your plugin's objects
 *
 * Rename this class from VRInterfaceFactory( to something more descriptive of your 
 * plugin object(s).
 *
 * @note: The "SAMPLE_API" descriptor needs to match the definition in your 
 * ...Defs.hpp file
 */
class VRInterface_API VRInterfaceFactory : public Factory
{
public:
	//default constructor
   VRInterfaceFactory();

	//constructor
	VRInterfaceFactory(StringArray creatList);

	//destructor (although in SubFactory.cpp, this is not of type virtual
   virtual ~VRInterfaceFactory();

	//copy constructor
   VRInterfaceFactory(const VRInterfaceFactory& elf);

	//assignment operator
   VRInterfaceFactory& operator=(const VRInterfaceFactory& elf); 

   // these declarations seem to be used in all plugins

	virtual GmatBase* CreateObject(const std::string &ofType,
											 const std::string &withName = "");

   virtual Subscriber* CreateSubscriber(const std::string &ofType,
													 const std::string &withName = "");
//	                                       const std::string &fileName = "");
	//file name is if using report file, may be good to implement 


   //factory templates here
	// Implement one and only one of the following per Factory:
	  //virtual SpaceObject*     CreateSpacecraft(const std::string &ofType,
	  //                                          const std::string &withName = "");
	  //virtual SpacePoint*      CreateSpacePoint(const std::string &ofType,
	  //                                          const std::string &withName = "");        
	  //virtual Propagator*      CreatePropagator(const std::string &ofType,
	  //                                          const std::string &withName = "");
	  //virtual ODEModel*        CreateODEModel(const std::string &ofType,
	  //                                          const std::string &withName = "");
	  //virtual PhysicalModel*   CreatePhysicalModel(const std::string &ofType,
	  //                                             const std::string &withName = "");
	  //virtual PropSetup*       CreatePropSetup(const std::string &ofType,
	  //                                         const std::string &withName = "");
	  //virtual Parameter*       CreateParameter(const std::string &ofType,
	  //                                         const std::string &withName = "");
	  //virtual Burn*            CreateBurn(const std::string &ofType,
	  //                                    const std::string &withName = "");
	  //virtual StopCondition*   CreateStopCondition(const std::string &ofType,
	  //                                             const std::string &withName = "");
	  //virtual CalculatedPoint* CreateCalculatedPoint(const std::string &ofType,
	  //                                               const std::string &withName = "");
	  //virtual CelestialBody*   CreateCelestialBody(const std::string &ofType,
	  //                                             const std::string &withName = "");
	  //virtual SolarSystem*     CreateSolarSystem(const std::string &ofType,
	  //                                           const std::string &withName = "");
	  //virtual Solver*          CreateSolver(const std::string &ofType,
	  //                                      const std::string &withName = ""); 
	  //virtual Subscriber*      CreateSubscriber(const std::string &ofType,
	  //                                          const std::string &withName = "",
	  //                                          const std::string &fileName = "");
	  //virtual EphemerisFile*   CreateEphemerisFile(const std::string &ofType,
	  //                                             const std::string &withName = "");
	  //virtual GmatCommand*     CreateCommand(const std::string &ofType,
	  //                                       const std::string &withName = "");
	  //virtual AtmosphereModel* CreateAtmosphereModel(const std::string &ofType,
	  //                                               const std::string &withName = "",
	  //                                               const std::string &forBody = "Earth");
	  //virtual Function*        CreateFunction(const std::string &ofType,
	  //                                        const std::string &withName = "");
	  //virtual Hardware*        CreateHardware(const std::string &ofType,
	  //                                        const std::string &withName = "");
	  //virtual AxisSystem*      CreateAxisSystem(const std::string &ofType,
	  //                                          const std::string &withName = "");
	  //virtual CoordinateSystem* CreateCoordinateSystem(const std::string &ofType,
	  //                                                 const std::string &withName = "");
	  //virtual MathNode*        CreateMathNode(const std::string &ofType,
	  //                                        const std::string &withName = "");
	  //virtual Attitude*        CreateAttitude(const std::string &ofType,
	  //                                        const std::string &withName = "");
	  //virtual MeasurementModel*
	  //                         CreateMeasurementModel(const std::string &ofType,
	  //                                        const std::string &withName = "");
	  //virtual CoreMeasurement* CreateMeasurement(const std::string &ofType,
	  //                                        const std::string &withName = "");
	  //virtual DataFile*        CreateDataFile(const std::string &ofType,
	  //                                        const std::string &withName = "");
	  //virtual ObType*          CreateObType(const std::string &ofType,
	  //                                      const std::string &withName = "");
	  //virtual TrackingSystem*  CreateTrackingSystem(const std::string &ofType,
	  //                                        const std::string &withName = "");
	  //virtual TrackingData*    CreateTrackingData(const std::string &ofType,
	  //                                        const std::string &withName = "");
	  //virtual Event*           CreateEvent(const std::string &ofType,
	  //                                     const std::string &withName = "");
	  //virtual EventLocator*    CreateEventLocator(const std::string &ofType,
	  //                                     const std::string &withName = "");
	  //virtual Interface*       CreateInterface(const std::string &ofType,
	  //                                         const std::string &withName = "");
};

#endif /* VRInterfaceFactory_hpp */
