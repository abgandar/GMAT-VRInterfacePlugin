//$Id$
//------------------------------------------------------------------------------
//                                  VRInterface
//------------------------------------------------------------------------------
// GMAT: General Mission Analysis Tool
//
// Declares VRInterface Class

#ifndef VRInterface_hpp
#define VRInterface_hpp


#include "Subscriber.hpp"
#include "VRInterfaceDefs.hpp"

#include <fstream>
#include <iostream>
#include <iomanip>			// for ostringstream
#include <iostream>			// unsure what for -- check
#include <string>				// for string::npos

//#include "PlotInterface.hpp"
#include "CoordinateConverter.hpp"
//#include "CoordinateSystem.hpp"

// class DataManager;

class VRInterface_API VRInterface : public Subscriber
{
public:
	// typical declarations 

	VRInterface(const std::string &type, const std::string &name);
	VRInterface(const VRInterface &op);
	VRInterface& operator=(const VRInterface&);
	virtual ~VRInterface();


	// methods inherited from GmatBase

	virtual bool Validate();
	virtual bool Initialize();

	virtual GmatBase* Clone() const;
	virtual void		Copy(const GmatBase* orig);

	virtual bool         SetName(const std::string &who,
		const std::string &oldName = "");

	virtual bool         TakeAction(const std::string &action,
		const std::string &actionData = "");

	virtual bool         RenameRefObject(const UnsignedInt type,
		const std::string &oldName,
		const std::string &newName);

	// methods inherited from Subscriber
	virtual bool Activate(bool state = true);

	virtual bool Distribute(const Real * dat, Integer len);

	// methods for parameters
	virtual std::string  GetParameterText(const Integer id) const;
	virtual Integer      GetParameterID(const std::string &str) const;
	virtual Gmat::ParameterType
		GetParameterType(const Integer id) const;
	virtual std::string  GetParameterTypeString(const Integer id) const;
	virtual bool         IsParameterReadOnly(const Integer id) const;

	virtual Integer      GetIntegerParameter(const Integer id) const;
	virtual Integer      SetIntegerParameter(const Integer id, const Integer value);
	virtual Integer      GetIntegerParameter(const std::string &label) const;
	virtual Integer      SetIntegerParameter(const std::string &label,
		const Integer value);

	virtual std::string  GetStringParameter(const Integer id) const;
	virtual std::string  GetStringParameter(const std::string &label) const;

	virtual bool         SetStringParameter(const std::string &label, const std::string &value);
	virtual bool         SetStringParameter(const std::string &label, const std::string &value, const Integer index);
	virtual bool         SetStringParameter(const Integer id, const std::string &value);
	virtual bool         SetStringParameter(const Integer id, const std::string &value, const Integer index);


	virtual const StringArray&
		GetStringArrayParameter(const Integer id) const;
	virtual const StringArray&
		GetStringArrayParameter(const std::string &label) const;

	virtual bool         GetBooleanParameter(const Integer id) const;
	virtual bool         GetBooleanParameter(const std::string &label) const;
	virtual bool         SetBooleanParameter(const Integer id,
		const bool value);
	virtual bool         SetBooleanParameter(const std::string &label,
		const bool value);

	virtual const BooleanArray&
		GetBooleanArrayParameter(const Integer id) const;
	virtual const BooleanArray&
		GetBooleanArrayParameter(const std::string &label) const;
	virtual bool         SetBooleanArrayParameter(const Integer id,
		const BooleanArray &valueArray);
	virtual bool         SetBooleanArrayParameter(const std::string &label,
		const BooleanArray &valueArray);

	virtual std::string  GetRefObjectName(const UnsignedInt type) const;
	virtual bool         HasRefObjectTypeArray();
	virtual const ObjectTypeArray&
		GetRefObjectTypeArray();
	virtual const StringArray&
		GetRefObjectNameArray(const UnsignedInt type);

	virtual GmatBase*    GetRefObject(const UnsignedInt type,
		const std::string &name);
	virtual bool         SetRefObject(GmatBase *obj, const UnsignedInt type,
		const std::string &name = "");

	virtual void         SetOrbitColorChanged(GmatBase *originator,
		const std::string &newColor,
		const std::string &objName,
		const std::string &desc);

	// for GUI population
	virtual UnsignedInt
		GetPropertyObjectType(const Integer id) const;

	bool GetShowObject(const std::string &name);
	void SetShowObject(const std::string &name, bool value);

	// internal methods
	const StringArray& GetSpacePointList();
	const StringArray& GetNonSpacecraftList();
	const StringArray& GetSpacecraftList(); // defined in GmatBase

protected:
	/// Calls PlotInterface for plotting non-solver data  
	bool         DataControl(const Real *dat, Integer len);
	
	/// Buffers published spacecraft orbit data
	virtual bool      BufferSpacecraftData(const Real *dat, Integer len);
	virtual bool		BufferCelestialBodyData(const Real *dat, Integer len);


	/// Returns object string list
	virtual std::string  GetObjectStringList() const;

	/// Adds Spacecraft and other objects to object arrays
	bool                 AddSpacePoint(const std::string &name, Integer index,
		bool show = true);
	/// Clears all object arrays called from TakeAction("Clear")
	bool                 ClearSpacePointList();
	/// Removes SpacePoint object from the object called from TakeAction("Remove")
	bool                 RemoveSpacePoint(const std::string &name);
	/// Finds the index of the element label from the element label array.
	Integer              FindIndexOfElement(StringArray &labelArray,
		const std::string &label);
	/// Builds dynamic arrays to pass to plotting canvas
	void                 BuildDynamicArrays();
	/// Clears dynamic arrays such as object name array, etc.
	void                 ClearDynamicArrays();
	/// Updates plotting object information such as plotting object pointer, option flags, etc.
	void                 UpdateObjectList(SpacePoint *sp, bool show = false);

	/// Debug message
	void                 WriteCoordinateSystem(CoordinateSystem *cs,
		const std::string &label = "");
	///
	bool						FixSpacePointArray();
	/// 
	Real						DeriveScRadius(const Real &scMass, const Real &scaler);
	///
	std::string				fileNameFromScript(const std::string &fileType);

	CoordinateSystem *mViewCoordSystem;

	std::string mOldName;
	std::string mViewCoordSysName;

	// object names and arrays
	std::vector<SpacePoint*> mObjectArray;	// list of SP and their options
	std::vector<SpacePoint*> mAllSpArray;
	std::vector<SpacePoint*> mCbArray;
	BooleanArray mDrawOrbitArray;
	BooleanArray mDrawObjectArray;

	StringArray mScNameArray;
	StringArray mObjectNameArray;
	StringArray mCbNameArray;		
	StringArray mAllSpNameArray;
	StringArray mAllRefObjectNames;

	Integer mAllSpCount;
	Integer mScCount;
	Integer mCbCount;	
	Integer mObjectCount;

	RealArray mSpRadii;
	Integer	 mScRadiiMin; 

	// user flags
	bool mExportAttitude;
	bool mExportColours;
	bool mDeriveRadii;

	// for data control
	Integer mDataCollectFrequency;
	Integer mNumPointsToRedraw;
	Integer mNumData;
	Integer mDataAbsentWarningCount;
	Integer mMaxData;
	bool isAbsentData;

	// arrays for holding distributed data
	RealArray mScXArray;
	RealArray mScYArray;
	RealArray mScZArray;
	RealArray mScVxArray;
	RealArray mScVyArray;
	RealArray mScVzArray;
	RealArray2D mScQArray;	// RealArray of RealArrays

	RealArray mCbXArray;
	RealArray mCbYArray;
	RealArray mCbZArray;
	RealArray mCbVxArray;
	RealArray mCbVyArray;
	RealArray mCbVzArray;
	RealArray2D mCbQArray;

	// arrays for holding previous distributed data
	BooleanArray mScPrevDataPresent;

	BooleanArray mCbPrevDataPresent;

	// maps for object and color
	// std::map<std::string, UnsignedInt> mCurrentOrbitColorMap;
	ColorMap mDefaultOrbitColorMap;

	// maps for object, flag, and epoch
	std::map<std::string, bool> mDrawOrbitMap;
	std::map<std::string, bool> mShowObjectMap;
	std::map<std::string, Real> mScInitialEpochMap;

	// file management
	// std::string jsonOutputPath;			// name of output path
	std::string jsonFileName;				// name of json file
	// std::string jsonFullPathFileName;	// name and path of file
	std::ofstream jstream;					// output data stream
		// this is currently passed to DataManager and not used in this class
		// consider declaring there instead if optimising 



	/// Enable drawing label for each added spacepoint (for output only)
	BooleanArray mDrawLabelArray; // y/n draw labels per each object (UNUSED)


	// An emumeration of parameters for VRInterface, starting at the end
	// of the @ref Subscriber::SubscriberParamCount enumeration.

	enum //template for now. Reflect changes across other 2 enums
	{
		ADD = SubscriberParamCount,    ///< Add spacepoint(s) to be plotted
		COORD_SYSTEM,                  ///< Set the plotting coordinate system
		EXPORT_ATTITUDE,
		EXPORT_TRAJECTORY_COLORS,
		DERIVE_RADII,
		SC_RADII,							 ///< Sets the radii of a spacecraft if spherical representation used
		//MODEL_FILE,							 ///< Sets the model file override for OpenFrames
		DATA_COLLECT_FREQUENCY,
		MAX_DATA,
		JSON_FILE,							///< Path of JSON file
		VRInterfaceParamCount,			 ///< Count of the parameters for this class
	};


private: // these will not be shared to other classes

	// these dictate the sizes of the index arrays to access enumerated
	// parameters
	static const Gmat::ParameterType
		PARAMETER_TYPE[VRInterfaceParamCount - SubscriberParamCount];
	static const std::string
		PARAMETER_TEXT[VRInterfaceParamCount - SubscriberParamCount];

};
#endif