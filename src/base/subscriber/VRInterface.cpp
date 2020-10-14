//$Id$
//------------------------------------------------------------------------------
//                                  VRInterface
//------------------------------------------------------------------------------
// GMAT: General Mission Analysis Tool
//
// Implements VRInterface Class

#include "VRInterface.hpp"
#include "MessageInterface.hpp"		// for debugging
#include "SubscriberException.hpp"	// for SubscriberException()
#include "TextParser.hpp"          // for SeparateBrackets()
#include "StringUtil.hpp"          // for ToReal()
#include "CoordinateConverter.hpp" // for Convert()
#include "SpaceObject.hpp"         // for GetEpoch()
#include "Spacecraft.hpp"			  // for deriving radii
#include "RgbColor.hpp"            // for Color::ToIntColor()
#include "ColorTypes.hpp"          // for namespace GmatColor::
#include "FileUtil.hpp"				  // for fileName validation
#include "AttitudeConversionUtility.hpp"	// for attitude conversation
#include "Moderator.hpp"				// for GetScriptFileName()
#include <cmath>						  // for M_PI

#include "DataManager.hpp"

#define _USE_MATH_DEFINES

#define DEBUG_BUFFER_UPDATE

#define __REMOVE_OBJ_BY_SETTING_FLAG__
#define __SHOW_WARNING_FOR_UNPUBLISHED_OBJ__


//---------------------------------
// static data
//---------------------------------
// remember to keep order
// prevent linker errors
const std::string
VRInterface::PARAMETER_TEXT[VRInterfaceParamCount - SubscriberParamCount] =
{
	"Add",
	"CoordinateSystem",
	"ExportAttitude",
	"ExportTrajectoryColours",
	"DeriveRadii",
	"MinimumSpacecraftRadii", 
	//"ModelFile",
	"DataCollectFrequency",
	"MaxDataPoints",
	"JsonFileLocation"
};


const Gmat::ParameterType
VRInterface::PARAMETER_TYPE[VRInterfaceParamCount - SubscriberParamCount] =
{
	Gmat::OBJECTARRAY_TYPE,			//"Add",
	Gmat::OBJECT_TYPE,            //"CoordinateSystem"
	Gmat::BOOLEAN_TYPE,				//"ExportAttitude",
	Gmat::BOOLEAN_TYPE,				//"ExportTrajectoryColours",
	Gmat::BOOLEAN_TYPE,				//"DeriveRadii",
	Gmat::INTEGER_TYPE,				//"MinimumSpacecraftRadii",
	//Gmat::FILENAME_TYPE,				//"ModelFile",
	Gmat::INTEGER_TYPE,				//"DataCollectFrequency",
	Gmat::INTEGER_TYPE,           //"MaxDataPoints"
	Gmat::FILENAME_TYPE,				//"JsonFile",

};


//------------------------------------------------------------------------------
// VRInterface(const std::string &type, const std::string &name)
//------------------------------------------------------------------------------
/**
 * The default constructor
 * Declaring and initializing default variables here
 * 
 * @param <type> Typename
 * @param <name> Name of the object
 * OPTIONAL @param for file management
 */
 //------------------------------------------------------------------------------
VRInterface::VRInterface(const std::string &type, const std::string &name)
	: Subscriber(type, name)
{
	// GmatBase data 
	parameterCount = VRInterfaceParamCount;
	objectTypes.push_back(GmatType::GetTypeId("VRInterface"));
	objectTypeNames.push_back("VRInterface");

	mViewCoordSystem = NULL; // or nullptr

	// use name of mission as default json name
	jsonFileName = fileNameFromScript(".json");

	// jsonFileName = { R"(C:\Users\User\Documents\Unity\JSONs\json.json)" };

	mSolverIterOption = SI_NONE;

	mOldName = instanceName;
	mViewCoordSysName = "EarthMJ2000Eq";

	mScRadiiMin = 50;

	mDataCollectFrequency = 1;
	mNumData = 0;
	mMaxData = 20000;
	mDataAbsentWarningCount = 0;

	mScNameArray.clear();
	mCbNameArray.clear();
	mObjectNameArray.clear();
	mAllSpNameArray.clear();
	mAllRefObjectNames.clear();
	mObjectArray.clear();
	mDrawOrbitArray.clear();
	mDrawObjectArray.clear();
	mAllSpArray.clear();
	mCbArray.clear();

	mSpRadii.clear();

	mScXArray.clear();
	mScYArray.clear();
	mScZArray.clear();
	mScVxArray.clear();
	mScVyArray.clear();
	mScVzArray.clear();
	mScQArray.clear();
	mScPrevDataPresent.clear();

	mCbXArray.clear();
	mCbYArray.clear();
	mCbZArray.clear();
	mCbVxArray.clear();
	mCbVyArray.clear();
	mCbVzArray.clear();
	mCbQArray.clear();
	mCbPrevDataPresent.clear();

	mDrawOrbitMap.clear();
	mShowObjectMap.clear();

	mDefaultOrbitColorMap.clear();

	mAllSpCount = 0;
	mScCount = 0;
	mObjectCount = 0;
	mCbCount = 0;

	mExportAttitude = true;
	mExportColours = true;
	mDeriveRadii = true;

	isAbsentData = false;
}

//------------------------------------------------------------------------------
// VRInterface(const VRInterface &vri)
//------------------------------------------------------------------------------
/**
 * The copy constructor
 */
 //------------------------------------------------------------------------------
VRInterface::VRInterface(const VRInterface &vri)
	: Subscriber(vri)
{
	mViewCoordSystem = vri.mViewCoordSystem;

	jsonFileName = vri.jsonFileName;

	mSolverIterOption = vri.mSolverIterOption;

	mOldName = vri.mOldName;
	mViewCoordSysName = vri.mViewCoordSysName;

	mScRadiiMin = vri.mScRadiiMin;

	mDataCollectFrequency = vri.mDataCollectFrequency;
	mMaxData = vri.mMaxData;

	mAllSpCount = vri.mAllSpCount;
	mScCount = vri.mScCount;
	mObjectCount = vri.mObjectCount;

	mObjectArray = vri.mObjectArray;
	mCbNameArray = vri.mCbNameArray;
	mDrawOrbitArray = vri.mDrawOrbitArray;
	mDrawObjectArray = vri.mDrawObjectArray;
	mAllSpArray = vri.mAllSpArray;
	mScNameArray = vri.mScNameArray;
	mObjectNameArray = vri.mObjectNameArray;
	mAllSpNameArray = vri.mAllSpNameArray;
	mAllRefObjectNames = vri.mAllRefObjectNames;
	mCbArray = vri.mCbArray;

	mSpRadii = vri.mSpRadii;

	mScXArray = vri.mScXArray;
	mScYArray = vri.mScYArray;
	mScZArray = vri.mScZArray;
	mScVxArray = vri.mScVxArray;
	mScVyArray = vri.mScVyArray;
	mScVzArray = vri.mScVzArray;
	mScQArray = vri.mScQArray;
	mScPrevDataPresent = vri.mScPrevDataPresent;

	mCbXArray = vri.mCbXArray;
	mCbYArray = vri.mCbYArray;
	mCbZArray = vri.mCbZArray;
	mCbVxArray = vri.mCbVxArray;
	mCbVyArray = vri.mCbVyArray;
	mCbVzArray = vri.mCbVzArray;
	mCbQArray = vri.mCbQArray;
	mCbPrevDataPresent = vri.mCbPrevDataPresent;

	mDrawOrbitMap = vri.mDrawOrbitMap;
	mShowObjectMap = vri.mShowObjectMap;

	mDefaultOrbitColorMap = vri.mDefaultOrbitColorMap;

	mNumData = vri.mNumData;
	mDataAbsentWarningCount = vri.mDataAbsentWarningCount;

	mExportAttitude = vri.mExportAttitude;
	mExportColours = vri.mExportColours;
	mDeriveRadii = vri.mDeriveRadii;

	isAbsentData = vri.isAbsentData;

	parameterCount = VRInterfaceParamCount;
}

//------------------------------------------------------------------------------
// VRInterface& operator=(const VRInterface &vri)
//------------------------------------------------------------------------------
/**
 * The assignment operator
 */
 //------------------------------------------------------------------------------
VRInterface& VRInterface::operator=(const VRInterface& vri)
{
	if (this == &vri)
		return *this;

	Subscriber::operator=(vri);

	mViewCoordSystem = vri.mViewCoordSystem;

	mSolverIterOption = vri.mSolverIterOption;

	jsonFileName = vri.jsonFileName;

	mOldName = vri.mOldName;
	mViewCoordSysName = vri.mViewCoordSysName;

	mScRadiiMin = vri.mScRadiiMin;

	mDataCollectFrequency = vri.mDataCollectFrequency;
	mMaxData = vri.mMaxData;

	mAllSpCount = vri.mAllSpCount;
	mScCount = vri.mScCount;
	mObjectCount = vri.mObjectCount;

	mObjectArray = vri.mObjectArray;
	mCbNameArray = vri.mCbNameArray;
	mDrawOrbitArray = vri.mDrawOrbitArray;
	mDrawObjectArray = vri.mDrawObjectArray;
	mAllSpArray = vri.mAllSpArray;
	mScNameArray = vri.mScNameArray;
	mObjectNameArray = vri.mObjectNameArray;
	mAllSpNameArray = vri.mAllSpNameArray;
	mAllRefObjectNames = vri.mAllRefObjectNames;
	mCbArray = vri.mCbArray;

	mSpRadii = vri.mSpRadii;

	mScXArray = vri.mScXArray;
	mScYArray = vri.mScYArray;
	mScZArray = vri.mScZArray;
	mScVxArray = vri.mScVxArray;
	mScVyArray = vri.mScVyArray;
	mScVzArray = vri.mScVzArray;
	mScQArray = vri.mScQArray;
	mScPrevDataPresent = vri.mScPrevDataPresent;

	mCbXArray = vri.mCbXArray;
	mCbYArray = vri.mCbYArray;
	mCbZArray = vri.mCbZArray;
	mCbVxArray = vri.mCbVxArray;
	mCbVyArray = vri.mCbVyArray;
	mCbVzArray = vri.mCbVzArray;
	mCbQArray = vri.mCbQArray;
	mCbPrevDataPresent = vri.mCbPrevDataPresent;

	mDrawOrbitMap = vri.mDrawOrbitMap;
	mShowObjectMap = vri.mShowObjectMap;

	mDefaultOrbitColorMap = vri.mDefaultOrbitColorMap;

	mNumData = vri.mNumData;
	mDataAbsentWarningCount = vri.mDataAbsentWarningCount;

	mExportAttitude = vri.mExportAttitude;
	mExportColours = vri.mExportColours;
	mDeriveRadii = vri.mDeriveRadii;

	isAbsentData = vri.isAbsentData;
	return *this;
}

//------------------------------------------------------------------------------
// ~VRInterface()
//------------------------------------------------------------------------------
/**
 * Destructor
 *
 * Clear buffers, delete caches etc.
 */
 //------------------------------------------------------------------------------
VRInterface::~VRInterface()
{
	jstream.flush();
	jstream.close();

	// clear buffers, delete cache data etc.
	// to prevent access violation exception, a class derived form
}




//------------------------------------------------------------------------------
// Methods inherited from GmatBase
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
//  bool Validate()
//------------------------------------------------------------------------------
/**
 * Performs any pre-run validation that the object needs.
 * Called on GUI window save
 *
 * @return true unless validation fails.
 */
 //------------------------------------------------------------------------------
bool VRInterface::Validate() {

	if (jsonFileName == "") {
		MessageInterface::PopupMessage(Gmat::WARNING_, "You must create/point to an existing JSON file. \n"
			"This cannot be blank");
		return false;
	}
	else {
		size_t found = jsonFileName.find(".json");
		if (found == std::string::npos) {
			MessageInterface::PopupMessage(Gmat::WARNING_, "Are you sure this is a valid JSON file?");
			return false;
		}
	}

	if (mAllSpNameArray.size() == 0) {
		MessageInterface::PopupMessage(Gmat::ERROR_, "No spacepoints selected\n"
			"This will cause a crash on run.");
		return false;
	}

	return true;
}


//------------------------------------------------------------------------------
// virtual bool Initialize()
//------------------------------------------------------------------------------
/**
* Set pointers and assignment. Initialise instance of interface.
*/
//------------------------------------------------------------------------------
bool VRInterface::Initialize() {
	bool foundSc = false;
	bool retval = false;

	if (GmatGlobal::Instance()->GetRunMode() == GmatGlobal::TESTING_NO_PLOTS)
		return true;

	Subscriber::Initialize(); // all others have this. Why?

	if (active && !isInitialized) {
		// if subscriber active, and this not yet initialized
		// stuff to do on start of mission run happens here 
		// careful with this if statement. Moved from lower down

		Integer nullCounter = 0;

		if (mAllSpCount == 0)
		{
			active = false;
			MessageInterface::ShowMessage
			("*** WARNING *** The %s named \"%s\" will be turned off. "
				"No SpacePoints were added to plot.\n", GetTypeName().c_str(), GetName().c_str());
			return false;
		}

		// check for spacecaft is included in the plot
		for (int i = 0; i < mAllSpCount; i++)
		{
			if (mAllSpArray[i])
			{
				if (mAllSpArray[i]->IsOfType(Gmat::SPACECRAFT))
				{
					foundSc = true;
					break;
				}
			}
			else
				nullCounter++;
		}

		if (nullCounter == mAllSpCount)	// if nothing selected
		{
			active = false;
			MessageInterface::ShowMessage
			("*** WARNING *** The %s named \"%s\" will be turned off.  "
				"%d SpaceObjects have NULL pointers.\n", GetTypeName().c_str(),
				GetName().c_str(), nullCounter);
			return false;
		}

		//if (!foundSc)	// turning this off, as purely planetary systems are necessary
		//{
		//	active = false;
		//	MessageInterface::ShowMessage
		//	("*** WARNING *** The %s named \"%s\" will be turned off. "
		//		"No Spacecraft was added to plot.\n", GetTypeName().c_str(), GetName().c_str());
		//	return false;
		//}

		// Build color maps
		for (int i = 0; i < mAllSpCount; i++) {
			SpacePoint *sp = mAllSpArray[i];
			if (sp) {
				mDefaultOrbitColorMap[mAllSpArray[i]->GetName()] = mAllSpArray[i]->GetCurrentOrbitColor();
			}
		}

		ClearDynamicArrays();
		BuildDynamicArrays();
		DataManager bdg;
			bdg.BuildDynamicBuffers(mObjectCount);

		isInitialized = true;
		retval = true;
	}
	else {
		if (!active) {
			retval = false;
		}
	}
	return retval;
}



//------------------------------------------------------------------------------
//  GmatBase* Clone() const
//------------------------------------------------------------------------------
/**
 * This method returns a clone of the VRInterface.
 *
 * @return clone of the VRInterface.
 *
 */
 //------------------------------------------------------------------------------
GmatBase* VRInterface::Clone() const
{
	return (new VRInterface(*this));
}


//------------------------------------------------------------------------------
// void Copy(const GmatBase* orig)
//------------------------------------------------------------------------------
/**
 * Sets this object to match another one.
 *
 * @param orig The original that is being copied.
 */
 //------------------------------------------------------------------------------
void VRInterface::Copy(const GmatBase* orig)
{
	operator=(*((VRInterface *)(orig)));
}


//------------------------------------------------------------------------------
// bool SetName(const std::string &who, const std;:string &oldName = "")
//------------------------------------------------------------------------------
/**
 * Set the name for this instance.
 *
 * @see GmatBase
 */
 //------------------------------------------------------------------------------
bool VRInterface::SetName(const std::string &who, const std::string &oldName)
{
	if (oldName == "")
		mOldName = instanceName;
	else
		mOldName = oldName;

	return GmatBase::SetName(who);
}


//------------------------------------------------------------------------------
// virtual bool TakeAction(const std::string &action,  
//                         const std::string &actionData = "");
//------------------------------------------------------------------------------
/**
 * This method performs action. It is a utility method that derived
 * classes override to provide functionality that cannnot be
 * implemented through basic parameter setting calls.
 *
 * @param <action> action to perform
 * @param <actionData> action data associated with action
 * @return true if action successfully performed
 *
 */
 //------------------------------------------------------------------------------
bool VRInterface::TakeAction(const std::string &action,
	const std::string &actionData)
{
	if (action == "Clear") {
		return ClearSpacePointList();
	}
	else if (action == "Remove") {
		return RemoveSpacePoint(actionData);
	}
	else if (action == "Finalize") {
		// This action is usually called when GMAT function finalizes
		// PlotInterface::DeleteGlPlot(instanceName);
	}
	else if (action == "PenUp") {
		//isDataOn = false;
		active = false;
		return true;
	}
	else if (action == "PenDown") {
		//isDataOn = true;
		active = true;
		return true;
	}

	return false;
}


//---------------------------------------------------------------------------
//  bool RenameRefObject(const UnsignedInt type,
//                       const std::string &oldName, const std::string &newName)
//---------------------------------------------------------------------------
/**
* Resets the reference object name when the reference object
* is renamed elsewhere.
*/
//------------------------------------------------------------------------------

bool VRInterface::RenameRefObject(const UnsignedInt type,
	const std::string &oldName,
	const std::string &newName)
{


	if (type == Gmat::SPACECRAFT || type == Gmat::GROUND_STATION ||
		type == Gmat::CALCULATED_POINT)
	{
		// for spacecraft name
		for (int i = 0; i < mAllSpCount; i++)
		{

			if (mAllSpNameArray[i] == oldName)
				mAllSpNameArray[i] = newName;
		}

		//----------------------------------------------------
		// Since spacecraft name is used as key for showing
		// and drawing object map, I can't change the key name,
		// so it is removed and inserted with new name.
		//----------------------------------------------------

		std::map<std::string, bool>::iterator drawOrbitPos, showObjectPos;
		drawOrbitPos = mDrawOrbitMap.find(oldName);
		showObjectPos = mShowObjectMap.find(oldName);
		if (drawOrbitPos != mDrawOrbitMap.end() &&
			showObjectPos != mShowObjectMap.end())
		{

			mDrawOrbitMap[newName] = mDrawOrbitMap[oldName];
			mShowObjectMap[newName] = mShowObjectMap[oldName];
			mDrawOrbitMap.erase(drawOrbitPos);
			mShowObjectMap.erase(showObjectPos);


		}

		//----------------------------------------------------
		// Since object name is used as key for object default
		// color map, I can't change the key name, so it is
		// removed and inserted with new name
		//----------------------------------------------------

		std::map<std::string, UnsignedInt>::iterator defOrbColorPos, defTargColorPos;
		defOrbColorPos = mDefaultOrbitColorMap.find(oldName);

		if (defOrbColorPos != mDefaultOrbitColorMap.end())
		{

			// add new spacecraft name key and delete old
			mDefaultOrbitColorMap[newName] = mDefaultOrbitColorMap[oldName];
			mDefaultOrbitColorMap.erase(defOrbColorPos);


		}
	}
	else if (type == Gmat::COORDINATE_SYSTEM)
	{
		if (mViewCoordSysName == oldName)
			mViewCoordSysName = newName;
	}


	return true;
}


//---------------------------------------------------------------
// Methods inherited from Subscriber
//---------------------------------------------------------------

//------------------------------------------------------------------------------
// void Activate(bool state)
//------------------------------------------------------------------------------
/**
 * Turns on or off the plot.
 * This methods is called from the Toggle command.
 * Toggle command implementation
 */
 //------------------------------------------------------------------------------
bool VRInterface::Activate(bool state)
{
	return Subscriber::Activate(state);
}


//------------------------------------------------------------------------------
// bool Distribute(const Real *dat, Integer len)
//------------------------------------------------------------------------------
/**
 * Process trajectory data distributed by the GMAT Publisher
 *
 * @param <dat> An array of data to receive
 * @param <len> The number of data in the array
 */
 //------------------------------------------------------------------------------
bool VRInterface::Distribute(const Real *dat, Integer len)
{
	#ifdef DEBUG_DISTRIBUTE
		MessageInterface::ShowMessage
		("===========================================================================\n"
			"VRInterface::Distribute() instanceName=%s, active=%d, isEndOfRun=%d, "
			"isEndOfReceive=%d\n   mAllSpCount=%d, mScCount=%d, len=%d, runstate=%d, "
			"isDataStateChanged=%d\n", instanceName.c_str(), active, isEndOfRun, isEndOfReceive,
			mAllSpCount, mScCount, len, runstate, isDataStateChanged);
	#endif


	if (isEndOfRun) {	// this is called twice at end of run
			DataManager wtj;
			if (wtj.WriteToJson(jsonFileName, jstream, mScCount, mCbCount,
						mScNameArray, mCbNameArray, mSpRadii,
						mDefaultOrbitColorMap, mMaxData,
						mExportAttitude, mExportColours, mDrawOrbitArray))
				MessageInterface::ShowMessage("VRInterface: Mission data exported successfully.\n");
			if (isAbsentData) {
				MessageInterface::PopupMessage(Gmat::WARNING_, "There was absent data. Did you propagate all SC?");
				isAbsentData = false;
			}
		return true;
	}

	if (isEndOfReceive) {
		// end of a trajectory segment
			if ((mSolverIterOption == SI_CURRENT) &&
			(runstate == Gmat::SOLVING || runstate == Gmat::SOLVEDPASS)) {
			// if solver passed and still iterating
			// PLACEHOLDER if plotting intermediate trajectories. e.g.
				// update data in solver buffer with current solver data
				// plot new trajectory segment to represent iteration steps 

			// currently, this won't trigger as mSolverIterOption always SI_NONE
			return true;
		}
	}

	if (len <= 0)	//If there is no data in buffer?
		return true;

	#ifdef DEBUG_DISTRIBUTE
		MessageInterface::ShowMessage("%s, len=%d\n", GetName().c_str(), len);
		for (int i = 0; i < len; i++)
			MessageInterface::ShowMessage("%.11f  ", dat[i]);
		MessageInterface::ShowMessage("\n");
	#endif

	if (!active) { // insert other conditions for inactivity here

		return true;
	}

	if ((mSolverIterOption == SI_NONE) && (runstate == Gmat::SOLVING)) {
		// no solver iterations and currently solving
	#ifdef DEBUG
			MessageInterface::ShowMessage
			("   Just returning: SolverIterations is %d and runstate is %d\n",
				mSolverIterOption, runstate);
	#endif
		return true;
	}

	// conditions passed, start buffering data

	if (len >= 7) {	// more than 7 if multiple sc
		// there is mission data in the stream, do main implementation here
		
		DataControl(dat, len);
		return true;
	}

	// what if 0<len<7 -- test this -- shouldn't happen!


	return true; // Distribute must always return true
					 // so that ReceiveData() is called.
}


//------------------------------------------------------------------------------
// Methods for parameters
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// std::string GetParameterText(const Integer id) const
//------------------------------------------------------------------------------
std::string VRInterface::GetParameterText(const Integer id) const
{
	if (id >= SubscriberParamCount && id < VRInterfaceParamCount)
		return PARAMETER_TEXT[id - SubscriberParamCount];
	else
		return Subscriber::GetParameterText(id);

}

//------------------------------------------------------------------------------
// Integer GetParameterID(const std::string &str) const
//------------------------------------------------------------------------------
Integer VRInterface::GetParameterID(const std::string &str) const
{
	if (str == "OrbitColor" || str == "TargetColor")
		return Gmat::PARAMETER_REMOVED;

	for (int i = SubscriberParamCount; i < VRInterfaceParamCount; i++)
	{
		if (str == PARAMETER_TEXT[i - SubscriberParamCount])
			return i;
	}

	return Subscriber::GetParameterID(str);
}

//------------------------------------------------------------------------------
// Gmat::ParameterType GetParameterType(const Integer id) const
//------------------------------------------------------------------------------
Gmat::ParameterType VRInterface::GetParameterType(const Integer id) const
{
	if (id >= SubscriberParamCount && id < VRInterfaceParamCount)
		return PARAMETER_TYPE[id - SubscriberParamCount];
	else
		return Subscriber::GetParameterType(id);
}

//------------------------------------------------------------------------------
// std::string GetParameterTypeString(const Integer id) const
//------------------------------------------------------------------------------
std::string VRInterface::GetParameterTypeString(const Integer id) const
{
	return GmatBase::PARAM_TYPE_STRING[GetParameterType(id)];
}

//---------------------------------------------------------------------------
//  bool  (const Integer id) const
//---------------------------------------------------------------------------
/*
 * Identify read-only parameters
 * Parameters listed here do not show up in the GUI
*/
bool VRInterface::IsParameterReadOnly(const Integer id) const
{
	switch (id) {
		case SOLVER_ITERATIONS:
			return true;
		//case COORD_SYSTEM:
		//	return true;
	//	case READ_ONLY_PARAMETER
	//		return true;
		default:
			return Subscriber::IsParameterReadOnly(id);
	}
}

//------------------------------------------------------------------------------
// virtual Integer GetIntegerParameter(const Integer id) const
//------------------------------------------------------------------------------
Integer VRInterface::GetIntegerParameter(const Integer id) const
{
	switch (id) {
		case DATA_COLLECT_FREQUENCY:
			return mDataCollectFrequency;
		case MAX_DATA:
			return mMaxData;
		case SC_RADII:
			return mScRadiiMin;
		default:
			return Subscriber::GetIntegerParameter(id);
	}
}


//------------------------------------------------------------------------------
// virtual Integer GetIntegerParameter(const std::string &label) const
//------------------------------------------------------------------------------
Integer VRInterface::GetIntegerParameter(const std::string &label) const
{
	return GetIntegerParameter(GetParameterID(label));
}


//------------------------------------------------------------------------------
// virtual Integer SetIntegerParameter(const Integer id, const Integer value)
//------------------------------------------------------------------------------
Integer VRInterface::SetIntegerParameter(const Integer id, const Integer value)
{
	switch (id) {
		case DATA_COLLECT_FREQUENCY:
			if (value > 0)
			{
				mDataCollectFrequency = value;
				return value;
			}
			else
			{
				SubscriberException se;
				se.SetDetails(errorMessageFormat.c_str(),
					GmatStringUtil::ToString(value, 1).c_str(),
					"DataCollectFrequency", "Integer Number > 0");
				throw se;
			}
		case MAX_DATA:
			if (value > 0)
			{
				mMaxData = value;
				return value;
			}
			else
			{
				SubscriberException se;
				se.SetDetails(errorMessageFormat.c_str(),
					GmatStringUtil::ToString(value, 1).c_str(),
					"MaxPlotPoints", "Integer Number > 0");
				throw se;
			}
		case SC_RADII:
			if (value > 0) {
				mScRadiiMin = value;
			}			
			else {
				SubscriberException se;
				se.SetDetails(errorMessageFormat.c_str(),
					GmatStringUtil::ToString(value, 1).c_str(),
					"SpacecraftRadii", "Integer Number > 0");
				throw se;
			}
			return value;
		default:
			return Subscriber::SetIntegerParameter(id, value);
	}
}


//------------------------------------------------------------------------------
// virtual Integer SetIntegerParameter(const std::string &label,
//                                     const Integer value)
//------------------------------------------------------------------------------
Integer VRInterface::SetIntegerParameter(const std::string &label,
	const Integer value)
{
	return SetIntegerParameter(GetParameterID(label), value);
}


//------------------------------------------------------------------------------
// std::string GetStringParameter(const Integer id) const
//------------------------------------------------------------------------------
std::string VRInterface::GetStringParameter(const Integer id) const
{


	switch (id) {
		case SOLVER_ITERATIONS:	// this is called for unknown reasons -- DOCUMENT
			return "None";	// default to None
		case ADD:
			return GetObjectStringList();
		case COORD_SYSTEM:
			return mViewCoordSysName;
		case JSON_FILE:
			return jsonFileName;
		default:
			return Subscriber::GetStringParameter(id);
	}
}


//------------------------------------------------------------------------------
// std::string GetStringParameter(const std::string &label) const
//------------------------------------------------------------------------------
std::string VRInterface::GetStringParameter(const std::string &label) const
{


	return GetStringParameter(GetParameterID(label));
}


//------------------------------------------------------------------------------
// bool SetStringParameter(const Integer id, const std::string &value)
//------------------------------------------------------------------------------
//Sets the string value of the parameter with the specied ID.
bool VRInterface::SetStringParameter(const Integer id, const std::string &value)
{
	switch (id) {
	case COORD_SYSTEM:
		mViewCoordSysName = value;
		return true;
	case ADD:
	{
		if (value[0] == '{')
		{
			try
			{
				TextParser tp;
				ClearSpacePointList();
				StringArray spList = tp.SeparateBrackets(value, "{}", ",");
				for (UnsignedInt i = 0; i < spList.size(); i++)
					AddSpacePoint(spList[i], mAllSpCount);
				return true;
			}
			catch (BaseException &)
			{
				SubscriberException se;
				se.SetDetails(errorMessageFormat.c_str(), value.c_str(),
					"Add", "Valid CelestialBody list");
				throw se;
			}
		}
		else
		{
			return AddSpacePoint(value, mAllSpCount);
		}
	}
	case JSON_FILE: {

		// do validation here
		if (!GmatFileUtil::IsValidFileName(value))
		{
			std::string msg = GmatFileUtil::GetInvalidFileNameMessage(1);
			SubscriberException se;
			se.SetDetails(errorMessageFormat.c_str(), value.c_str(), "Filename", msg.c_str());
			throw se;
		}
		if (value == "") {	// if emptty, create new 
			jsonFileName = fileNameFromScript(".json");
		}
		else 
			jsonFileName = value;
		return true;
	}
	default:
		return Subscriber::SetStringParameter(id, value);
	}
}


//------------------------------------------------------------------------------
// bool SetStringParameter(const std::string &label, const std::string &value)
//------------------------------------------------------------------------------
bool VRInterface::SetStringParameter(const std::string &label,
	const std::string &value)
{
	return SetStringParameter(GetParameterID(label), value);
}


//------------------------------------------------------------------------------
// virtual bool SetStringParameter(const Integer id, const std::string &value,
//                                 const Integer index)
//------------------------------------------------------------------------------
/* Sets a string in a vector of strings, where the vector has 
 * the specified ID and the input string is placed in the vector 
 * element identified by index.
*/
bool VRInterface::SetStringParameter(const Integer id, const std::string &value,
	const Integer index)
{
	switch (id)
	{
	case COORD_SYSTEM:
		mViewCoordSysName = value;
		return true;
	case ADD:
	{
		//if (value[0] == '{')
		//{
		//	try
		//	{
		//		TextParser tp;
		//		ClearSpacePointList();
		//		StringArray spList = tp.SeparateBrackets(value, "{}", ",");
		//		for (UnsignedInt i = 0; i < spList.size(); i++)
		//			AddSpacePoint(spList[i], mAllSpCount);
		//		return true;
		//	}
		//	catch (BaseException &)
		//	{
		//		SubscriberException se;
		//		se.SetDetails(errorMessageFormat.c_str(), value.c_str(),
		//			"Add", "Valid CelestialBody list");
		//		throw se;
		//	}
		//}
		//else
		return AddSpacePoint(value, mAllSpCount);
	}
	//case ORBIT_COLOR:
	//case TARGET_COLOR:
		//if (value[0] == '[')
		//	PutUnsignedIntValue(id, value);
		//return true;
	default:
		return Subscriber::SetStringParameter(id, value);
	}
}


//------------------------------------------------------------------------------
// virtual bool SetStringParameter(const std::string &label,
//                                 const std::string &value,
//                                 const Integer index)
//------------------------------------------------------------------------------
bool VRInterface::SetStringParameter(const std::string &label,
	const std::string &value,
	const Integer index)
{


	return SetStringParameter(GetParameterID(label), value, index);
}



//------------------------------------------------------------------------------
// const StringArray& GetStringArrayParameter(const Integer id) const
//------------------------------------------------------------------------------
const StringArray& VRInterface::GetStringArrayParameter(const Integer id) const
{
	switch (id)
	{
	case ADD:
		return mAllSpNameArray;
	default:
		return Subscriber::GetStringArrayParameter(id);
	}
}


//------------------------------------------------------------------------------
// const StringArray& GetStringArrayParameter(const std::string &label) const
//------------------------------------------------------------------------------
const StringArray& VRInterface::GetStringArrayParameter(const std::string &label) const
{
	return GetStringArrayParameter(GetParameterID(label));
}


//------------------------------------------------------------------------------
// bool GetBooleanParameter(const Integer id) const
//------------------------------------------------------------------------------
bool VRInterface::GetBooleanParameter(const Integer id) const
{
	switch (id) {
		case EXPORT_ATTITUDE:
			return mExportAttitude;
		case EXPORT_TRAJECTORY_COLORS:
			return mExportColours;
		case DERIVE_RADII:
			return mDeriveRadii;
		default:
			return Subscriber::GetBooleanParameter(id);
	}
	//if (id == SHOW_PLOT)
	//	return active;
	
}


//------------------------------------------------------------------------------
// bool GetBooleanParameter(const std::string &label) const
//------------------------------------------------------------------------------
bool VRInterface::GetBooleanParameter(const std::string &label) const
{
	return GetBooleanParameter(GetParameterID(label));
}


//------------------------------------------------------------------------------
// bool SetBooleanParameter(const Integer id, const bool value)
//------------------------------------------------------------------------------
bool VRInterface::SetBooleanParameter(const Integer id, const bool value)
{
	switch (id) {
		case EXPORT_ATTITUDE:
			mExportAttitude = value;
			return mExportAttitude;
		case EXPORT_TRAJECTORY_COLORS:
			mExportColours = value;
			return mExportColours;
		case DERIVE_RADII:
			mDeriveRadii = value;
			return mDeriveRadii;
		default:
			return Subscriber::SetBooleanParameter(id, value);
	
	}
}


//------------------------------------------------------------------------------
// bool SetBooleanParameter(const std::string &label, const bool value)
//------------------------------------------------------------------------------
bool VRInterface::SetBooleanParameter(const std::string &label, const bool value)
{
	return SetBooleanParameter(GetParameterID(label), value);
}


//---------------------------------------------------------------------------
//const BooleanArray& GetBooleanArrayParameter(const Integer id) const
//---------------------------------------------------------------------------
/**
 * @see GmatBase
 */
 //------------------------------------------------------------------------------
const BooleanArray& VRInterface::GetBooleanArrayParameter(const Integer id) const
{
	switch (id) {
	//case DRAW_OBJECT:
	//{
	//	return mDrawObjectArray;
	//}
	//case DRAW_LABEL:
	//{
	//	return mDrawLabelArray;
	//}
	default:
		return Subscriber::GetBooleanArrayParameter(id);
	}
}


//---------------------------------------------------------------------------
//const BooleanArray& GetBooleanArrayParameter(const std::string &label) const
//---------------------------------------------------------------------------
/**
 * @see GmatBase
 */
 //------------------------------------------------------------------------------
const BooleanArray& VRInterface::GetBooleanArrayParameter(const std::string &label) const
{
	Integer id = GetParameterID(label);
	return GetBooleanArrayParameter(id);
}


//---------------------------------------------------------------------------
//  bool SetBooleanArrayParameter(const Integer id, const BooleanArray &valueArray)
//---------------------------------------------------------------------------
/**
 * @see GmatBase
 */
 //------------------------------------------------------------------------------
bool VRInterface::SetBooleanArrayParameter(const Integer id,
	const BooleanArray &valueArray)
{


	//if (id == DRAW_OBJECT)
	//{


	//	// Check size of arrays in Initialize() or Interpreter::FinalPass()?
	//	//if (mAllSpNameArray.size() != valueArray.size())
	//	//   throw SubscriberException
	//	//      ("The count doesn't match with added objects" + GetParameterText(id));

	//	mDrawObjectArray = valueArray;
	//	Integer minCount = mAllSpNameArray.size() < mDrawObjectArray.size() ?
	//		mAllSpNameArray.size() : mDrawObjectArray.size();

	//	// GUI uses mShowObjectMap so update it
	//	for (Integer i = 0; i < minCount; i++)
	//	{
	//		bool tf = mDrawObjectArray[i];
	//		mShowObjectMap[mAllSpNameArray[i]] = tf;
	//	}
	//	return true;
	//}
	return Subscriber::SetBooleanArrayParameter(id, valueArray);
}


//---------------------------------------------------------------------------
//  bool SetBooleanArrayParameter(const std::string &label,
//                                const BooleanArray &valueArray)
//---------------------------------------------------------------------------
/**
 * @see GmatBase
 */
 //------------------------------------------------------------------------------
bool VRInterface::SetBooleanArrayParameter(const std::string &label,
	const BooleanArray &valueArray)
{
	Integer id = GetParameterID(label);
	return SetBooleanArrayParameter(id, valueArray);
}


//------------------------------------------------------------------------------
// virtual std::string GetRefObjectName(const UnsignedInt type) const
//------------------------------------------------------------------------------
std::string VRInterface::GetRefObjectName(const UnsignedInt type) const
{
	if (type == Gmat::COORDINATE_SYSTEM)
	{
		return mViewCoordSysName; //just return this
	}

	return Subscriber::GetRefObjectName(type);
}


//------------------------------------------------------------------------------
// virtual bool HasRefObjectTypeArray()
//------------------------------------------------------------------------------
/**
 * @see GmatBase
 */
 //------------------------------------------------------------------------------
bool VRInterface::HasRefObjectTypeArray()
{
	return true;
}


//------------------------------------------------------------------------------
// const ObjectTypeArray& GetRefObjectTypeArray()
//------------------------------------------------------------------------------
/**
 * Retrieves the list of ref object types used by this class.
 *
 * @return the list of object types.
 *
 */
 //------------------------------------------------------------------------------
const ObjectTypeArray& VRInterface::GetRefObjectTypeArray()
{
	refObjectTypes.push_back(Gmat::SPACE_POINT);
	return refObjectTypes;
}


//------------------------------------------------------------------------------
// virtual const StringArray& GetRefObjectNameArray(const UnsignedInt type)
//------------------------------------------------------------------------------
const StringArray& VRInterface::GetRefObjectNameArray(const UnsignedInt type)
{

	if (type == Gmat::COORDINATE_SYSTEM || type == Gmat::UNKNOWN_OBJECT)
	{
		refObjectNames.push_back(mViewCoordSysName);
	}

	if (type == Gmat::SPACE_POINT || type == Gmat::UNKNOWN_OBJECT)
	{
		refObjectNames.insert(refObjectNames.end(), mAllSpNameArray.begin(),
			mAllSpNameArray.end());
	}


	return refObjectNames;
}


//------------------------------------------------------------------------------
// virtual GmatBase* GetRefObject(const UnsignedInt type,
//                                const std::string &name)
//------------------------------------------------------------------------------
GmatBase* VRInterface::GetRefObject(const UnsignedInt type,
	const std::string &name)
{
	if (type == Gmat::COORDINATE_SYSTEM)
	{
		if (name == mViewCoordSysName)
			return mViewCoordSystem;
	}

	return Subscriber::GetRefObject(type, name);
}


//------------------------------------------------------------------------------
// virtual bool SetRefObject(GmatBase *obj, const UnsignedInt type,
//                           const std::string &name = "")
//------------------------------------------------------------------------------
/**
 * Set reference object pointer.
 *
 * @param <obj>  Reference object pointer to set to given object type and name
 * @param <type> Reference object type
 * @param <name> Reference object name
 */
 //------------------------------------------------------------------------------
bool VRInterface::SetRefObject(GmatBase *obj, const UnsignedInt type,
	const std::string &name)
{


	if (obj == NULL)
	{

		return false;
	}

	std::string realName = name;
	if (name == "")
		realName = obj->GetName();

	// if (obj->IsOfType(Gmat::SPACE_POINT))
	if ((obj->IsOfType(Gmat::SPACECRAFT)) || (obj->IsOfType(Gmat::CELESTIAL_BODY))) {

		for (Integer i = 0; i < mAllSpCount; i++) {

			if (mAllSpNameArray[i] == realName) {
				mAllSpArray[i] = (SpacePoint*)(obj);
			}
		}

		// If spacecraft, save initial epoch so that data will not be buffered before
		// the initial epoch
		if (obj->IsOfType(Gmat::SPACECRAFT)) {
			SpaceObject *so = (SpaceObject*)(obj);

			mScInitialEpochMap[so->GetName()] = so->GetEpoch();
		}
		return true;
	}

	else if (type == Gmat::COORDINATE_SYSTEM)
	{
		if (realName == mViewCoordSysName)
			mViewCoordSystem = (CoordinateSystem*)obj;

		return true;
	}

	else {
		MessageInterface::PopupMessage(Gmat::WARNING_, "Unsupported SpacePoint(s) selected. These will be ignored at run.");
		return true;
	}

	return Subscriber::SetRefObject(obj, type, realName);
}

//------------------------------------------------------------------------------
// virtual void SetOrbitColorChanged(GmatBase *originator, const std::string &newColor, ...)
//------------------------------------------------------------------------------
/**
 * Sets object orbit color change.
 *
 * @param originator  The assignment command pointer who is setting
 * @param newColor  New color to be applied to the object
 * @param objName  Name of the object
 * @param desc  Description of property change
 * @param isSpacecraft Set to true if object is a Spcecraft
 */
 //------------------------------------------------------------------------------
void VRInterface::SetOrbitColorChanged(GmatBase *originator,
	const std::string &newColor,
	const std::string &objName,
	const std::string &desc)
{


	UnsignedInt intColor = RgbColor::ToIntColor(newColor);
	mDefaultOrbitColorMap[objName] = intColor;


}

//---------------------------------------------------------------------------
// UnsignedInt GetPropertyObjectType(const Integer id) const
//---------------------------------------------------------------------------
UnsignedInt VRInterface::GetPropertyObjectType(const Integer id) const
{
	if (id == ADD)
		return Gmat::SPACE_POINT;
	if (id== COORD_SYSTEM)
		return Gmat::COORDINATE_SYSTEM;

	return Subscriber::GetPropertyObjectType(id);
}


//------------------------------------------------------------------------------
// bool GetShowObject(const std::string &name)
//------------------------------------------------------------------------------
bool VRInterface::GetShowObject(const std::string &name)
{


	return mShowObjectMap[name];
}


//------------------------------------------------------------------------------
// void SetShowObject(const std::string &name, bool value)
//------------------------------------------------------------------------------
void VRInterface::SetShowObject(const std::string &name, bool value)
{


	mShowObjectMap[name] = value;
	if (value)
		mDrawOrbitMap[name] = value;

	if (mShowObjectMap.find(name) != mShowObjectMap.end())
	{
		for (int i = 0; i < mAllSpCount; i++)
			if (mAllSpNameArray[i] == name)
				mDrawObjectArray[i] = value;
	}
}



//------------------------------------------------------------------------------
// Internal Methods
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// const StringArray& GetSpacePointList()
//------------------------------------------------------------------------------
const StringArray& VRInterface::GetSpacePointList()
{

	return mAllSpNameArray;
}

//------------------------------------------------------------------------------
// const StringArray& GetNonSpacecraftList()
//------------------------------------------------------------------------------
const StringArray& VRInterface::GetNonSpacecraftList()
{
	return mObjectNameArray;
}


//------------------------------------------------------------------------------
// Protected Methods
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// bool DataControl(const Real *dat, Integer len)
//------------------------------------------------------------------------------
bool VRInterface::DataControl(const Real *dat, Integer len)
{

	// Skip data if data publishing command such as Propagate is inside a function
	// and this VRInterface is not a global nor a local object (i.e declared in the main script)
	// (LOJ: 2015.08.17)
	if (currentProvider && currentProvider->TakeAction("IsInFunction"))
	{
		bool skipData = false;
		// Check for spacepoints if data should be skipped or not
		for (int i = 0; i < mAllSpCount; i++)
		{
			SpacePoint *sp = mAllSpArray[i];

			if (sp)
			{

				// Skip data if VRInterface is global and spacepoint is local
				if (IsGlobal() && sp->IsLocal())
				{

					skipData = true;
					break;
				}
				// Skip data if spacepoint is not a global nor a local object
				else if (!(sp->IsGlobal()) && !(sp->IsLocal()))
				{
					skipData = true;
					break;
				}
			}
		}

		if (skipData)
		{
			return true;
		}
	}

	mNumData++;

	// Buffer data if data collect frequency is met or first data
	if ((mNumData % mDataCollectFrequency) == 0 || (mNumData == 1))
	{
		bool status = (BufferSpacecraftData(dat, len) && 
							BufferCelestialBodyData(dat, len));



		// if solving and plotting current iteration just return
		if (status == false)
			return true;

		bool solving = false;
		if (runstate == Gmat::SOLVING)
			solving = true;

		bool inFunction = false;
		if (currentProvider && currentProvider->TakeAction("IsInFunction"))
			inFunction = true;

		// publish final solution data to plotter/data manager
		DataManager atb;
		atb.AddToBuffer(dat[0],
			mScCount, mCbCount, mScNameArray, mCbNameArray,
			mScXArray, mScYArray, mScZArray,
			mScVxArray, mScVyArray, mScVzArray,
			mScQArray,
			mCbXArray, mCbYArray, mCbZArray,
			mCbVxArray, mCbVyArray, mCbVzArray,
			mCbQArray,
			solving, mSolverIterOption, isDataOn, mMaxData, inFunction);

	}


	return true;
}


//------------------------------------------------------------------------------
// Integer BufferOrbitData(const Real *dat, Integer len)
//------------------------------------------------------------------------------
/**
 * @return true if continue to updating plot
 *         false if solving and plotting current iteration
 */
 //------------------------------------------------------------------------------
bool VRInterface::BufferSpacecraftData(const Real * dat, Integer len)
{
	// @note
	// New Publisher code doesn't assign currentProvider anymore,
	// it just copies current labels. There was an issue with
	// provider id keep incrementing if data is regisgered and
	// published inside a GmatFunction

	StringArray dataLabels = theDataLabels[0];

	// method only applies to spacecraft 

	Integer idX, idY, idZ;
	Integer idVx, idVy, idVz;
	Integer scIndex = -1;

	for (Integer i = 0; i < mScCount; i++)
		// iterates for all spacecraft
	{
		idX = FindIndexOfElement(dataLabels, mScNameArray[i] + ".X");
		idY = FindIndexOfElement(dataLabels, mScNameArray[i] + ".Y");
		idZ = FindIndexOfElement(dataLabels, mScNameArray[i] + ".Z");

		idVx = FindIndexOfElement(dataLabels, mScNameArray[i] + ".Vx");
		idVy = FindIndexOfElement(dataLabels, mScNameArray[i] + ".Vy");
		idVz = FindIndexOfElement(dataLabels, mScNameArray[i] + ".Vz");

		//append quat to this

		scIndex++;	// potentially unecessary

		Rvector quat;
		// access Spacecraft member of mObjectArray reference
		// this assumes that first 'half' of this array
		Spacecraft *sc = (Spacecraft*)mObjectArray[i];	
		// If any of index not found, handle absent data and continue with the next spacecraft
		if (idX == -1 || idY == -1 || idZ == -1 ||
			idVx == -1 || idVy == -1 || idVz == -1)
		{
			isAbsentData = true;
			mScPrevDataPresent[scIndex] = false;
			continue;
		}

		// If distributed data coordinate system is different from view
		// coordinate system, convert data here.

		// If we convert after current epoch, it will not give correct
		// results, if origin is spacecraft,
		// ie, sat->GetMJ2000State(epoch) will not give correct results.
		if ((theDataCoordSystem != NULL && mViewCoordSystem != NULL) &&
			(mViewCoordSystem != theDataCoordSystem))
		{

			CoordinateConverter coordConverter;
			Rvector6 inState, outState;

			// convert position and velocity
			inState.Set(dat[idX], dat[idY], dat[idZ],
				dat[idVx], dat[idVy], dat[idVz]);

			coordConverter.Convert(dat[0], inState, theDataCoordSystem,
				outState, mViewCoordSystem);

			mScXArray[scIndex] = outState[0];
			mScYArray[scIndex] = outState[1];
			mScZArray[scIndex] = outState[2];
			mScVxArray[scIndex] = outState[3];
			mScVyArray[scIndex] = outState[4];
			mScVzArray[scIndex] = outState[5];

			// convert attitude
			// omit sc->HasAttitude() due to misunderstood behaviour
			if (sc->HasAttitude()) {
				Rmatrix33 cosMat = sc->GetAttitude(dat[0]);
				Rmatrix33 rotMat = coordConverter.GetLastRotationMatrix();
				Rmatrix33 finalMat = cosMat * rotMat.Transpose();
				quat = AttitudeConversionUtility::ToQuaternion(finalMat);
			}
			else 
				// default state, Gmat gives this anyways if HasAttitude() not used.
				quat = Rvector(4, 0.0, 0.0, 0.0, 1.0);	
		}

		else {
		// if coord system is MJ200Eq, no conversion needed
			mScXArray[scIndex] = dat[idX];
			mScYArray[scIndex] = dat[idY];
			mScZArray[scIndex] = dat[idZ];
			mScVxArray[scIndex] = dat[idVx];
			mScVyArray[scIndex] = dat[idVy];
			mScVzArray[scIndex] = dat[idVz];
			
			if (sc->HasAttitude()) {
				Rmatrix33 cosMat = sc->GetAttitude(dat[0]);
				Rvector quat = AttitudeConversionUtility::ToQuaternion(cosMat);
			}
			else
				quat = Rvector(4, 0.0, 0.0, 0.0, 1.0);
		}

		// perform attitude assignment here
		mScQArray[Q1][i] = quat[Q1];
		mScQArray[Q2][i] = quat[Q2];
		mScQArray[Q3][i] = quat[Q3];
		mScQArray[Q4][i] = quat[Q4];

		mScPrevDataPresent[scIndex] = true;	// consider removing obselete code

	}

	// solver data control for future use -- consider removing
	if (mSolverIterOption == SI_CURRENT)
	{
		// save data when targeting or last iteration
		if (runstate == Gmat::SOLVING || runstate == Gmat::SOLVEDPASS)
		{
			return true; // careful having this here
		}

		if (runstate == Gmat::SOLVING)
		{
			return false;	// solving 
		}
	}

	return true;
}

//------------------------------------------------------------------------------
// bool BufferCelestialBodyData(const Real *dat, Integer len)
//------------------------------------------------------------------------------
/**
 * @return 1 if continue to updating plot
 *         2 if solving and plotting current iteration
 */
 //------------------------------------------------------------------------------
bool VRInterface::BufferCelestialBodyData(const Real *dat, Integer len)
{
	Integer cbIndex = -1;

	for (int i = 0; i < mCbCount; i++)	//iterate through all celestial bodies
	{
		cbIndex++;

		SpacePoint *cb = mCbArray[i];	// can also try with mObjectArray
		//SpacePoint *cb = mObjectArray[i + mScCount];
		Rvector quat;
		Rvector6 cbMjEqState;

		// attempting error handling here
		try
		{
			cbMjEqState = cb->GetMJ2000State(dat[0]);	// assuming dat[0] is time
		}
		catch (BaseException &)
		{
			SubscriberException se;
			se.SetDetails(errorMessageFormat.c_str(),
				"Error getting Cb state");
			throw se;
		}

		// If distributed data coordinate system is different from view
		// coordinate system, convert data here.
		// If we convert after current epoch, it will not give correct
		// results, if origin is spacecraft,
		// ie, cb->GetMJ2000State(epoch) will not give correct results.

		if ((theDataCoordSystem != NULL && mViewCoordSystem != NULL) &&
			(mViewCoordSystem != theDataCoordSystem)) {
			CoordinateConverter coordConverter;
			Rvector6 outState;

			coordConverter.Convert(dat[0], cbMjEqState, theDataCoordSystem,
				outState, mViewCoordSystem);

			mCbXArray[cbIndex] = outState[0];
			mCbYArray[cbIndex] = outState[1];
			mCbZArray[cbIndex] = outState[2];
			mCbVxArray[cbIndex] = outState[3];	// are we even sure this is V??
			mCbVyArray[cbIndex] = outState[4];
			mCbVzArray[cbIndex] = outState[5];

			if (cb->HasAttitude()) {
				Rmatrix33 cosMat = cb->GetAttitude(dat[0]);
				Rmatrix33 rotMat = coordConverter.GetLastRotationMatrix();
				Rmatrix33 finalMat = cosMat * rotMat.Transpose();
				quat = AttitudeConversionUtility::ToQuaternion(finalMat);
			}
			else
				quat = Rvector(4, 0.0, 0.0, 0.0, 1.0);
		}

		else {
			mCbXArray[cbIndex] = cbMjEqState[0];
			mCbYArray[cbIndex] = cbMjEqState[1];
			mCbZArray[cbIndex] = cbMjEqState[2];
			mCbVxArray[cbIndex] = cbMjEqState[3];	// are we even sure this is V??
			mCbVyArray[cbIndex] = cbMjEqState[4];
			mCbVzArray[cbIndex] = cbMjEqState[5];

			if (cb->HasAttitude()) {
				Rmatrix33 cosMat = cb->GetAttitude(dat[0]);
				quat = AttitudeConversionUtility::ToQuaternion(cosMat);
			}
			else
				quat = Rvector(4, 0.0, 0.0, 0.0, 1.0);
		}

		mCbQArray[Q1][i] = quat[Q1];
		mCbQArray[Q2][i] = quat[Q2];
		mCbQArray[Q3][i] = quat[Q3];
		mCbQArray[Q4][i] = quat[Q4];

		mCbPrevDataPresent[cbIndex] = true;
	}

	// skipped out view coordinate system stuff here 
			
	// if has attitude, update attitude here

	// skipped out Current trajecotry implementation

	// this data control potentially doesn't have to happen as the same check is performed in BufferOrbitData

	if (mSolverIterOption == SI_CURRENT) {
		// save data when targeting or last iteration
		if (runstate == Gmat::SOLVING || runstate == Gmat::SOLVEDPASS) {
			return true; // careful having this here
		}
		if (runstate == Gmat::SOLVING) {
			return false;	// solving 
		}
	}

	return true;
}



//------------------------------------------------------------------------------
// virtual std::string GetObjectStringList() const
//------------------------------------------------------------------------------
/**
 * Formats object names into {}. This includes all SpacePoint objects to list.
 * If subclass should not include all objects, then this method should be
 * overridden in the subclass.
 * This protected method is inherited from the Subscriber
 */
 //------------------------------------------------------------------------------
std::string VRInterface::GetObjectStringList() const
{
	MessageInterface::ShowMessage("VRInterface::GetObjectStringList() entered\n");

	Integer objCount = mAllSpNameArray.size();
	std::string objList = "{ ";
	for (Integer i = 0; i < objCount; i++)
	{
		if (i == objCount - 1)
			objList += mAllSpNameArray[i];
		else
			objList += mAllSpNameArray[i] + ", ";
	}
	objList += " }";
	return objList;
}


//------------------------------------------------------------------------------
// bool AddSpacePoint(const std::string &name, Integer index, bool show = true)
//------------------------------------------------------------------------------
// Adds Spacecraft and other objects to object arrays
bool VRInterface::AddSpacePoint(const std::string &name, Integer index, bool show)
{


	// if name not in the list, add
	if (find(mAllSpNameArray.begin(), mAllSpNameArray.end(), name) ==
		mAllSpNameArray.end())
	{
		// Do we want to add any new object here? Like Sun in the following.
		// VRInterface.Add = {DefaultSC, Earth};
		// VRInterface.Add = {Sun};
		// If yes, we should not check for index. Just commenting out for now (LOJ: 2011.01.14)
		//if (name != "" && index == mAllSpCount)
		if (name != "")
		{
			mAllSpNameArray.push_back(name);
			mAllSpArray.push_back(NULL);
			mAllSpCount = mAllSpNameArray.size();

			mDrawOrbitMap[name] = show;
			mShowObjectMap[name] = show;

			// Initially set to white
			mDefaultOrbitColorMap[name] = GmatColor::WHITE;

			// Ignore array value more than actual map size
			if (mDrawObjectArray.size() < mShowObjectMap.size())
				mDrawObjectArray.push_back(true); //added (LOJ: 2011.01.13 for bug 2215 fix)
		}
	}



	return true;
}



//------------------------------------------------------------------------------
// bool ClearSpacePointList()
//------------------------------------------------------------------------------
// Clears all object arrays called from TakeAction("Clear")
bool VRInterface::ClearSpacePointList()
{
	//updateHere
	//MessageInterface::ShowMessage("VRInterface::ClearSpacePointList()\n");

	mAllSpNameArray.clear();
	mAllSpArray.clear();
	mObjectArray.clear();
	mDrawOrbitArray.clear();
	mDrawObjectArray.clear();
	mScNameArray.clear();
	mCbNameArray.clear();
	mObjectNameArray.clear();
	mCbArray.clear();

	mSpRadii.clear();

	mScXArray.clear();
	mScYArray.clear();
	mScZArray.clear();
	mScVxArray.clear();
	mScVyArray.clear();
	mScVzArray.clear();
	mScQArray.clear();
	mScPrevDataPresent.clear();

	mCbXArray.clear();
	mCbYArray.clear();
	mCbZArray.clear();
	mCbVxArray.clear();
	mCbVyArray.clear();
	mCbVzArray.clear();
	mCbQArray.clear();

	mCbPrevDataPresent.clear();

	mDefaultOrbitColorMap.clear();

	mAllSpCount = 0;
	mCbCount = 0;
	mScCount = 0;
	mObjectCount = 0;
	mDataAbsentWarningCount = 0;

	return true;
}


//------------------------------------------------------------------------------
// bool RemoveSpacePoint(const std::string &name)
//------------------------------------------------------------------------------
/*
 * Removes spacecraft from the spacecraft list
 *
 * @param <name> spacecraft name to be removed from the list
 *
 * @return true if spacecraft was removed from the list, false otherwise
 *
 */
 //------------------------------------------------------------------------------
bool VRInterface::RemoveSpacePoint(const std::string &name)
{
	//-----------------------------------------------------------------
#ifdef __REMOVE_OBJ_BY_SETTING_FLAG__
//-----------------------------------------------------------------

	for (UnsignedInt i = 0; i < mObjectNameArray.size(); i++)
	{
		if (mObjectNameArray[i] == name)
		{
			mDrawOrbitArray[i] = false;
			// PlotInterface::SetGlDrawOrbitFlag(instanceName, mDrawOrbitArray);
			return true;
		}
	}

	return false;

	//-----------------------------------------------------------------
#else
//-----------------------------------------------------------------

	bool removedFromScArray = false;
	bool removedFromAllSpArray = false;

	//-------------------------------------------------------
	// remove from mScNameArray
	//-------------------------------------------------------


	StringArray::iterator scPos =
		find(mScNameArray.begin(), mScNameArray.end(), name);

	if (scPos != mScNameArray.end())
	{
		MessageInterface::ShowMessage("sc to be erased=%s\n", (*scPos).c_str());

		// erase given spacecraft from the arrays
		mScNameArray.erase(scPos);
		mScXArray.erase(mScXArray.begin());
		mScYArray.erase(mScYArray.begin());
		mScZArray.erase(mScZArray.begin());
		mScVxArray.erase(mScVxArray.begin());
		mScVyArray.erase(mScVyArray.begin());
		mScVzArray.erase(mScVzArray.begin());
		mScPrevDataPresent.erase(mScPrevDataPresent.begin());
		mScPrevEpoch.erase(mScPrevEpoch.begin());
		mScPrevX.erase(mScPrevX.begin());
		mScPrevY.erase(mScPrevY.begin());
		mScPrevZ.erase(mScPrevZ.begin());
		mScPrevVx.erase(mScPrevVx.begin());
		mScPrevVy.erase(mScPrevVy.begin());
		mScPrevVz.erase(mScPrevVz.begin());

		mScCount = mScNameArray.size();



		removedFromScArray = true;
	}


	//-------------------------------------------------------
	// remove from mAllSpNameArray and mObjectNameArray
	//-------------------------------------------------------


	StringArray::iterator spPos =
		find(mAllSpNameArray.begin(), mAllSpNameArray.end(), name);
	StringArray::iterator objPos =
		find(mObjectNameArray.begin(), mObjectNameArray.end(), name);

	if (spPos != mAllSpNameArray.end() && objPos != mObjectNameArray.end())
	{
		mAllSpNameArray.erase(spPos);
		mObjectNameArray.erase(objPos);
		mAllSpCount = mAllSpNameArray.size();
		removedFromAllSpArray = true;



		std::map<std::string, UnsignedInt>::iterator orbColorPos, targColorPos;
		orbColorPos = mDefaultOrbitColorMap.find(name);
		targColorPos = mDefaultTargetColorMap.find(name);

		if (orbColorPos != mDefaultOrbitColorMap.end() &&
			targColorPos != mDefaultTargetColorMap.end())
		{
			// erase given spacecraft name
			mDefaultOrbitColorMap.erase(orbColorPos);
			mDefaultTargetColorMap.erase(targColorPos);
			mAllSpCount = mAllSpNameArray.size();
			removedFromAllSpArray = true;
		}
	}

	//-------------------------------------------------------
	// remove from mObjectArray
	//-------------------------------------------------------


	for (std::vector<SpacePoint*>::iterator objptPos = mObjectArray.begin();
		objptPos != mObjectArray.end(); ++objptPos)
	{
		MessageInterface::ShowMessage
		("mObjectArray=%s\n", (*objptPos)->GetName().c_str());
		if ((*objptPos)->GetName() == name)
		{
			mObjectArray.erase(objptPos);
			break;
		}
	}



	// Set all object array and pointers
	if (removedFromScArray && removedFromAllSpArray)
		PlotInterface::SetGlObject(instanceName, mObjectNameArray, mObjectArray);

	return (removedFromScArray && removedFromAllSpArray);

#endif
}


//------------------------------------------------------------------------------
// Integer FindIndexOfElement(StringArray &labelArray, const std::string &label)
//------------------------------------------------------------------------------
/*
 * Finds the index of the element label from the element label array.
 *
 * Typical element label array contains:
 *    All.epoch, scName.X, scName.Y, scName.Z, scName.Vx, scName.Vy, scName.Vz.
 */
 //------------------------------------------------------------------------------
Integer VRInterface::FindIndexOfElement(StringArray &labelArray,
	const std::string &label)
{
	std::vector<std::string>::iterator pos;
	pos = find(labelArray.begin(), labelArray.end(), label);
	if (pos == labelArray.end())
		return -1;
	else
		return distance(labelArray.begin(), pos);
}


//------------------------------------------------------------------------------
// bool FixSpacePointArray(const std::string &name, Integer index, bool show = true)
//------------------------------------------------------------------------------
// Removes any spacepoints will NULL object reference 
// runs after AddSpacePoint and before BuildDynamicBuffers
bool VRInterface::FixSpacePointArray()
{
	for (int i = 0; i < mAllSpCount; i++) {
		if (mAllSpArray[i] == nullptr) {	// or NULL
			MessageInterface::PopupMessage(Gmat::ERROR_, "SpacePoints will NULL reference found. Removing before run.");
			std::vector<SpacePoint*>::iterator allSpArrayPos = mAllSpArray.begin() + i;

			mAllSpArray.erase(allSpArrayPos);

			mDrawOrbitMap.erase(mAllSpNameArray[i]);	// check these
			mShowObjectMap.erase(mAllSpNameArray[i]);
			mDefaultOrbitColorMap.erase(mAllSpNameArray[i]);

			StringArray::iterator mAllSpNameArrayPos = mAllSpNameArray.begin() + i;

			mAllSpNameArray.erase(mAllSpNameArrayPos);	// check
			// mAllSpNameArray[i].erase();
			mAllSpCount = mAllSpNameArray.size();	// check that this works

			// Ignore array value more than actual map size
			if (mDrawObjectArray.size() < mShowObjectMap.size());
				//mDrawObjectArray.push_back(true); //added (LOJ: 2011.01.13 for bug 2215 fix)
		}
	}

	return true;
}

//------------------------------------------------------------------------------
// Real DeriveScRadius(const std::string &, Integer index, bool show = true)
//------------------------------------------------------------------------------
/*	An algorithm to find radius assuming average density 
 * and spherical spacecraft shape
 * ref: http://www.projectrho.com/public_html/rocket/advdesign.php
 * @scMass mass of spacecraft
 * @scaler independent scaler to make Sc easier to see
 */
Real VRInterface::DeriveScRadius(const Real &scMass, const Real &scaler) {
	Real	vol, 
			rad, 
			dens = 610.0; // kg/m^3, approximate density of Hubble ST
	vol = scMass / dens;
	rad = std::pow((vol * (3. / 4.) * (1 / M_PI)), 1. / 3.);
	return rad * scaler;
}

//------------------------------------------------------------------------------
// void BuildDynamicArrays()
//------------------------------------------------------------------------------
void VRInterface::BuildDynamicArrays()
{
	FixSpacePointArray();

	// Build Quaternion arrays
	for (int i = Q1; i < (Q4+1); i++) {	// append +1 if off-by-one
	// push back 4 times
		mScQArray.push_back(RealArray());
		mCbQArray.push_back(RealArray());
	}

	//// avoiding for loop here -- DOESN'T WORK
	//RealArray tempQ{ 0,0,0,0 };
	//mScQArray.push_back(tempQ);
	//mCbQArray.push_back(tempQ);

	// Add spacecraft objects to the list first
	for (int i = 0; i < mAllSpCount; i++) {
		if (mAllSpArray[i]) {
			if (mAllSpArray[i]->IsOfType(Gmat::SPACECRAFT)) {
				// Add to spacecraft list
				mScNameArray.push_back(mAllSpNameArray[i]);

				mScXArray.push_back(0.0);
				mScYArray.push_back(0.0);
				mScZArray.push_back(0.0);
				mScVxArray.push_back(0.0);
				mScVyArray.push_back(0.0);
				mScVzArray.push_back(0.0);

				mScQArray[Q1].push_back(0.0);
				mScQArray[Q2].push_back(0.0);
				mScQArray[Q3].push_back(0.0);
				mScQArray[Q4].push_back(0.0);

				mScPrevDataPresent.push_back(false);

				// Add to all object list
				mObjectNameArray.push_back(mAllSpNameArray[i]);
				mDrawOrbitArray.push_back(mDrawOrbitMap[mAllSpNameArray[i]]);
				mDrawObjectArray.push_back(mShowObjectMap[mAllSpNameArray[i]]);
				mObjectArray.push_back(mAllSpArray[i]);
				

				Real scRadius = 
					DeriveScRadius(((Spacecraft*)(mAllSpArray[i]))->GetRealParameter(42),200.);
				// enumeration value 42 for UpdateTotalMass(), 29 just for dry mass

				if (scRadius < mScRadiiMin || scRadius > 1000 || mDeriveRadii == false) {
					// assuming Sc at first half of SpRadii array
					mSpRadii.push_back(mScRadiiMin);
				}
				else 
					mSpRadii.push_back(scRadius);

				#pragma region old method
				//mSpRadii.push_back(0.1*((Spacecraft*)(mAllSpArray[i]))->GetRealParameter(42));
				//// 42 for UpdateTotalMass(), 29 just for dry mass
				//// assuming Sc at first half of SpRadii array
				//if (mSpRadii[i] < mScRadiiMin || mSpRadii[i] > 1000 || mDeriveRadii == false) {
				//	mSpRadii[i] = mScRadiiMin;
				//}
				#pragma endregion
			}
		}
	}

	// Add celestial bodies to the list

	for (int i = 0; i < mAllSpCount; i++) {
		if (mAllSpArray[i]) {
			if (mAllSpArray[i]->IsOfType(Gmat::CELESTIAL_BODY)) {
				mCbXArray.push_back(0.0);
				mCbYArray.push_back(0.0);
				mCbZArray.push_back(0.0);
				mCbVxArray.push_back(0.0);
				mCbVyArray.push_back(0.0);
				mCbVzArray.push_back(0.0);

				mCbQArray[Q1].push_back(0.0);
				mCbQArray[Q2].push_back(0.0);
				mCbQArray[Q3].push_back(0.0);
				mCbQArray[Q4].push_back(0.0);

				mCbPrevDataPresent.push_back(false);

				// add to name array
				mCbNameArray.push_back(mAllSpNameArray[i]);

				// add to Cb array
				mCbArray.push_back(mAllSpArray[i]);

				// add to all objects list 
				mObjectNameArray.push_back(mAllSpNameArray[i]);
				mDrawOrbitArray.push_back(mDrawOrbitMap[mAllSpNameArray[i]]);
				mDrawObjectArray.push_back(mShowObjectMap[mAllSpNameArray[i]]);
				mObjectArray.push_back(mAllSpArray[i]);

				// add to radii array. Syntax from ViewCanvas addOtherObject...
				mSpRadii.push_back(((CelestialBody*)(mAllSpArray[i]))->GetEquatorialRadius());

			}
			else {
				/*MessageInterface::ShowMessage
				("The SpacePoint name: %s has NULL pointer.\n Removal not implemented"
					"from the %s.\n", mAllSpNameArray[i].c_str(), GetTypeName().c_str());*/
			}
		}
	}

	mScCount = mScNameArray.size();
	mObjectCount = mObjectNameArray.size();
	mCbCount = mCbNameArray.size();

}


//------------------------------------------------------------------------------
// void ClearDynamicArrays()
//------------------------------------------------------------------------------
void VRInterface::ClearDynamicArrays()
{
	//updateHere

	mObjectNameArray.clear();
	mObjectArray.clear();
	mDrawOrbitArray.clear();
	mDrawObjectArray.clear();
	mScNameArray.clear();
	mCbNameArray.clear();

	mSpRadii.clear();

	mScXArray.clear();
	mScYArray.clear();
	mScZArray.clear();
	mScVxArray.clear();
	mScVyArray.clear();
	mScVzArray.clear();
	mScQArray.clear();
	mScPrevDataPresent.clear();
	//mScPrevEpoch.clear();
	//mScPrevX.clear();
	//mScPrevY.clear();
	//mScPrevZ.clear();
	//mScPrevVx.clear();
	//mScPrevVy.clear();
	//mScPrevVz.clear();
	//mScPrevQArray.clear();

	mCbXArray.clear();
	mCbYArray.clear();
	mCbZArray.clear();
	mCbVxArray.clear();
	mCbVyArray.clear();
	mCbVzArray.clear();
	mCbQArray.clear();
	mCbPrevDataPresent.clear();
	//mCbPrevEpoch.clear();
	//mCbPrevX.clear();
	//mCbPrevY.clear();
	//mCbPrevZ.clear();
	//mCbPrevVx.clear();
	//mCbPrevVy.clear();
	//mCbPrevVz.clear();
	//mCbPrevQArray.clear();
}


//------------------------------------------------------------------------------
// void UpdateObjectList(SpacePoint *sp, bool show = false)
//------------------------------------------------------------------------------
/**
 * Add non-spacecraft object to the list.
 */
 //------------------------------------------------------------------------------
void VRInterface::UpdateObjectList(SpacePoint *sp, bool show)
{
	// Add all spacepoint objects
	std::string name = sp->GetName();
	StringArray::iterator pos =
		find(mObjectNameArray.begin(), mObjectNameArray.end(), name);

	// if name not found, add to arrays
	if (pos == mObjectNameArray.end())
	{

		mObjectNameArray.push_back(name);
		mObjectArray.push_back(sp);
		mDrawOrbitMap[name] = show;
		mShowObjectMap[name] = show;
		mDrawOrbitArray.push_back(show);
		mDrawObjectArray.push_back(show);
		mObjectCount = mObjectNameArray.size();
	}
	else
	{

	}
}


//------------------------------------------------------------------------------
// void WriteCoordinateSystem(CoordinateSystem *cs, const std::string &label = "")
//------------------------------------------------------------------------------
// for debug
void VRInterface::WriteCoordinateSystem(CoordinateSystem *cs, const std::string &label)
{
	if (cs == NULL)
	{
		MessageInterface::ShowMessage("%s CoordinateSystem is NULL\n");
		return;
	}

	std::string originType = "UNKNOWN";
	std::string originName = "UNKNOWN";
	if (cs->GetOrigin())
	{
		originType = (cs->GetOrigin())->GetTypeName();
		originName = (cs->GetOrigin())->GetName();
	}

	MessageInterface::ShowMessage
	("%s = <%p>'%s', isInitialized = %d, origin = <%p><%s>'%s'\n", label.c_str(), cs,
		cs->GetName().c_str(), cs->IsInitialized(), cs->GetOrigin(), originType.c_str(),
		originName.c_str());
}

//------------------------------------------------------------------------------
// std::string	VRInterface::fileNameFromModerator(const std::string &fileType)
//------------------------------------------------------------------------------
// Method to return the script file name with '.script' replaced by another string
std::string	VRInterface::fileNameFromScript(const std::string &fileType) {
	std::string retString = Moderator::Instance()->GetScriptInterpreter()->GetMainScriptFileName();

	if (retString != "") {
		size_t pos = retString.find(".script");
		//if (pos == std::string::npos)
		//	retString = "json.json";
		//else
		retString.replace(pos, retString.length(), fileType);
		return retString;
	}
	else {
		retString = "json.json";
		return retString;
	}


}