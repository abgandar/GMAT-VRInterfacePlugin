#define JSON_DLL
//#define	SP	0
//#define	X	1
//#define	Y	2
//#define	Z	3
//#define	VX	4
//#define	VY	5
//#define	VZ	6

#include "VRInterfaceDefs.hpp"

#include <fstream>
#include <iostream>		// for string stream
#include <sstream>		// for string stream
#include <iomanip>		// for setw()
#include <stdio.h>		// for sprintf
// #include <json/json.h>


class VRInterface_API DataManager // : public VRInterface
{
public:
	// singleton implementation:
	// static DataManager &Instance();

	DataManager();
	virtual ~DataManager();

	DataManager(const DataManager &source);
	DataManager& operator=(const DataManager &rhs);

	void BuildDynamicBuffers(const Integer numSp);
	void ClearDynamicBuffers();

	void AddToBuffer(
		const Real time,
		const Integer scCount, const Integer cbCount,
		const StringArray scNames, const StringArray cbNames,
		const RealArray scPosX, const RealArray scPosY, const RealArray scPosZ,
		const RealArray scVelX, const RealArray scVelY, const RealArray scVelZ,
		const RealArray2D scQ,
		const RealArray cbPosX, const RealArray cbPosY, const RealArray cbPosZ,
		const RealArray cbVelX, const RealArray cbVelY, const RealArray cbVelZ,
		const RealArray2D cbQ,
		bool solving, Integer solverOption,
		bool drawing, const Integer maxData, bool inFunction = false);

	bool WriteToJson(
		const std::string& jsonFileName, 
		std::ofstream& jstream,
		const Integer& scCount, const Integer& cbCount,
		const StringArray scNames, const StringArray cbNames,
		const RealArray spRadii,
		const ColorMap &orbitColourMap, const Integer& maxData,
		const bool exportAttitude, const bool exportColours,
		const BooleanArray orbitsToDraw);

protected:

	// static bool maxDataExceeded;

	// prevents out-of-bounds exception due to Distribute being called twice
	// at EndOfRun
	static bool areBuffersCleared;

	static RealArray2D storedSpPosX; // [numSp][RealArray maxData]
	static RealArray2D storedSpPosY; 
	static RealArray2D storedSpPosZ; 
	static RealArray2D storedSpVelX; 
	static RealArray2D storedSpVelY; 
	static RealArray2D storedSpVelZ; 

	static RealArray2D storedSpQ1;	// [numSp][RealArray maxData]
	static RealArray2D storedSpQ2;
	static RealArray2D storedSpQ3;
	static RealArray2D storedSpQ4;

	static RealArray storedTime; // [maxData]

};

// implementations for methods to prevent unresolved externals 