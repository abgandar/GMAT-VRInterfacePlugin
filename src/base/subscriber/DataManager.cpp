#include "DataManager.hpp"
#include "MessageInterface.hpp"
#include "RgbColor.hpp"

// #define DEBUG_ADDTOBUFFER


// static data
// bool DataManager::maxDataExceeded = false;

RealArray2D DataManager::storedSpPosX;
RealArray2D DataManager::storedSpPosY;
RealArray2D DataManager::storedSpPosZ;
RealArray2D DataManager::storedSpVelX;
RealArray2D DataManager::storedSpVelY;
RealArray2D DataManager::storedSpVelZ;


RealArray2D DataManager::storedSpQ1;
RealArray2D DataManager::storedSpQ2;
RealArray2D DataManager::storedSpQ3;
RealArray2D DataManager::storedSpQ4;

RealArray DataManager::storedTime;

bool DataManager::areBuffersCleared = false;


//------------------------------------------------------------
// Constructor
//------------------------------------------------------------
DataManager::DataManager() {
	// nothing to do here
}

//------------------------------------------------------------
// Destructor
//------------------------------------------------------------
DataManager::~DataManager() {
	// clear up, nothing atm
}

//------------------------------------------------------------
// Copy constructor
//------------------------------------------------------------
DataManager::DataManager(const DataManager &dm) {
	storedSpPosX = dm.storedSpPosX;
	storedSpPosY = dm.storedSpPosY;
	storedSpPosZ = dm.storedSpPosZ;
	storedSpVelX = dm.storedSpVelX;
	storedSpVelY = dm.storedSpVelY;
	storedSpVelZ = dm.storedSpVelZ;

	storedSpQ1 = dm.storedSpQ1;
	storedSpQ2 = dm.storedSpQ2;
	storedSpQ3 = dm.storedSpQ3;
	storedSpQ4 = dm.storedSpQ4;

	storedTime = dm.storedTime;
}

//------------------------------------------------------------
// Assignment Operator
//------------------------------------------------------------
DataManager& DataManager::operator=(const DataManager& dm) {
	storedSpPosX = dm.storedSpPosX;
	storedSpPosY = dm.storedSpPosY;
	storedSpPosZ = dm.storedSpPosZ;
	storedSpVelX = dm.storedSpVelX;
	storedSpVelY = dm.storedSpVelY;
	storedSpVelZ = dm.storedSpVelZ;

	storedSpQ1 = dm.storedSpQ1;
	storedSpQ2 = dm.storedSpQ2;
	storedSpQ3 = dm.storedSpQ3;
	storedSpQ4 = dm.storedSpQ4;

	storedTime = dm.storedTime;

	return *this;
}

//------------------------------------------------------------
// Initialise dynamic buffer arrays 
//------------------------------------------------------------
/*
* @maxData -- sets upper size of arrays
* @numSp -- number of Objects
*/
void DataManager::BuildDynamicBuffers(const Integer numSp) {
	// RealArray numSpCol (numSp);
	for (int i = 0; i < (numSp); i++) {
		storedSpPosX.push_back(RealArray());
		storedSpPosY.push_back(RealArray());
		storedSpPosZ.push_back(RealArray());
		storedSpVelX.push_back(RealArray());
		storedSpVelY.push_back(RealArray());
		storedSpVelZ.push_back(RealArray());

		storedSpQ1.push_back(RealArray());
		storedSpQ2.push_back(RealArray());
		storedSpQ3.push_back(RealArray());
		storedSpQ4.push_back(RealArray());
	}
}

//------------------------------------------------------------
// Clear buffers
//------------------------------------------------------------
/*
* Deallocation of memory
*/
void DataManager::ClearDynamicBuffers() {
	storedSpPosX.clear();
	storedSpPosY.clear();
	storedSpPosZ.clear();
	storedSpVelX.clear();
	storedSpVelY.clear();
	storedSpVelZ.clear();
	
	storedSpQ1.clear();
	storedSpQ2.clear();
	storedSpQ3.clear();
	storedSpQ4.clear();

	storedTime.clear();

	areBuffersCleared = true;
}

//------------------------------------------------------------
// buffer orbit data
//------------------------------------------------------------
/*
* Called instead of UpdateGlPlot. Miror logic flow in here
* Consider renaming and implementing in other .cpp file

*/
void DataManager::AddToBuffer(
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
	bool drawing, const Integer maxData, bool inFunction)
{
	areBuffersCleared = false;

	#ifdef DEBUG_ADDTOBUFFER
	MessageInterface::ShowMessage
	("===========================================================================\n");
	MessageInterface::ShowMessage
		("DataManager::AddToBuffer(): time=%0.11f, scCount=%d, cbCount=%d",time,scCount,cbCount);
	MessageInterface::ShowMessage
	("\nscNames:	|posX:		|posY:		|posZ:		|velX:		|velY:		|velZ:	");
	
	for (int i = 0; i<scCount; i++) {
		MessageInterface::ShowMessage
		("\n%s	%0.2f		%0.2f		%0.2f		%0.2f		%0.2f		%0.2f",
		scNames[i].c_str(),scPosX[i],scPosY[i],scPosZ[i],scVelX[i],scVelY[i],scVelZ[i]);
	}
	MessageInterface::ShowMessage
	("\ncbNames:	|posX:		|posY:		|posZ:		|velX:		|velY:		|velZ:	");

	for (int i = 0; i < cbCount; i++) {
		MessageInterface::ShowMessage
		("\n%s		%0.2f		%0.2f		%0.2f		%0.2f		%0.2f		%0.2f",
			cbNames[i].c_str(), cbPosX[i], cbPosY[i], cbPosZ[i], cbVelX[i], cbVelY[i], cbVelZ[i]);
	}
	MessageInterface::ShowMessage(
		"\ntargetColourMap, solving=%d, solverOption=%d,\n",
		solving, solverOption,
		"\nupdateCanvas=%d, drawing=%d, inFunction=%d\n", updateCanvas, drawing, inFunction);
	#endif

	// push back buffers with current sc state
	for (int i = 0; i < scCount; i++) {
		storedSpPosX[i].push_back(scPosX[i]);
		storedSpPosY[i].push_back(scPosY[i]);
		storedSpPosZ[i].push_back(scPosZ[i]);
		storedSpVelX[i].push_back(scVelX[i]);
		storedSpVelY[i].push_back(scVelY[i]);
		storedSpVelZ[i].push_back(scVelZ[i]);

		storedSpQ1[i].push_back(scQ[Q1][i]);
		storedSpQ2[i].push_back(scQ[Q2][i]);
		storedSpQ3[i].push_back(scQ[Q3][i]);
		storedSpQ4[i].push_back(scQ[Q4][i]);
	}
		
	// push back buffers with current cb state
	// maintaining indexing by starting from last sc
	for (int i = 0; i < cbCount; i++) {
		storedSpPosX[scCount + i].push_back(cbPosX[i]);
		storedSpPosY[scCount + i].push_back(cbPosY[i]);
		storedSpPosZ[scCount + i].push_back(cbPosZ[i]);
		storedSpVelX[scCount + i].push_back(cbVelX[i]);
		storedSpVelY[scCount + i].push_back(cbVelY[i]);
		storedSpVelZ[scCount + i].push_back(cbVelZ[i]);

		storedSpQ1[scCount + i].push_back(cbQ[Q1][i]);
		storedSpQ2[scCount + i].push_back(cbQ[Q2][i]);
		storedSpQ3[scCount + i].push_back(cbQ[Q3][i]);
		storedSpQ4[scCount + i].push_back(cbQ[Q4][i]);
	}

	storedTime.push_back(time);
}

//------------------------------------------------------------
// Write buffers to json file
//------------------------------------------------------------
/*
* @maxData passed onto here for final data control
* Writes buffers from mission to json file
* Triggers at end of mission run
*/
bool DataManager::WriteToJson(
	const std::string& jsonFileName,
	std::ofstream& jstream, 
	const Integer& scCount, const Integer& cbCount,
	const StringArray scNames, const StringArray cbNames,
	const RealArray spRadii,
	const ColorMap &orbitColourMap, const Integer& maxData,
	const bool exportAttitude, const bool exportColours,
	const BooleanArray orbitsToDraw) {

	if (scCount == 0 && cbCount == -842150451) {
		// last resort error handling. cbCount not guaranteed to be this value
		// if no spacepoints were selected. Consider: cbCount > reasonableValue
		MessageInterface::PopupMessage(Gmat::ERROR_, "There is no data to write.\n"
			"Are you sure you have selected any objects to visualise?\n"
			"No data was written during this run.");
		return false;
	}

	if (areBuffersCleared == false) {	
	// prevents out-of-bounds exception, as func called twice at end of run
		bool maxDataExceeded = false;

		// max data check
		// * this should check if the TOTOL size of ALL arrays
		// * exceeds data limit, not just first spacepoint
		if (storedSpPosX[0].size() >= maxData) {
			maxDataExceeded = true;
			MessageInterface::PopupMessage(Gmat::INFO_, "Max Data exceeded. \n Handling stationary objects.\n"
																		"THIS IS NOT YET IMPLEMENTED");
			// HandeStationary();
				// something similar to Unity's LineUtility algo
				// WARNING -- array checker in Unity must be updated. Or append flag to JSON
		}
		if (storedSpPosX[0].size() >= maxData) {
			maxDataExceeded = true;
			MessageInterface::PopupMessage(Gmat::INFO_, "Max Data exceeded again. \n Thinning out the data set.\n"
																	"THIS IS NOT YET IMPLEMENTED");
			// ThinOut();
				// Take out every nth point 
				// OR implement dimensionality reduction algos
				// e.g. https://www.analyticsvidhya.com/blog/2018/08/dimensionality-reduction-techniques-python/
		}
		else {
			maxDataExceeded = false;
		}

	// initialise JSON file

	// open file and clear contents
	// OR flag used as out overridden by trunc
		if (!jstream.is_open()) {
			jstream.open(jsonFileName, std::ofstream::out | std::ofstream::trunc);
		}
		
		// Close the stream 
		if (jstream.is_open()) {
			jstream.close();
		}

		if (!jstream.is_open()) {
			// open in append mode. Fine as only one io event takes place
			jstream.open(jsonFileName, std::ofstream::app);
		}

		std::ostringstream jsonBuilder;
		jsonBuilder << "{";
		jsonBuilder << "\t" << "\"info\": {\n";
		jsonBuilder << "\t\t" << "\"coordinates\": \"cartesian\",\n";
		jsonBuilder << "\t\t" << "\"units\": \"km\"\n";
		jsonBuilder << "\t" << "},\n";

		jsonBuilder << "\t" << "\"orbits\": [\n";
		for (int i = 0; i < scCount; i++) {	// spacecraft done first

			jsonBuilder << "\t\t" << "{\n";
			jsonBuilder << "\t\t\t" << "\"name\": \"" << scNames[i] << "\",\n";
			// for now, just draw all objects as line,display
			// and default colour schemes
			jsonBuilder << "\t\t\t" << "\"display\": \"" << "line,point" << "\",\n";
			jsonBuilder << "\t\t\t" << "\"radius\": " << spRadii[i] << ",\n";

			//// this one outputs Int colour
			//jsonBuilder << "\t\t\t" << "\"color\": \"" << orbitColourMap.find(scNames[i])->second << "\",\n";

			//// this one outputs [250 0 0]
			//jsonBuilder << "\t\t\t" << "\"color\": \"" << 
			//	RgbColor::ToRgbString(orbitColourMap.find(scNames[i])->second) << "\",\n";

			if (exportColours == true) {
				// this one ouputs rrr,ggg,bbb
				RgbColor tempColor = orbitColourMap.find(scNames[i])->second;
				char colorBuffer[20];	// array size can be reduced to 13, as outputs rrr,ggg,bbb
				sprintf(colorBuffer, "%d,%d,%d", tempColor.Red(), tempColor.Green(), tempColor.Blue());		// tempColor.Alpha()

				jsonBuilder << "\t\t\t" << "\"color\": \"" << colorBuffer << "\",\n";
			}
			else {
				jsonBuilder << "\t\t\t" << "\"color\":,\n";	// either this, or nothing at all
			}

			jsonBuilder << "\t\t\t" << "\"eph\": [\n";
			for (int j = 0; j < storedSpPosX[0].size(); j++) {
				jsonBuilder << "\t\t\t\t" << "[" << std::setprecision(10)
					<< std::setw(14) << storedSpPosX[i][j] << ","
					<< std::setw(14) << storedSpPosY[i][j] << ","
					<< std::setw(14) << storedSpPosZ[i][j] << ","
					<< std::setw(14) << storedSpVelX[i][j] << ","
					<< std::setw(14) << storedSpVelY[i][j] << ","
					<< std::setw(14) << storedSpVelZ[i][j] << "],\n";
			}
			jsonBuilder << "\t\t\t" << "],\n";

			if (exportAttitude == true) {
				jsonBuilder << "\t\t\t" << "\"att\": [\n";
				for (int j = 0; j < storedSpQ1[0].size(); j++) {
					jsonBuilder << "\t\t\t\t" << "[" << std::setprecision(10)
						<< std::setw(14) << storedSpQ1[i][j] << ","
						<< std::setw(14) << storedSpQ2[i][j] << ","
						<< std::setw(14) << storedSpQ3[i][j] << ","
						<< std::setw(14) << storedSpQ4[i][j] << "],\n";
				}
				jsonBuilder << "\t\t\t" << "],\n";
			}

			jsonBuilder << "\t\t\t" << "\"time\": [";
			for (int k = 0; k < storedTime.size(); k++) {
				jsonBuilder << std::setprecision(10) << storedTime[k] << ",";
			}
			jsonBuilder << "]\n";
			jsonBuilder << "\t\t" << "},\n";

		}
		for (int i = 0; i < cbCount; i++) {	// celestial bodies done second

			jsonBuilder << "\t\t" << "{\n";
			jsonBuilder << "\t\t\t" << "\"name\": \"" << cbNames[i] << "\",\n";
			// for now, just draw all objects as line,display
			// and default colour schemes
			jsonBuilder << "\t\t\t" << "\"display\": \"" << "line,point" << "\",\n";
			jsonBuilder << "\t\t\t" << "\"radius\": " << spRadii[i + scCount] << ",\n";

			if (exportColours == true) {
				RgbColor tempColor = orbitColourMap.find(cbNames[i])->second;
				char colorBuffer[20];
				sprintf(colorBuffer, "%d,%d,%d", tempColor.Red(), tempColor.Green(), tempColor.Blue());		// tempColor.Alpha()

				jsonBuilder << "\t\t\t" << "\"color\": \"" << colorBuffer << "\",\n";
			}
			else {
				jsonBuilder << "\t\t\t" << "\"color\":,\n";
			}

			jsonBuilder << "\t\t\t" << "\"eph\": [\n";
			for (int j = 0; j < storedSpPosX[0].size(); j++) {
				jsonBuilder << "\t\t\t\t" << "[" << std::setprecision(10)
					<< std::setw(12) << storedSpPosX[i + scCount][j] << ","
					<< std::setw(12) << storedSpPosY[i + scCount][j] << ","
					<< std::setw(12) << storedSpPosZ[i + scCount][j] << ","
					<< std::setw(12) << storedSpVelX[i + scCount][j] << ","
					<< std::setw(12) << storedSpVelY[i + scCount][j] << ","
					<< std::setw(12) << storedSpVelZ[i + scCount][j] << "],\n";
			}
			jsonBuilder << "\t\t\t" << "],\n";

			if (exportAttitude == true) {
				jsonBuilder << "\t\t\t" << "\"att\": [\n";
				for (int j = 0; j < storedSpQ1[0].size(); j++) {
					jsonBuilder << "\t\t\t\t" << "[" << std::setprecision(10)
						<< std::setw(14) << storedSpQ1[i + scCount][j] << ","
						<< std::setw(14) << storedSpQ2[i + scCount][j] << ","
						<< std::setw(14) << storedSpQ3[i + scCount][j] << ","
						<< std::setw(14) << storedSpQ4[i + scCount][j] << "],\n";
				}
				jsonBuilder << "\t\t\t" << "],\n";
			}

			jsonBuilder << "\t\t\t" << "\"time\": [";
			for (int k = 0; k < storedTime.size(); k++) {
				jsonBuilder << std::setprecision(10) << storedTime[k] << ",";
			}
			jsonBuilder << "]\n";
			jsonBuilder << "\t\t" << "},\n";

		}

		jsonBuilder << "\t" << "]\n";
		jsonBuilder << "}";

		jstream << jsonBuilder.str();

		// Close the stream 
		if (jstream.is_open()) {
			jstream.close();
		}
		ClearDynamicBuffers();
		return true;
	}
	else if (areBuffersCleared == true)
		return false;
}