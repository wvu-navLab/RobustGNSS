/**
 * @file   GnssData.cpp
 * @brief  Tools required to read/write GNSS data
 * @author Ryan Watson
 */

#include <gtsam/gnssNavigation/GnssData.h>

using namespace std;

namespace gtsam {

	vector<rnxData> readGNSS(const std::string &fileLoc) {
		/*
		inputs ::
			fileLoc ---> path to data file
		output ::
			data ---> gnss data in gtsam format
                     { epoch, svn, satXYZ, computed_range, rangeLC, phaseLC }
		*/
		vector<rnxData> data;
		string data_file = findExampleDataFile(fileLoc);
		ifstream is(data_file.c_str());

		while (is) {
			int svn, count;
			double week, sow, satX, satY, satZ, rho, cb, rel, rangeLC, phaseLC;
			Point3 satXYZ, computed_range;
            string constellation;
			is >> week >> sow  >> count >> constellation
		       >> svn >> rangeLC >> phaseLC
			   >> rho >> cb >> rel >> satX >> satY >> satZ;
			data.push_back(rnxData(sow, count, svn ,Point3(satX,satY,satZ),
					       (rho - cb - rel) , rangeLC, phaseLC));
		}
		is.clear(); /* clears the end-of-file and error flags */
		return data;
	}


	void writeStates(Values &results, string outputFile){
		/*
		inputs ::
			results -->
			outputFile --> name of file to write state est. to. [string]
		*/
		ofstream outFile(outputFile.c_str());
		int epoch = 0;
		Values::ConstFiltered<gnssStateVec> result_poses = results.filter<gnssStateVec>();
		foreach (const Values::ConstFiltered<gnssStateVec>::KeyValuePair& key_value, result_poses)
		{
			gnssStateVec p = key_value.value;
			outFile << "stateVec " << epoch++
				<< " "  << p.x() << " " << p.y()
				<< " " 	<< p.z() << " " << p.cb()
				<< " " << p.tz() << endl;
		}
	}

    void writeNavFrame(Values &results, Point3 &nom, string outputFile){
		ofstream outFile(outputFile.c_str());
		int epoch = 0;
		Values::ConstFiltered<gnssStateVec> result_poses = results.filter<gnssStateVec>();
		foreach (const Values::ConstFiltered<gnssStateVec>::KeyValuePair& key_value, result_poses)
		{
            		gnssStateVec p = key_value.value;
            		Point3 delta(p.x(),p.y(),p.z());
			        Point3 ecef = (nom - delta);
            		Point3 enu = xyz2enu(ecef,nom);
            		outFile << epoch++ << " " << enu.x()
				<< " " << enu.y() << " " << enu.z() << endl;

        	}
    }

    void writeEarthFrame(Values &results, Point3 &nom, string outputFile){
		ofstream outFile(outputFile.c_str());
		int epoch = 0;
		Values::ConstFiltered<gnssStateVec> result_poses = results.filter<gnssStateVec>();
		foreach (const Values::ConstFiltered<gnssStateVec>::KeyValuePair& key_value, result_poses)
		{
            		gnssStateVec p = key_value.value;
            		Point3 delta(p.x(),p.y(),p.z());
			        Point3 ecef = nom - delta;
            		outFile << epoch++ << " " << ecef.x()
				<< " " << ecef.y() << " " << ecef.z() << endl;

        	}
    }

	void writeSwitches( Values &results, string outputFile, vector<string> switchIndex){
		/*
		inputs ::
			results --> optimizer output
			outputFile --> name of file to write switch states to [string]
			Optional ::
				switchIndex --> index by epoch and visible satellite (i.e. obs 4 would be Switch_0_4) [vector]
		*/
		ofstream outFile(outputFile.c_str());
 		int epoch = 0;
		Values::ConstFiltered<SwitchVariableLinear> result_switches = results.filter<SwitchVariableLinear>();
		foreach (const Values::ConstFiltered<SwitchVariableLinear>::KeyValuePair& key_value, result_switches) {
			int index = epoch++;
			outFile << switchIndex[index] << " "
				<< index << " " <<  key_value.value.value() << endl;
		}
	}
}
