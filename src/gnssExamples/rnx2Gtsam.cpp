/*
    *  @file   rnxToGtsam.cpp
    *  @author Ryan 
    *  @brief  Script to convert Rinex File format to format utilized by GTSAM. 

    * To do ::
        * Currenlty only works with SP3 
        * Need to get nom. pos. from rinex header
        * Get DOY from header for trop estimation 
        * need to write to file. Currently only prints to screen.
*/

// GPSTK
#include <gtsam/gpstk/MJD.hpp>
#include <gtsam/gpstk/PowerSum.hpp>
#include <gtsam/gpstk/Decimate.hpp>
#include <gtsam/gpstk/TropModel.hpp>
#include <gtsam/gpstk/BasicModel.hpp>
#include <gtsam/gpstk/CommonTime.hpp>
#include <gtsam/gpstk/PCSmoother.hpp>
#include <gtsam/gpstk/SimpleFilter.hpp>
#include <gtsam/gpstk/MWCSDetector.hpp>
#include <gtsam/gpstk/SatArcMarker.hpp>
#include <gtsam/gpstk/Rinex3NavData.hpp>
#include <gtsam/gpstk/GNSSconstants.hpp>              
#include <gtsam/gpstk/ComputeLinear.hpp>
#include <gtsam/gpstk/GPSWeekSecond.hpp>
#include <gtsam/gpstk/DataStructures.hpp>
#include <gtsam/gpstk/Rinex3ObsStream.hpp>
#include <gtsam/gpstk/Rinex3NavStream.hpp>
#include <gtsam/gpstk/ComputeTropModel.hpp>
#include <gtsam/gpstk/SP3EphemerisStore.hpp>
#include <gtsam/gpstk/ComputeSatPCenter.hpp>
#include <gtsam/gpstk/EclipsedSatFilter.hpp>
#include <gtsam/gpstk/GPSEphemerisStore.hpp>
#include <gtsam/gpstk/RequireObservables.hpp>
#include <gtsam/gpstk/CorrectObservables.hpp>
#include <gtsam/gpstk/LinearCombinations.hpp>
#include <gtsam/gpstk/GravitationalDelay.hpp>
#include <gtsam/gpstk/PhaseCodeAlignment.hpp>

// GTSAM
#include <gtsam/slam/dataset.h>
#include <gtsam/gnssNavigation/GnssData.h>

// BOOST 
#include <boost/program_options.hpp>

// STD 
#include <iomanip>
#include <iostream>

using namespace std;
using namespace gpstk;
using namespace gtsam;
using namespace boost;

namespace po = boost::program_options;

int main(int argc, char *argv[])
{

    string rnx_file, nav_file, sp3_file, out_file;
    int dec_int, itsBelowThree = 0, count = 0;

    cout << fixed << setprecision(3);   // Set a proper output format

    po::options_description desc("Available options");
    desc.add_options()
    ("help,h", "Print help message")
    ("obs", po::value<string>(&rnx_file)->default_value(""), 
            "Observation file to read")
    ("nav", po::value<string>(&nav_file)->default_value(""), 
            "Navigation file to read.")
    ("sp3", po::value<string>(&sp3_file)->default_value(""),
            "SP3 file to read.")
    ("out", po::value<string>(&out_file)->default_value(""), 
            "output file.")
    ("dec", po::value<int>(&dec_int)->default_value(0), 
            "decimate input obs file");
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
    po::notify(vm);

    if ( rnx_file.empty() )
    {
        cout << " Must pass in obs file !!! Try --obs " << desc << endl; 
        exit(1);
    }
    if ( nav_file.empty() && sp3_file.empty() )
    {
        cout << " Must pass in ephemeris file !!! Try --sp3 or --nav" << desc << endl; 
        exit(1);
    }

    string obs_path = findExampleDataFile(rnx_file);
    string nav_path = findExampleDataFile(sp3_file);

    // Create the input observation file stream
    Rinex3ObsStream rin(obs_path);

    // Declare a "SP3EphemerisStore" object to handle precise ephemeris
    SP3EphemerisStore SP3EphList;

    // Set flags to reject satellites with bad or absent positional
    // values or clocks
    SP3EphList.rejectBadPositions(true);
    SP3EphList.rejectBadClocks(true);

    // Load all the SP3 ephemerides files
    SP3EphList.loadFile(nav_path);

      // BELL station nominal position
    Position nominalPos(856295.3346, -4843033.4111, 4048017.6649);

      // Declare a NeillTropModel object, setting the defaults
    NeillTropModel neillTM( nominalPos.getAltitude(),
                           nominalPos.getGeodeticLatitude(), 347);

    // This is the GNSS data structure that will hold all the
    // GNSS-related information
    gnssRinex gRin;

    RequireObservables requireObs(TypeID::L1);
    requireObs.addRequiredType(TypeID::L2);

    // Declare a simple filter object to screen PC
    SimpleFilter lcFilter;
    lcFilter.setFilteredType(TypeID::LC);

    // Object to smooth RangeLC
    PCSmoother smoothPC;
    smoothPC.setMaxWindowSize(100); // set smooth window

      // Declare a couple of basic modelers
    BasicModel basic(nominalPos, SP3EphList);

      // Objects to mark cycle slips
    MWCSDetector markCSMW;      // Checks Merbourne-Wubbena cycle slip

      // Declare an object to correct observables
    CorrectObservables corr(SP3EphList);

      // Objects to compute the tropospheric data
    ComputeTropModel computeTropo(neillTM);

      // object def several linear combinations
    LinearCombinations comb;

    // Object to compute linear combinations for cycle slip detection
    ComputeLinear linear1(comb.pdeltaCombination);
    linear1.addLinear(comb.ldeltaCombination);
    linear1.addLinear(comb.mwubbenaCombination);
    linear1.addLinear(comb.liCombination);

    // object to compute the ionosphere-free combinations
    ComputeLinear linear2(comb.pcCombWithC1);
    linear2.addLinear(comb.lcCombination);

    // object to compute prefit residuals
    ComputeLinear linear3(comb.pcPrefit);
    linear3.addLinear(comb.lcPrefit);

    // Object to keep track of satellite arcs
    SatArcMarker markArc;
    markArc.setDeleteUnstableSats(true);

    // Objects to compute gravitational delay effects
    GravitationalDelay grDelay(nominalPos);

    // Object to align phase with code measurements
    PhaseCodeAlignment phaseAlign;

    // Object to remove eclipsed satellites
    EclipsedSatFilter eclipsedSV;

    TypeIDSet tset;
    tset.insert(TypeID::prefitC);
    tset.insert(TypeID::prefitL);

    //Decimate decimateData(15.0, 1.0, SP3EphList.getInitialTime());

    // Loop over all data epochs
    while(rin >> gRin)
    {
        TimeSystem sys;
        sys.fromString("GPS");
        CommonTime time(gRin.header.epoch);
        time.setTimeSystem(sys);
        GPSWeekSecond gpstime( time );
        
        try
        {
            gRin >> requireObs      // Check if required observations are present
            >> linear1          // Compute linear combinations to detect CS
            >> markCSMW         // Mark cycle slips: Melbourne-Wubbena
            >> markArc          // Keep track of satellite arcs
            >> basic            // Compute the basic components of model
            >> eclipsedSV       // Remove satellites in eclipse
            >> grDelay          // Compute gravitational delay
            >> computeTropo     // Compute tropospheric effect
            >> linear2          // Compute ionosphere-free combinations
            >> lcFilter         // Filter out spurious data
            >> phaseAlign       // Align phases with codes
            >> smoothPC        // Compute carrier smoothed rangelc observables 
            >> linear3;          // Compute prefit residuals
        }
        catch(Exception& e)
        {
            //cerr << "Exception at epoch: " << time << "; " << e << endl;
            continue;
        }
        catch(...)
        {
            cerr << "Unknown exception at epoch: " << time << endl;
            continue;
        }
        TypeIDSet types;
        types.insert(TypeID::satX);
        types.insert(TypeID::satY);
        types.insert(TypeID::satZ);
        types.insert(TypeID::PC);
        types.insert(TypeID::LC);
        types.insert(TypeID::rho);
        types.insert(TypeID::tropo);
        types.insert(TypeID::dtSat);
        types.insert(TypeID::rel);
        gRin.keepOnlyTypeID(types);
         
        // Iterate through the GNSS Data Structure
        satTypeValueMap::const_iterator it;
        typeValueMap::const_iterator itObs;
        if (gRin.numSats() > 3) 
        {
            if ( itsBelowThree > 0 )
            {
                itsBelowThree = 0;             
               continue;
            }
            for (it = gRin.body.begin(); it!= gRin.body.end(); it++) 
            {  
                cout << gpstime.week << " ";
                cout << gpstime.sow << " ";
                cout << count << " ";
                cout << (*it).first << " ";
                
                typeValueMap::const_iterator itObs;
                for( itObs  = (*it).second.begin();  itObs != (*it).second.end(); itObs++ )
                {
                    cout << (*itObs).first << " ";
                    cout << (*itObs).second << " ";
                } 
                cout << endl;
            } 
            count++;
        } 
	else { itsBelowThree++; }
    }
return 0;
}
