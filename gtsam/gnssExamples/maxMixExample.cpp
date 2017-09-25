/**
    *  @file   maxMixExample.cpp
    *  @author Ryan
    *  @brief  Simple implementaion of the max-mixture pseudorange factor. 

    * how to run ::
      * ./maxMixExample -h
      * ./maxMixExample -g collect1.gtsam --writeENU --mixWeight 1e-6
**/

// GTSAM
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/gnssNavigation/GnssData.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/FolderUtils.h>
#include <gtsam/gnssNavigation/GnssPostfit.h>
#include <gtsam/gnssNavigation/gnssStateVec.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/robustModels/PseudorangeMaxMix.h>
#include <gtsam/gnssNavigation/PseudorangeFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// BOOST
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/serialization/export.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

// STANDARD
#include <fstream>
#include <iostream>
#include <ios>

// Intel Threading Building Block
#ifdef GTSAM_USE_TBB
  #include <tbb/tbb.h>
  #undef max // TBB seems to include windows.h and we don't want these macros
  #undef min
#endif

using namespace std;
using namespace gtsam;
using namespace boost;
namespace po = boost::program_options;

typedef noiseModel::Diagonal diagNoise;
namespace NM = gtsam::noiseModel;

int main(int argc, char** argv) {

    bool noTrop, switchFactor, switchFactorWBetween, writeGraph, elWeight;
    bool writeENU, writeECEF, loose, tight, robust = false;
    int currKey=-1, trop=1, startEpoch=0, satKeyPrev=-1, sc=1, nThreads;
    int num_gps_factors=0, factorCount=0, lastStep, firstStep, initIter;
    double switchInit = 1.0, residualThresh, measWeight, weight;
    string gnssFile, outputFile, residualTxtInit="initResidaul.txt";
    string residualTxtOut="finalResidual.txt",textExtension=".txt", strategy;
    string switchExtension = "Switch.txt", graphExtension=".dot", dir;
    vector<int> numFactors;
    vector<string> satIndexLiteral;
    vector<rnxData> data;

    // define std out print color
    const string red("\033[0;31m");
    const string green("\033[0;32m");
    const string lineBreak = "###########################\n";

    NonlinearFactorGraph graph, graphMaxMix;

    po::options_description desc("Available options");
    desc.add_options()
    ("help,h", "Print help message")
    ("gpsObs,i", po::value<string>(&gnssFile)->default_value(""),
     "Input GNSS data file")
    ("outFile,o", po::value<string>(&outputFile)->default_value("initResults"),
     "Write graph and solution to the specified file.")
    ("firstStep,f", po::value<int>(&firstStep)->default_value(0),
     "First step to process from the dataset file")
    ("lastStep,l", po::value<int>(&lastStep)->default_value(-1),
     "Last step to process, or -1 to process until the end of the dataset")
    ("threads", po::value<int>(&nThreads)->default_value(-1),
    "Number of threads, or -1 to use all processors")
    ("noTrop", "Will turn residual troposphere estimation off. Troposphere will still be modeled.")
    ("initIter",po::value<int>(&initIter)->default_value(50),
     "Number of iterations before initial postfit data edit")
    ("dir", po::value<string>(&dir)->default_value(""),
     "Total path to store generated data")
    ("elWeight,el", "Elevation angle dependant measuremnt weighting")
    ("measWeight" , po::value<double>(&measWeight)->default_value(5.0),
    "Noise applied to each GNSS observable")
    ("mixWeight",po::value<double>(&weight)->default_value(1e-6),
     "Weighting applied to hypothesis noise model for Max-Mix")
    ("writeGraph",
     "Write graph to text file. Do not write large graphs (i.e. Nodes>=100)")
    ("writeECEF", "write ecef solution to file")
    ("writeENU", "write enu solution to file")
    ;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
    po::notify(vm);

    writeGraph = (vm.count("writeGraph")>0);
    writeENU = (vm.count("writeENU")>0);
    writeECEF = (vm.count("writeECEF")>0);
    noTrop = (vm.count("noTrop") > 0);

    if ( gnssFile.empty() ) {
        cout << red << "\n\n GNSS data must be specified\n"
             << "\n\n" << green << desc << endl;
        exit(1);
    }

    if ( noTrop ) { trop = 0; }

    #ifdef GTSAM_USE_TBB
    std::auto_ptr<tbb::task_scheduler_init> init;
    if(nThreads > 0) {
        cout << "\n\n Using " << nThreads << " threads " << endl;
        init.reset(new tbb::task_scheduler_init(nThreads));
    }
    else
        cout << green << " \n\n Using threads for all processors" << endl;
    #else
        if(nThreads > 0) {
          cout << red <<" \n\n GTSAM is not compiled with TBB, so threading is"
                    << " disabled and the --threads option cannot be used."
                    << endl;
          exit(1);
        }
    #endif

    // set up directory to store all generated data
    if ( dir.empty() ) { dir = getTimestamp(); }
    makeDir( dir );
    chdir( dir.c_str() );

    // Specify the starting location of the platform in ECEF XYZ. 
    Point3 nomXYZ(854815.0369, -4842487.5527,  4048906.3786);
    Point3 nomNED(0.0,0.0,0.0);

    // Define the noise model
    SharedNoiseModel hypothesis = diagNoise::Gaussian::Covariance(
        ( (Vector(1) << measWeight ).finished() ).asDiagonal() );
    SharedNoiseModel null_model = diagNoise::Gaussian::Covariance(
         ( (Vector(1) << measWeight/weight).finished() ).asDiagonal() );
    SharedNoiseModel stateProcessNoise = diagNoise::Sigmas(
        (Vector(5) << 15.0, 15.0, 15.0, 1e3 ,0.1).finished() );
    SharedNoiseModel priorNoise = diagNoise::Sigmas(
        (Vector(5) << 1e5,1e5,1e5, 3e8, 0.30).finished() );

    Values initialEstimate;
    gnssStateVec initEst((Vector(5) << 0.0, 0.0, 0.0, 0.0, 0.0).finished());

    // Read GNSS data
    try { data = readGNSS(gnssFile); }
    catch(std::exception& e)
    {
        cout << red << "\n\n Cannot read GNSS data file " << endl;
        exit(1);
    }

    strategy = "GNSS only with max-mixtures robust noise model";
    cout << green << "\n\n" << lineBreak << " GNSS Data File :: "
        << gnssFile << endl;
    cout << "\n Processing Strategy  :: "  << strategy << endl;
    cout << lineBreak << endl;

    if ( firstStep != 0 ) {
        for( unsigned int i = 0; i < data.size(); i++ ) {
            if ( firstStep == get<0>(data[i]) ) { break; }
            ++startEpoch;
        }
    }
    if ( lastStep < 0 ) { lastStep = get<0>(data.back()); }

    // Construct Graph --- Only Pseudorange factors
    for(unsigned int i = startEpoch; i < data.size(); i++ ) {

      // Grab current epoch's observables
      int gnssTime = get<0>(data[i]);
      int currKey = get<1>(data[i]);
      int nextKey = get<1>(data[i+1]);
      int svn = get<2>(data[i]);
      Point3 satXYZ = get<3>(data[i]);
      double rho = get<4>(data[i]);
      double range = get<5>(data[i]);

      // Add prior and between factors 
      if ( currKey != nextKey ) {
          graph.add(PriorFactor<gnssStateVec>(Symbol('x',currKey) , initEst, priorNoise));
          ++factorCount;
          initialEstimate.insert(Symbol('x',currKey),initEst);
          if ( currKey > startEpoch ) {
              graph.add(BetweenFactor<gnssStateVec>(Symbol('x',currKey),
                        Symbol('x',currKey-1), initEst, stateProcessNoise));

              graphMaxMix.add(BetweenFactor<gnssStateVec>(Symbol('x',currKey),
                        Symbol('x',currKey-1), initEst, stateProcessNoise));

              ++factorCount;
         }
        if ( (lastStep > 0)  && (lastStep == nextKey) ) { break; }
      }

      Point3 satNED = ned2enu( xyz2enu(satXYZ, nomXYZ ) );
  
      // Generate and add pseudorange factor
      PseudorangeFactor pseudorangeFactor(Symbol('x',currKey) , (range - rho),
      obsMap(satNED, nomNED, trop), satXYZ, diagNoise::Sigmas( (Vector(1)
      << measWeight).finished() ));

      graph.add(pseudorangeFactor);

      // Add max-mixture pseudorange factor to seperate graph
      PseudorangeMaxMix maxMixFactor(Symbol('x',currKey) , (range - rho),
      obsMap(satNED, nomNED, trop), hypothesis, null_model, measWeight, weight);

      graphMaxMix.add(maxMixFactor);

      satIndexLiteral.push_back(to_string(currKey) + " " + to_string(svn));
    }

    // Initial Optimization of Graph
    LevenbergMarquardtParams params;
    params.maxIterations = initIter;

    Values result = LevenbergMarquardtOptimizer(graph, initialEstimate, params).optimize();

    NonlinearFactorGraph masterGraph = graph;

    try {
        string textOut = outputFile;
        string switchOut = outputFile;
        string graphOut = outputFile;
        string enuSol = "initialEnu.sol";
        textOut += textExtension;
        switchOut += switchExtension;
        graphOut += graphExtension;
        if (writeGraph) { ofstream os(graphOut); masterGraph.saveGraph(os,result);}
        if (writeENU) { ofstream os(enuSol); writeNavFrame( result, nomXYZ, enuSol ); }
        writeStates( result, textOut );
    }
    catch(std::exception& e) { cout << e.what() << endl; exit(1); }

    // add optimized reults to max-mix graph
    int k = 0;
    gnssStateVec prevState(0,0,0,0,0);
		Values::ConstFiltered<gnssStateVec> result_states = result.filter<gnssStateVec>();
		foreach (const Values::ConstFiltered<gnssStateVec>::KeyValuePair& key_value, result_states)
		{
      if ( key_value.key > lastStep ) { break; }
      gnssStateVec optimizedState = key_value.value;
      graphMaxMix.add(PriorFactor<gnssStateVec>(Symbol('x',k), optimizedState,
        priorNoise));
    }

    Values optimizedResults = LevenbergMarquardtOptimizer(graphMaxMix, result).optimize();

    try {
        // Render to PDF using "fdp pseudorange.dot -Tpdf > graph.phf"
        string optimizedGraph = "finalGraph.dot";
        string resultString = "finalResults.txt";
        string enuSol = "enu.sol";
        string ecefSol = "ecef.sol";
        writeStates( optimizedResults, resultString );
        vector<double> residuals = getResiduals(nomXYZ, optimizedResults, data);
        writeResiduals( residuals, residualTxtOut, satIndexLiteral );
        if (writeENU) { ofstream os(enuSol); writeNavFrame( optimizedResults, nomXYZ, enuSol ); }
        if (writeGraph){ ofstream os(optimizedGraph); graphMaxMix.saveGraph(os,optimizedResults); }
        if (writeECEF) { ofstream os(ecefSol); writeEarthFrame( optimizedResults, nomXYZ, ecefSol ); }
      }
    catch(std::exception& e) { cout << e.what() << endl; exit(1); }

  return 0;
}
