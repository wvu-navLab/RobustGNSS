/**
 *  @file   switchFactorExample.cpp
 *  @author Ryan Watson & Jason Gross
 *  @brief  Factor graph to process GNSS data. This example only handles pseudorange

    Current states estimated ::

            {   delta Pos  }
        X = {   Clk Bias   }
            {  zenith Trop }

 *  Ex:

 * Running
        ./examples/switchFactorExample -i data.txt --writeENU --dir example1
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
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/robustModels/PseudorangeMaxMix.h>
#include <gtsam/gnssNavigation/PseudorangeFactor.h>
#include <gtsam/robustModels/PseudorangeSwitchFactor.h>
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

        bool noTrop, writeSwitch, switchFactorWBetween, writeGraph, elWeight, using_fs=false;
        bool writeENU, writeECEF, writeBias, loose, tight, robust = false, first_ob = true;
        int currKey=-1, trop=1, startEpoch=0, satKeyPrev=-1, sc=1, nThreads, startKey;
        int num_gps_factors=0, factorCount=0, lastStep, firstStep, initIter;
        double measWeight, switchInit, switchPrior;
        double residualThresh, processScale(1.0), timeStep(0.1), prevTime, percentFaulty;
        string gnssFile, outputFile, residualTxtInit="initResidaul.txt";
        string residualTxtOut="finalResidual.txt",textExtension=".txt", strategy;
        string switchExtension = "Switch.txt", graphExtension=".dot", dir;
        vector<string> satIndexLiteral;
        vector<rnxData> data;
        vector<int> numFactors;

        cout.precision(10);
        // define std out print color
        const string red("\033[0;31m");
        const string green("\033[0;32m");
        const string lineBreak = "###########################\n";

        NonlinearFactorGraph graph;

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
                ("initIter",po::value<int>(&initIter)->default_value(100),
                "Number of iterations before initial postfit data edit")
                ("dir", po::value<string>(&dir)->default_value(""),
                "Total path to store generated data")
                ("elWeight,el", "Elevation angle dependant measuremnt weighting")
                ("residualThresh,res", po::value<double>(&residualThresh)->default_value(15.0), "Residual threshold to mark outliers")
                ("switchInit", po::value<double>(&switchInit)->default_value(1.0),
                "Inital switchable constraint value")
                ("percentFaulty", po::value<double>(&percentFaulty)->default_value(0.0),
                "Percentage of observations to add faults. (scale [0,1])")
                ("switchPrior", po::value<double>(&switchPrior)->default_value(0.1),
                "Initial Uncertainty in the switchable constraint")
                ("measWeight", po::value<double>(&measWeight)->default_value(3.0),
                "Noise applied to each GNSS observable")
                ("writeGraph",
                "Write graph to text file. Do not write large graphs (i.e. Nodes>=100)")
                ("writeECEF", "write ecef solution to file")
                ("writeENU", "write enu solution to file")
                ("writeSwitch", "write the switch values to file")
        ;

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        po::notify(vm);

        writeGraph = (vm.count("writeGraph")>0);
        writeENU = (vm.count("writeENU")>0);
        writeECEF = (vm.count("writeECEF")>0);
        writeBias = (vm.count("writeBias")>0);
        writeSwitch = (vm.count("writeSwitch")>0);
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
        Point3 nomNED(0.0, 0.0, 0.0);
        Point3 nomXYZ(856295.3346, -4843033.4111, 4048017.6649);

        double output_time = 0.0;
        double rangeWeight = pow(measWeight,2);

        nonBiasStates initEst((gtsam::Vector(5) << 0,0,0,0,0).finished());

        Values initial_values;
        using symbol_shorthand::X; // nonBiasStates ( dx, dy, dz, trop, cb )
        using symbol_shorthand::S; // switch factor

        noiseModel::Diagonal::shared_ptr nonBias_InitNoise = noiseModel::Diagonal::Sigmas((gtsam::Vector(5) << 30.0, 30.0, 30.0, 1e3, 1e-1).finished());

        noiseModel::Diagonal::shared_ptr nonBias_ProcessNoise = noiseModel::Diagonal::Sigmas((gtsam::Vector(5) << 3.0, 3.0, 3.0, 10, 1e-3).finished());

        SharedNoiseModel switchPriorModel = noiseModel::Diagonal::Sigmas( (Vector(1) << switchPrior ).finished() );
        // Read GNSS data
        try { // data = readGNSS(gnssFile);
                data = readGNSSFaulty(gnssFile, 0.0, 20.0, percentFaulty);
        }
        catch(std::exception& e)
        {
                cout << red << "\n\n Cannot read GNSS data file " << endl;
                exit(1);
        }

        strategy = "GNSS Only using L2 Optimization";
        cout << green << "\n\n" << lineBreak << " GNSS Data File :: "
             << gnssFile << endl;
        cout << "\n Processing Strategy  :: "  << strategy << endl;
        cout << lineBreak << endl;

        if ( firstStep != 0 ) {
                using_fs = true;
                for( unsigned int i = 0; i < data.size(); i++ ) {
                        if ( firstStep == get<1>(data[i]) ) { break; }
                        ++startEpoch;
                }
        }
        if ( lastStep < 0 ) { lastStep = get<0>(data.back()); }

        // Construct Graph --- Only Pseudorange factors
        for(unsigned int i = startEpoch; i < data.size()-1; i++ ) {

                // Get the current epoch's observables
                double gnssTime = get<0>(data[i]);
                int currKey = get<1>(data[i]);
                if (first_ob || using_fs) {
                        startKey = currKey; first_ob=false; using_fs=false;
                        graph.add(PriorFactor<nonBiasStates>(X(currKey), initEst,  nonBias_InitNoise));
                        //++factorCount;
                        initial_values.insert(X(currKey), initEst);
                }
                int nextKey = get<1>(data[i+1]);
                int svn = get<2>(data[i]);
                Point3 satXYZ = get<3>(data[i]);
                double dTrop = deltaTrop(satXYZ,nomXYZ);
                double rho = get<4>(data[i]);
                double range = get<5>(data[i]);


                // Add constraint for pseudorange observable
                PseudorangeFactor gpsFactor(X(currKey), (range - rho), satXYZ, nomXYZ, diagNoise::Sigmas( (gtsam::Vector(1) << elDepWeight(satXYZ, nomXYZ, rangeWeight)).finished()));

                graph.add(gpsFactor);
                numFactors.push_back(++factorCount);
                satIndexLiteral.push_back(to_string(currKey) + " " + to_string(svn));

                // Add prior and between factors
                if ( currKey != nextKey ) {
                        if ( lastStep == nextKey ) { break; }
                        if ( currKey > startKey ) {
                                double delta = gnssTime - prevTime;
                                noiseModel::Diagonal::shared_ptr nonBias_ProcessNoise = noiseModel::Diagonal::Sigmas((gtsam::Vector(5) << 3.0*delta, 3.0*delta, 3.0*delta, 10*delta, 1e-3*delta).finished());
                        }
                        initial_values.insert(X(nextKey),initEst);
                        graph.add(BetweenFactor<nonBiasStates>(X(currKey), X(nextKey), initEst, nonBias_ProcessNoise));
                        ++factorCount;
                        prevTime = gnssTime;
                }
        }

        // Optimize the graph
        LevenbergMarquardtParams Initparams;
        Initparams.maxIterations = 2;
        Values result = LevenbergMarquardtOptimizer(graph, initial_values, Initparams).optimize();
        NonlinearFactorGraph masterGraph = graph;

        // Calculate residuals to make switch states
        vector<double> residuals = getResiduals(nomXYZ, result, data);
        writeResiduals( residuals, residualTxtInit, satIndexLiteral );
        vector<int> outlierKeys = markResiduals( residuals, residualThresh );

        // Switch State Loop
        // Only add switch states to observables with residual greater than threshold
        for(unsigned int i = 0; i < outlierKeys.size(); i++ ) {

                int dataIndex = outlierKeys[i];
                int factorIndex = numFactors[dataIndex];
                int currKey = get<1>(data[dataIndex]);
                Point3 satXYZ = get<3>(data[dataIndex]);
                double rho = get<4>(data[dataIndex]);
                double range = get<5>(data[dataIndex]);

                // Add switchable prior factor for pseudorange observable
                boost::shared_ptr<PriorFactor <SwitchVariableLinear> > switchPrior(
                        new PriorFactor<SwitchVariableLinear>(S(i), SwitchVariableLinear(switchInit), switchPriorModel));

                // Add switchable constraint for pseudorange observable
                PseudorangeSwitchFactor gpsSwitchFactor(X(currKey),S(i), (range - rho), satXYZ, nomXYZ, diagNoise::Sigmas( (gtsam::Vector(1) << elDepWeight(satXYZ, nomXYZ, rangeWeight)).finished()));

                // add init est. for switchable factor
                initial_values.insert(S(i),SwitchVariableLinear(switchInit));
                // add pseudorange switch factor prior
                graph.replace(factorIndex, switchPrior);
                // add pseudorange switchable factor
                graph.add(gpsSwitchFactor);
        }
        // Optimize the graph
        LevenbergMarquardtParams params;
        params.maxIterations = initIter;
        Values optResult = LevenbergMarquardtOptimizer(graph, initial_values, params).optimize();

        try {
                // Render to PDF using "fdp pseudorange.dot -Tpdf > graph.phf"
                string enuSol = "enu.sol";
                string ecefSol = "ecef.sol";
                string optimizedGraph = "finalGraph.dot";
                string resultString = "finalResults.txt";
                string switchValues = "switchValues.txt";
                writeStates( optResult, resultString );
                vector<double> residuals = getResiduals(nomXYZ, optResult, data);
                writeResiduals( residuals, residualTxtOut, satIndexLiteral );
                if (writeENU) { ofstream os(enuSol); writeNavFrame( optResult, nomXYZ, enuSol ); }
                if (writeGraph) { ofstream os(optimizedGraph); graph.saveGraph(os,optResult); }
                if (writeECEF) { ofstream os(ecefSol); writeEarthFrame(optResult,nomXYZ,ecefSol); }
                if (writeSwitch) {writeSwitches( optResult, switchValues, satIndexLiteral ); }
        }
        catch(std::exception& e) { cout << e.what() << endl; exit(1); }
        return 0;
}
