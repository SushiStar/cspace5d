/*
 * This is a environment in continuous space. Based on sbpl library environment template.
 *
 * It's used for planning in (x,y,z,yaw) space. Roll and pitch are assumed to be 0.
 * Robot  is assumed to be a point robot.
 *
 * Need another map configuration file to describe the environment:
 * Height, width, length, resolution etc;
 *
 * Date: Jun/4/2018
 * Author: Wei Du
 *
 * Version: 1.0
 * This version dose not require preprocessed motion primitive file.
 * The successors are generated from actions(which are described by formulations) onlion.
 *
 */

#include <cmath>
#include <cstring>
#include <ctime>
#include <set>
#include <iterator>
#include <fstream>
#include <iostream>

// sbpl config
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <contxyzyaw/environment_xyzyaw.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

// smpl config  3dbfs && stl_reader
#include <smpl/geometry/voxelize.h>
#include <contxyzyaw/stl_reader.h>

#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
static clock_t time_neighbor = 0;
#endif


#if !defined(MAX)
#define MAX(A,B) ( (A) > (B) ? (A):(B) )
#endif

#if !defined(MIN)
#define MIN(A,B) ( (A) < (B) ? (A):(B) )
#endif

static long int checks = 0;
static int numofNewSuccs = 0;

double Sqrt(double x) {
    unsigned int i = *(unsigned int *) &x;
    i += 127 << 23;
    i >>= 1;
    return *(double*) &i;
}


// sorting pairs
bool sortbysec( const std::pair<size_t, double> &a,
        const std::pair<size_t, double> &b ){
    return (a.second < b.second);
}


// state hashentry setup
std::size_t hashkey(double x_, double y_, double z_, double yaw_,
        double vxy_, double vz_, double omega_){

    // to specify the precision
    std::size_t x, y, z, yaw, vxy, vz, omega;
    /*
     *x = (int)(std::round(x_ * 10003.0));
     *y = (int)(std::round(y_ * 10001.0));
     *z = (int)(std::round(z_ * 10007.0));
     *yaw = (int)(std::round(yaw_ * 10037.0));
     *vxy = (int)(std::round(vxy_ * 10000.0));
     */
    x = boost::hash_value(x_);
    y = boost::hash_value(y_);
    z = boost::hash_value(z_);
    yaw = boost::hash_value(yaw_);
    vxy = boost::hash_value(vxy_);

    std::size_t seed = 11;
    /*
     *boost::hash_combine(seed, x*2654435761);
     *boost::hash_combine(seed, y*2654435761);
     *boost::hash_combine(seed, z*2654435761);
     *boost::hash_combine(seed, yaw*2654435761);
     *boost::hash_combine(seed, vxy*2654435761);
     */
    boost::hash_combine(seed, x);
    boost::hash_combine(seed, y);
    boost::hash_combine(seed, z);
    boost::hash_combine(seed, yaw);
    boost::hash_combine(seed, vxy);

    return seed;
}


EnvironmentCONTXYZYAW::EnvironmentCONTXYZYAW(){
    EnvCONTXYZYAWCfg.obsthresh = CNTSPACE_DEFAULTOBSTHRESH; // the value that pretty much makes it disabled
    EnvCONTXYZYAWCfg.cost_inscribed_thresh = EnvCONTXYZYAWCfg.obsthresh;
    // the value that pretty much maeks it disabled
    EnvCONTXYZYAWCfg.cost_possibly_circumscribed_thresh = -1;

    iteration = 0;
    bucketsize = 0; //fixed bucket size
    blocksize = 1;

    EnvCONTXYZYAW.bInitialized = false;

    EnvCONTXYZYAWCfg.actionwidth = CNTSPACE_DEFAULT_ACTIONWIDTH;

    EnvCONTXYZYAWCfg.NumYawDirs = CONTXYZYAW_YAWDIRS;

    // no memory allocated in cfg yet
    EnvCONTXYZYAWCfg.ActionsV = nullptr;
    EnvCONTXYZYAWCfg.PredActionsV = nullptr;
    bfsSearch = nullptr;
    kdtree = nullptr;
    EnvAction = nullptr;
    NNtable = nullptr;

    std::srand(time(nullptr));

    output.open("debug_state.txt");

}   // constructor

EnvironmentCONTXYZYAW::~EnvironmentCONTXYZYAW(){
    SBPL_PRINTF("destorying XYZYAW\n");

    // delete actions
    if( EnvCONTXYZYAWCfg.ActionsV != nullptr ){
  
            delete[] EnvCONTXYZYAWCfg.ActionsV[tind];
        }
        delete[] EnvCONTXYZYAWCfg.ActionsV;
        EnvCONTXYZYAWCfg.ActionsV = nullptr;
    }
    if( EnvCONTXYZYAWCfg.PredActionsV != nullptr ){
        delete[] EnvCONTXYZYAWCfg.PredActionsV;
        EnvCONTXYZYAWCfg.PredActionsV = nullptr;
    }

    output.close();

    if(bfsSearch != nullptr){
        delete bfsSearch;
        bfsSearch =  nullptr;
    }

    if( nullptr != kdtree ){
        delete kdtree;
    }

}       // destructor 


// not having a motion primitive file in this environment.
bool EnvironmentCONTXYZYAW::InitializeEnv(const char* cfgEnvFile,  const char* stlEnvFile){

    // sbpl error handling;
    SBPL_INFO("InitializeEnv start: cfgEnvFile=%s  stlEnvFile=%s\n", cfgEnvFile,  stlEnvFile);
    fflush(stdout);
    FILE* fCfg1 = fopen(cfgEnvFile, "r");
    FILE* fCfg2 = fopen(stlEnvFile, "r");
    if (nullptr == fCfg1){
        std::stringstream ss;
        ss << "ERROR: unable to open " << cfgEnvFile;
        throw SBPL_Exception(ss.str());
    }
    fclose(fCfg1);

    if (nullptr == fCfg2){
        std::stringstream ss;
        ss << "ERROR: unable to open " << stlEnvFile;
        throw SBPL_Exception(ss.str());
    }
    fclose(fCfg2);

    // Read in envrionment cfg file
    ReadinEnvParam(cfgEnvFile);

    //Read in file function.
    ReadConfiguration(stlEnvFile);

    // Initialize bfs class
    InitGeneral();


    if( EnvCONTXYZYAWCfg.StartYaw < 0 || 
            EnvCONTXYZYAWCfg.StartYaw >= 2 * M_PI ){
        throw new SBPL_Exception("ERROR: illegal start coordiantes for yaw");
    }
    if( EnvCONTXYZYAWCfg.EndYaw < 0 || 
            EnvCONTXYZYAWCfg.EndYaw >= 2 * M_PI ){
        throw new SBPL_Exception("ERROR: illegal end coordiantes for yaw");
    }


    SBPL_PRINTF("size of env: %d  by %d by %d\n", EnvCONTXYZYAWCfg.EnvLength, EnvCONTXYZYAWCfg.EnvWidth, EnvCONTXYZYAWCfg.EnvHeight);
    return true;

}   // InitializeEnv


void EnvironmentCONTXYZYAW::InitializeEnvironment(){
    EnvCONTXYZYAWHashEntry_t* HashEntry;

    std::cout<<"Length: " << EnvCONTXYZYAWCfg.EnvLength << " Width:  "<<EnvCONTXYZYAWCfg.EnvWidth
        <<" Height: "<< EnvCONTXYZYAWCfg.EnvHeight << std::endl;

    SBPL_PRINTF("environment stores states in hash table");

    StateID2CoordTable.clear();


    // kdtree
    successors.StateID2CoordTable = &StateID2CoordTable;
    kdtree = new kd_tree_t(4 /*dim*/, successors, nanoflann::KDTreeSingleIndexAdaptorParams(10) );


    // create start state
    if( nullptr == (HashEntry = this->GetHashEntry(
                    EnvCONTXYZYAWCfg.StartX,
                    EnvCONTXYZYAWCfg.StartY,
                    EnvCONTXYZYAWCfg.StartZ,
                    EnvCONTXYZYAWCfg.StartYaw,
                    EnvCONTXYZYAWCfg.StartVXY,
                    EnvCONTXYZYAWCfg.StartVZ,
                    EnvCONTXYZYAWCfg.StartW)) ){

        HashEntry = CreateNewHashEntry(
                EnvCONTXYZYAWCfg.StartX,
                EnvCONTXYZYAWCfg.StartY,
                EnvCONTXYZYAWCfg.StartZ,
                EnvCONTXYZYAWCfg.StartYaw,
                EnvCONTXYZYAWCfg.StartVXY,
                EnvCONTXYZYAWCfg.StartVZ,
                EnvCONTXYZYAWCfg.StartW );

    }
    EnvCONTXYZYAW.startstateid = HashEntry->stateID;

    // create goal state
    if( nullptr == (HashEntry = this->GetHashEntry(
                    EnvCONTXYZYAWCfg.EndX,
                    EnvCONTXYZYAWCfg.EndY,
                    EnvCONTXYZYAWCfg.EndZ,
                    EnvCONTXYZYAWCfg.EndYaw,
                    EnvCONTXYZYAWCfg.EndVXY,
                    EnvCONTXYZYAWCfg.EndVZ,
                    EnvCONTXYZYAWCfg.EndW)) ){

        HashEntry = CreateNewHashEntry(
                EnvCONTXYZYAWCfg.EndX,
                EnvCONTXYZYAWCfg.EndY,
                EnvCONTXYZYAWCfg.EndZ,
                EnvCONTXYZYAWCfg.EndYaw,
                EnvCONTXYZYAWCfg.EndVXY,
                EnvCONTXYZYAWCfg.EndVZ,
                EnvCONTXYZYAWCfg.EndW );
    }
    EnvCONTXYZYAW.goalstateid = HashEntry->stateID;

    EnvCONTXYZYAW.bInitialized = true;

}   // InitializeEnvironment

void EnvironmentCONTXYZYAW::InitializeEnvConfig(){
    EnvAction = new AirplaneAction();
    EnvAction->InitializeAction();
    EnvCONTXYZYAWCfg.nominalvel_mpersecs = EnvAction->GetMaxSpeed();
}

bool EnvironmentCONTXYZYAW::InitGeneral(){
    // Initializeaction
    InitializeEnvConfig(); 

    // Initialize Environment
    InitializeEnvironment();

    // precompute heuristics
    //ComputeHeuristicValues();

    return true;
}


void EnvironmentCONTXYZYAW::ComputeHeuristicValues(){
    SBPL_PRINTF("Precomputing heuristics...\n");


    // put all obstacles into cell
    for(auto vertex:walls){
        bfsSearch->setWall( vertex.x(), vertex.y(),  vertex.z() ); 
    }

    int goalx, goaly, goalz;
    goalx = CONTXY2DISC(EnvCONTXYZYAWCfg.EndX, EnvCONTXYZYAWCfg.cellsize_m);
    goaly = CONTXY2DISC(EnvCONTXYZYAWCfg.EndY, EnvCONTXYZYAWCfg.cellsize_m);
    goalz = CONTXY2DISC(EnvCONTXYZYAWCfg.EndZ, EnvCONTXYZYAWCfg.cellsize_m);

    bfsSearch->run(goalx, goaly, goalz);

    // int getDistance(int x, int y, intz);

    SBPL_PRINTF("done\n");
}

// read in configuration parameters of environmet
void EnvironmentCONTXYZYAW::ReadinEnvParam(const char* cfgEnvFile){
    // read inthe configuration of environment and initialize
    // EnvNAVXYTHETACCfg structure

    char sTemp[1024], sTemp1[1024];
    unsigned char dTemp;
    int x, y, z;

    FILE* fCfg = fopen(cfgEnvFile, "r");

    // discretization(cells)
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
    }
    strcpy(sTemp1, "discretization(cells):");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format (discretization)" <<
            " Expected " << sTemp1 << " got " << sTemp;
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
    }
    EnvCONTXYZYAWCfg.EnvLength = atoi(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
    }
    EnvCONTXYZYAWCfg.EnvWidth = atoi(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early (discretization)");
    }
    EnvCONTXYZYAWCfg.EnvHeight = atoi(sTemp);

    //cellsize
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    strcpy(sTemp1, "cellsize(kilometers):");
    if (strcmp(sTemp1, sTemp) != 0) {
        std::stringstream ss;
        ss << "ERROR: configuration file has incorrect format" <<
            " Expected " << sTemp1 << " got " << sTemp;
        throw SBPL_Exception(ss.str());
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.cellsize_m = atof(sTemp); 

    // start(meters,rads):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.StartX = atof(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.StartY = atof(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.StartZ = atof(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.StartYaw = atof(sTemp);

    if (EnvCONTXYZYAWCfg.StartX < 0 ||
            EnvCONTXYZYAWCfg.StartX >= EnvCONTXYZYAWCfg.EnvLength/EnvCONTXYZYAWCfg.cellsize_m){
        throw SBPL_Exception("ERROR: illegal start coordinates");
    }

    if (EnvCONTXYZYAWCfg.StartY < 0 ||
            EnvCONTXYZYAWCfg.StartY >= EnvCONTXYZYAWCfg.EnvWidth/EnvCONTXYZYAWCfg.cellsize_m){
        throw SBPL_Exception("ERROR: illegal start coordinates");
    }

    if (EnvCONTXYZYAWCfg.StartZ < 0 ||
            EnvCONTXYZYAWCfg.StartZ >= EnvCONTXYZYAWCfg.EnvHeight/EnvCONTXYZYAWCfg.cellsize_m){
        throw SBPL_Exception("ERROR: illegal start coordinates");
    }


    // end(meters,rads):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.EndX = atof(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.EndY = atof(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.EndZ = atof(sTemp);

    if (fscanf(fCfg, "%s", sTemp) != 1) {
        throw SBPL_Exception("ERROR: ran out of env file early");
    }
    EnvCONTXYZYAWCfg.EndYaw = atof(sTemp);


    if (EnvCONTXYZYAWCfg.EndX < 0 ||
            EnvCONTXYZYAWCfg.EndX >= (double)EnvCONTXYZYAWCfg.EnvLength/EnvCONTXYZYAWCfg.cellsize_m){
        throw SBPL_Exception("ERROR: illegal end coordinates");
    }

    if (EnvCONTXYZYAWCfg.EndY < 0 ||
            EnvCONTXYZYAWCfg.EndY >= (double)EnvCONTXYZYAWCfg.EnvWidth/EnvCONTXYZYAWCfg.cellsize_m){
        throw SBPL_Exception("ERROR: illegal end coordinates");
    }

    if (EnvCONTXYZYAWCfg.EndZ < 0 ||
            EnvCONTXYZYAWCfg.EndZ >= (double)EnvCONTXYZYAWCfg.EnvHeight/EnvCONTXYZYAWCfg.cellsize_m){
        throw SBPL_Exception("ERROR: illegal end coordinates");
    }


    EnvCONTXYZYAWCfg.nominalvel_mpersecs = 1.0;

    bfsSearch = new sbpl::motion::BFS_3D(EnvCONTXYZYAWCfg.EnvLength+1, EnvCONTXYZYAWCfg.EnvWidth+1, EnvCONTXYZYAWCfg.EnvHeight+1);

    //NNtable = new std::vector<EnvCONTXYZYAWHashEntry_t*>**[EnvCONTXYZYAWCfg.EnvWidth+1];
    //for(int i = 0; i<= EnvCONTXYZYAWCfg.EnvWidth; ++i){
    //NNtable[i] = new std::vector<EnvCONTXYZYAWHashEntry_t*>* [EnvCONTXYZYAWCfg.EnvLength + 1];
    //for(int j = 0; j<= EnvCONTXYZYAWCfg.EnvLength; ++j){
    //NNtable[i][j] = new std::vector<EnvCONTXYZYAWHashEntry_t*> [EnvCONTXYZYAWCfg.EnvHeight];
    //}
    //}

    fclose(fCfg);

}   // ReadinEnvParam


// Read an stl file and convert it into voxels based on the environment resolution
// environment description file should be read first before readin this file
// if the resolution is too small, then too much voxels would be generated
// causing std::bad_alloc error
void EnvironmentCONTXYZYAW::ReadConfiguration( const char* stlEnvFile){


    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::uint32_t> indices;
    std::vector<Eigen::Vector3d> voxels;
    vertices.clear();
    indices.clear();
    voxels.clear();
    walls.clear();

    // load stl file
    std::vector<double> coords, normals;
    std::vector<std::uint32_t> tris, solids;

    try{
        stl_reader::ReadStlFile(stlEnvFile, coords, normals, tris, solids);
        const size_t numTris = tris.size()/3;

        for(size_t itri = 0; itri < numTris; ++itri){
            indices.push_back((std::uint32_t)itri*3 + 0);
            indices.push_back((std::uint32_t)itri*3 + 1);
            indices.push_back((std::uint32_t)itri*3 + 2);

            for(size_t icorner = 0; icorner < 3; ++icorner){
                double* c = &coords[3*tris[3*itri + icorner]];

                Eigen::Vector3d currentVertex( (double)c[0],(double)c[1], (double)c[2] );
                vertices.push_back(currentVertex);
            }
        }

    }catch (std::exception& e){
        std::cout << e.what() << std::endl;
    }

    // voxelize the mesh
    bool fill = false;
    sbpl::geometry::VoxelizeMesh(vertices, indices, EnvCONTXYZYAWCfg.cellsize_m, voxels, fill);

    // convert voxels in continuous space to discrete space    once and for all
    int x,y,z;
    int tempx, tempy, tempz;
    for(auto point:voxels){
        x = CONTXY2DISC(point.x(), EnvCONTXYZYAWCfg.cellsize_m);
        y = CONTXY2DISC(point.y(), EnvCONTXYZYAWCfg.cellsize_m);
        z = CONTXY2DISC(point.z(), EnvCONTXYZYAWCfg.cellsize_m);
        if(z < 0) z = 0;

        //Eigen::Vector3i pt(x, y, z);
        //walls.insert(pt);

        // thicken the wall
        for(int i = -1; i<2; ++i){
            tempx = x +i;
            if(tempx < 0) continue;
            for(int j = -1; j<2; ++j){
                tempy = y +j;
                if(tempy < 0) continue;
                for(int k = -3; k<2; ++k){
                    tempz = z +k;
                    if(tempz < 0)continue;
                    Eigen::Vector3i pt(tempx, tempy, tempz);
                    walls.insert(pt);
                }
            }
        }
    }
}   // ReadConfiguration


int EnvironmentCONTXYZYAW::SetStart(double x_m, double y_m, double z_m, double yaw_r,
        double vxy, double vz, double w){

    if(!IsWithinMapCell(x_m,y_m,z_m)){
        SBPL_ERROR("ERROR: trying to set a start cell %.3f %.3f %.3f that is outside of map\n",x_m , y_m, z_m);
        return -1;
    }
    SBPL_PRINTF("env: setting start to %.3f %.3f %.3f %.3f \n", x_m, y_m, z_m, yaw_r);

    if( !IsValidCell(x_m, y_m, z_m ) ){
        SBPL_PRINTF("WARNING: start configuration %.3f %.3f %.3f is invalid\n", x_m, y_m, z_m, yaw_r);
    }


    EnvCONTXYZYAWHashEntry_t* OutHashEntry;
    if(nullptr == (OutHashEntry = GetHashEntry(x_m, y_m, z_m, yaw_r,vxy,vz,w)) ){
        printf("creating new start.\n");
        OutHashEntry=CreateNewHashEntry(x_m, y_m, z_m, yaw_r,vxy,vz,w);
        OutHashEntry->parentStateID = OutHashEntry->stateID;
        OutHashEntry->valid_children_ratio = 1;
    }

    if(EnvCONTXYZYAW.startstateid != OutHashEntry->stateID ){
        bNeedtoRecomputeStartHeuristics = true;
        // because termination condition can be not all states TODO - make it dependent on term. condition
        bNeedtoRecomputeGoalHeuristics = true;
    }

    // set start
    EnvCONTXYZYAW.startstateid = OutHashEntry->stateID;
    EnvCONTXYZYAWCfg.StartX = x_m;
    EnvCONTXYZYAWCfg.StartY = y_m;
    EnvCONTXYZYAWCfg.StartZ = z_m;
    EnvCONTXYZYAWCfg.StartYaw = yaw_r;
    EnvCONTXYZYAWCfg.StartVXY = vxy;
    EnvCONTXYZYAWCfg.StartVZ = vz;
    EnvCONTXYZYAWCfg.StartW = w;

    return EnvCONTXYZYAW.startstateid;

}   // SetStart


int EnvironmentCONTXYZYAW::SetGoal(double x_m, double y_m, double z_m, double yaw_r,
        double vxy, double vz, double w){

    if(!IsWithinMapCell(x_m,y_m,z_m)){
        SBPL_ERROR("ERROR: trying to set a goal cell %.3f %.3f %.3f that is outside of map\n",x_m , y_m, z_m);
        return -1;
    }
    SBPL_PRINTF("env: setting goal to %.3f %.3f %.3f %.3f \n", x_m, y_m, z_m, yaw_r);

    if( !IsValidCell(x_m, y_m, z_m) ){
        SBPL_PRINTF("WARNING: goal configuration %.3f %.3f %.3f is invalid\n", x_m, y_m, z_m, yaw_r);
    }


    EnvCONTXYZYAWHashEntry_t* OutHashEntry;
    if(nullptr == (OutHashEntry = GetHashEntry(x_m, y_m, z_m, yaw_r,vxy,vz,w)) ){
        printf("creating new goal.\n");
        OutHashEntry=CreateNewHashEntry(x_m, y_m, z_m, yaw_r,vxy,vz,w);
        OutHashEntry->parentStateID = OutHashEntry->stateID;
        OutHashEntry->valid_children_ratio = 1;
    }

    if(EnvCONTXYZYAW.goalstateid != OutHashEntry->stateID ){
        // because termination condition may not plan all the way to the new goal
        bNeedtoRecomputeStartHeuristics = true;
        // because goal heuristic changes 
        bNeedtoRecomputeGoalHeuristics = true;
    }

    // set goal 
    //OutHashEntry->parentStateID = -10;
    EnvCONTXYZYAW.goalstateid = OutHashEntry->stateID;
    EnvCONTXYZYAWCfg.EndX = x_m;
    EnvCONTXYZYAWCfg.EndY = y_m;
    EnvCONTXYZYAWCfg.EndZ = z_m;
    EnvCONTXYZYAWCfg.EndYaw = yaw_r;
    EnvCONTXYZYAWCfg.EndVXY = vxy;
    EnvCONTXYZYAWCfg.EndVZ = vz;
    EnvCONTXYZYAWCfg.EndW = w;


    ComputeHeuristicValues();

    //AddVirtualPoints();

    return EnvCONTXYZYAW.goalstateid;

}   // SetGoal

bool EnvironmentCONTXYZYAW::IsWithinMapCell(double x, double y, double z){
    return ( x>= 0 && x < (int)EnvCONTXYZYAWCfg.EnvLength &&
            y>= 0 && y < (int)EnvCONTXYZYAWCfg.EnvWidth &&
            z>= 0 && z < (int)EnvCONTXYZYAWCfg.EnvHeight);  
}

void EnvironmentCONTXYZYAW::EnsureHeuristicsUpdated(bool bGoalHeuristics){

    // backward search
    if(bNeedtoRecomputeStartHeuristics && !bGoalHeuristics){

        int startx, starty, startz;
        startx = CONTXY2DISC(EnvCONTXYZYAWCfg.StartX, EnvCONTXYZYAWCfg.cellsize_m);
        starty = CONTXY2DISC(EnvCONTXYZYAWCfg.StartY, EnvCONTXYZYAWCfg.cellsize_m);
        startz = CONTXY2DISC(EnvCONTXYZYAWCfg.StartZ, EnvCONTXYZYAWCfg.cellsize_m);

        //bfsSearch->run(startx, starty, startz);

        bNeedtoRecomputeStartHeuristics = false;
        //SBPL_PRINTF("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromstart->getlowerboundoncostfromstart_inmm(
        //EnvNAVXYTHETACCfg.StartX_c, EnvNAVXYTHETACCfg.StartY_c) / EnvNAVXYTHETACCfg.nominalvel_mpersecs));
    }

    // forwardsearch
    if(bNeedtoRecomputeGoalHeuristics && bGoalHeuristics){

        int goalx, goaly, goalz;
        goalx = CONTXY2DISC(EnvCONTXYZYAWCfg.EndX, EnvCONTXYZYAWCfg.cellsize_m);
        goaly = CONTXY2DISC(EnvCONTXYZYAWCfg.EndY, EnvCONTXYZYAWCfg.cellsize_m);
        goalz = CONTXY2DISC(EnvCONTXYZYAWCfg.EndZ, EnvCONTXYZYAWCfg.cellsize_m);

        //bfsSearch->run(goalx, goaly, goalz);

        bNeedtoRecomputeGoalHeuristics = false;
        //SBPL_PRINTF("2dsolcost_infullunits=%d\n", (int)(grid2Dsearchfromgoal->getlowerboundoncostfromstart_inmm(
        //EnvNAVXYTHETACCfg.StartX_c, EnvNAVXYTHETACCfg.StartY_c) / EnvNAVXYTHETACCfg.nominalvel_mpersecs));
    }
}



int EnvironmentCONTXYZYAW::GetFromToHeuristic(int FromStateID, int ToStateID){
    //heuristic is always used 
#if USE_HEUR == 0
    return 0;
#endif

#if DEBUG
    if( FromStateID >= (int)StateID2CoordTable.size() ||
            ToStateID >= (int)StateID2CoordTable.size() ){
        SBPL_ERROR("ERROR in EnvCONTXYZYAW... function: stateID illegal\n");
        throw SBPL_Exception("");
    }
#endif

    // get x,y,z,yaw for the state
    EnvCONTXYZYAWHashEntry_t* FromHashEntry = StateID2CoordTable[FromStateID];
    EnvCONTXYZYAWHashEntry_t* ToHashEntry = StateID2CoordTable[ToStateID];

    // check if one of the gridsearches already computed and then use it.

    return (int)(CONTXYZYAW_COSTMULT_MTOMM *
            EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, FromHashEntry->Z,
                ToHashEntry->X, ToHashEntry->Y, ToHashEntry->Z) /
            EnvCONTXYZYAWCfg.nominalvel_mpersecs);

}   // GetFromToHeuristic 


void EnvironmentCONTXYZYAW::GetCoordFromState(
        int stateID, double& x, double& y, double &z, double &yaw,
        double& vxy, double &vz, double &w) const{
    EnvCONTXYZYAWHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    x = HashEntry->X;
    y = HashEntry->Y;
    z = HashEntry->Z;
    yaw = HashEntry->Yaw;
    vxy = HashEntry->VXY;
    vz = HashEntry->VZ;
    w = HashEntry->W;
}

int EnvironmentCONTXYZYAW::GetStateFromCoord(
        double x, double y, double z, double yaw, double vxy, double vz, double w){
    EnvCONTXYZYAWHashEntry_t* OutHashEntry;
    if( nullptr == (OutHashEntry = GetHashEntry(x,y,z,yaw,vxy,vz,w)) ){
        OutHashEntry = CreateNewHashEntry(x,y,z,yaw,vxy,vz,w);
    }
    return OutHashEntry->stateID;
}

int EnvironmentCONTXYZYAW::GetGoalHeuristic(int stateID){
#if 0==USE_HEUR
    return 0;
#endif

#if DEBUG
    if( stateID >= (int)StateID2CoordTable.size() ){
        throw SBPL_Exception("ERROR in EnvCONTXYZYAW... function: stateID illegal");
    }
#endif

    EnvCONTXYZYAWHashEntry_t* HashEntry = StateID2CoordTable[stateID];
    auto parentEntry = StateID2CoordTable.at(HashEntry->parentStateID);

    int discx, discy, discz;
    discx = CONTXY2DISC(HashEntry->X, EnvCONTXYZYAWCfg.cellsize_m);
    discy = CONTXY2DISC(HashEntry->Y, EnvCONTXYZYAWCfg.cellsize_m);
    discz = CONTXY2DISC(HashEntry->Z, EnvCONTXYZYAWCfg.cellsize_m);

    double inflation = 1;
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

#if PENALTY

    /* nanoflann */
    double query_pt[4] = {HashEntry->X, HashEntry->Y, HashEntry->Z, HashEntry->Yaw};
    const size_t numofneighbors = 20;
    size_t neibIndex[numofneighbors];
    double distance[numofneighbors];

    //double T = 6.0 * parentEntry->valid_children_ratio;
    double T = 5.0;

    nanoflann::KNNResultSet<double> resultSet(numofneighbors);
    resultSet.init(neibIndex, distance);
    //clock_t mm = clock();
    auto found = kdtree->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(0));
    //std::cout<<  (double)(clock() - mm)/CLOCKS_PER_SEC << std::endl;

    if(found){

        for( auto i = 0; i < numofneighbors; ++i ) {
            
            auto neighborEntry = StateID2CoordTable[neibIndex[i]];
            if( neighborEntry->parentStateID == HashEntry->parentStateID || 
                    neighborEntry->stateID == HashEntry->parentStateID) {
                continue;
            } else {

                //double dist = std::sqrt( distance[i]) + 0.007 * std::cos(neighborEntry->Yaw - HashEntry->Yaw); 
                double dist = std::sqrt( distance[i]);
                //std::cout << dist << std::endl;

                double r = EnvCONTXYZYAWCfg.radius*MAX(0.1,parentEntry->valid_children_ratio);

                if (dist > r) {
                    break;
                }
                inflation =  T *(1.00 - dist/r) + 1;
                break;

            }
        }
    }
#endif

#if TIME_DEBUG
    time_neighbor += clock()-currenttime;
#endif

    // bfs heuristic
    double Dh = (double) bfsSearch->getDistance(discx, discy, discz);
    Dh *= 20.0;

    double Eh = 1000* EuclideanDistance_m(HashEntry->X, HashEntry->Y, HashEntry->Z,
            EnvCONTXYZYAWCfg.EndX, EnvCONTXYZYAWCfg.EndY, EnvCONTXYZYAWCfg.EndZ);
    //std::cout << "Dh: " << Dh << "  Eh: "<< Eh << std::endl;

    int h = (int)(MAX(Dh, Eh) * inflation);
    //int h = std::max(Dh, Eh);

    return h;

}   // GetGoalHeuristic


double EnvironmentCONTXYZYAW::EuclideanDistance_m(
        double x1, double y1, double z1, double x2, double y2, double z2){

    double d0 = x1-x2;
    double d1 = y1-y2;
    double d2 = z1-z2;
    double sqdist = d0*d0 + d1*d1 + d2*d2;
    return std::sqrt(sqdist);
}


// when cell (0,0) changes its status it also does the same for the 3D states
// whose incoming actions are potentially affected when cell (0,0) changes its
// status
void EnvironmentCONTXYZYAW::ComputeReplanningData(){}


bool EnvironmentCONTXYZYAW::IsValidCell(double x, double y, double z){
    int discx, discy, discz;
    discx = CONTXY2DISC(x, EnvCONTXYZYAWCfg.cellsize_m);
    discy = CONTXY2DISC(y, EnvCONTXYZYAWCfg.cellsize_m);
    discz = CONTXY2DISC(z, EnvCONTXYZYAWCfg.cellsize_m);

    return ( discx >= 0 && discx < EnvCONTXYZYAWCfg.EnvLength &&
            discy >= 0 && discy < EnvCONTXYZYAWCfg.EnvWidth &&
            discz >= 0 && discz < EnvCONTXYZYAWCfg.EnvHeight  
            && !bfsSearch->isWall(discx, discy, discz)
           );
}

bool EnvironmentCONTXYZYAW::IsValidCell(int discx, int discy, int discz){
    return ( discx >= 0 && discx < EnvCONTXYZYAWCfg.EnvLength &&
            discy >= 0 && discy < EnvCONTXYZYAWCfg.EnvWidth &&
            discz >= 0 && discz < EnvCONTXYZYAWCfg.EnvHeight  
            && !bfsSearch->isWall(discx, discy, discz)
           );

}

// normalizeDiscAngle
EnvCONTXYZYAWHashEntry_t*
EnvironmentCONTXYZYAW::GetHashEntry(double x, double y, double z, double yaw,
        double vxy, double vz, double w){
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    auto key = hashkey(x, y, z, yaw, vxy, vz, w);
    auto search = Coord2StateIDHashTable.find(key);

    if(search != Coord2StateIDHashTable.end()){
        return search->second;
    }

#if TIME_DEBUG
    time_gethash += clock()-currenttime;
#endif

    return nullptr;
}

EnvCONTXYZYAWHashEntry_t*
EnvironmentCONTXYZYAW::CreateNewHashEntry(double x, double y, double z, double yaw,
        double vxy, double vz, double w){
#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    EnvCONTXYZYAWHashEntry_t* HashEntry = new EnvCONTXYZYAWHashEntry_t();
    HashEntry->X = x;
    HashEntry->Y = y;
    HashEntry->Z = z;
    HashEntry->Yaw = yaw;
    HashEntry->VXY = vxy;
    HashEntry->VZ = vz;
    HashEntry->W = w;

    // insert into NNtable
    int discx, discy, discz;
    discx = CONTXY2DISC(x, EnvCONTXYZYAWCfg.cellsize_m);
    discy = CONTXY2DISC(y, EnvCONTXYZYAWCfg.cellsize_m);
    discz = CONTXY2DISC(z, EnvCONTXYZYAWCfg.cellsize_m);

    // row column height
    //NNtable[discy][discx][discz].push_back(HashEntry);


    // create a new id, which is the last elements's index in StateID2CoordTable
    HashEntry->stateID = StateID2CoordTable.size();
    StateID2CoordTable.push_back(HashEntry);

    Coord2StateIDHashTable[hashkey(x,y,z,yaw,vxy,vz,w)] = HashEntry;

    // I do not know what this is @_@
    // insert into initialize the mappings 
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);

    for(int i=0; i<NUMOFINDICES_STATEID2IND; ++i){
        StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }
    if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1){
        throw SBPL_Exception("ERROR in Env... function: last state has incorrect stateID");
    }

#if PENALTY
/*
 *    numofNewSuccs++;
 *
 *    if(numofNewSuccs >= 100){
 *        numofNewSuccs = 0;
 *        // should be put into createnewhashentry
 *        kdtree->addPoints( (size_t)HashEntry->stateID - 100 , (size_t)HashEntry->stateID);
 *    }
 */
    kdtree->addPoints( (size_t)HashEntry->stateID , (size_t)HashEntry->stateID);
#endif

#if TIME_DEBUG
    time_createhash += clock()-currenttime;
#endif

    return HashEntry;

}   // CreateNewHashEntry


void EnvironmentCONTXYZYAW::AddVirtualPoints() {
    // the dimension of kdtree is x, y, z;
    
    std::cout << "add virtual points." << std::endl;


    double x1 = EnvCONTXYZYAWCfg.StartX;
    double x2 = EnvCONTXYZYAWCfg.EndX;
    double X  = (x1-x2);
    //double X  = EnvCONTXYZYAWCfg.EnvLength;
    double y1 = EnvCONTXYZYAWCfg.StartY;
    double y2 = EnvCONTXYZYAWCfg.EndY;
    double Y  = (y1-y2);
    //double Y  = EnvCONTXYZYAWCfg.EnvWidth;
    double z1 = EnvCONTXYZYAWCfg.StartZ;
    double z2 = EnvCONTXYZYAWCfg.EndZ;
    double Z  = (z1-z2);
    //double Z  = EnvCONTXYZYAWCfg.EnvHeight;

    double x, y, z, yaw;
    for(int i = 0; i < 500; ++i) {

        x =  static_cast <double> (rand()) / static_cast <double> (RAND_MAX/X) + x1;
        y =  static_cast <double> (rand()) / static_cast <double> (RAND_MAX/Y) + y1;
        z =  static_cast <double> (rand()) / static_cast <double> (RAND_MAX/Z) + z1;

        auto hashentry = CreateNewHashEntry(x,y,z,yaw, 0.0, 0.0, 0.0);
    }

    std::cout << "add virtual points done." << std::endl;
}


int EnvironmentCONTXYZYAW::GetActionCost(double SourceX, double SourceY, double SourceZ, double SourceYaw, EnvCONTXYZYAWAction_t* action){
}



void EnvironmentCONTXYZYAW::GetSuccs(
        int SourceStateID, 
        std::vector<int> *SuccIDV,
        std::vector<int> *CostV,
        std::vector<EnvCONTXYZYAWAction_t*>* actionV) {

#if TIME_DEBUG
    clock_t currenttime = clock();
#endif

    int aind;

    SuccIDV->clear();
    CostV->clear();

    if(actionV != nullptr){
        actionV->clear();
        actionV->reserve(EnvCONTXYZYAWCfg.actionwidth);
    }

    if(SourceStateID == EnvCONTXYZYAW.goalstateid) return;

    // predcessor state
    EnvCONTXYZYAWHashEntry_t* HashEntry = StateID2CoordTable[SourceStateID];

    std::vector<double> sourceState = {HashEntry->X, HashEntry->Y, HashEntry->Z,
                        HashEntry->Yaw, HashEntry->VXY, HashEntry->VZ, HashEntry->W};

    std::vector<double> endState;
    std::vector<sbpl_xyz_rpy_pt_t> interpts;

    EnvCONTXYZYAWAction_t* xyzyawaction;
    bool validcell=true;
    int cost;                                       // -1 if the action is illegal
    double denominator = 0;

    for (aind = 0; aind < EnvCONTXYZYAWCfg.actionwidth; ++aind) {
        endState.clear();
        EnvAction->GetTrajAndCost(aind, sourceState, endState, interpts, cost);

        if (-1 == cost) {
            continue;
        }
        denominator += 1.0;
        for (auto temp:interpts) {
            if(!IsValidCell( (double)temp.x, (double)temp.y, (double)temp.z )) {
                validcell = false;
                break;
            }
        }

        if(!validcell) {
            validcell = true;
            continue;
        }

        // then its a valid successor

        EnvCONTXYZYAWHashEntry_t* OutHashEntry;
        if (IsWithinGoalRegion(endState[0],endState[1],endState[2],endState[3])){

            OutHashEntry = StateID2CoordTable[EnvCONTXYZYAW.goalstateid];
            OutHashEntry->parentStateID = SourceStateID;

        }else{

            if (nullptr == (OutHashEntry = GetHashEntry(endState[0], endState[1],
                            endState[2], endState[3],
                            endState[4], endState[5],
                            endState[6]) )){

                OutHashEntry = CreateNewHashEntry(endState[0], endState[1],
                        endState[2], endState[3],
                        endState[4], endState[5],
                        endState[6] ); 
                OutHashEntry->parentStateID = SourceStateID;
            }
            
        }

        // all success: new and old;
        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);

        xyzyawaction = new EnvCONTXYZYAWAction_t((unsigned char) aind, sourceState[3], 
                endState[3], cost);
        xyzyawaction->intermptV = std::move(interpts);

        if ( nullptr != actionV){
            actionV->push_back(xyzyawaction);
        }
    }
    HashEntry->valid_children_ratio = (double)SuccIDV->size() / denominator;
    
#if TIME_DEBUG
	time_getsuccs += clock()-currenttime;
#endif

}   // GetSuccs


void EnvironmentCONTXYZYAW::GetSuccs(
        int SourceStateID, 
        std::vector<int>* SuccIDV, 
        std::vector<int>* CostV){
        GetSuccs(SourceStateID, SuccIDV, CostV, nullptr);
}

void EnvironmentCONTXYZYAW::PrintState(int stateID, bool bVerbose, FILE *fOut){
#if DEBUG
    if(stateID >= (int)StateID2CoordTable.size()){
        SBPL_ERROR("ERROR in EnvCONTXYZYAW... function: stateID illegal (2)\n");
        throw SBPL_Exception();
    }
#endif
    

    if(nullptr == fOut){
        fOut = stdout;
    }

    EnvCONTXYZYAWHashEntry_t* HashEntry = StateID2CoordTable[stateID];

    if(stateID == EnvCONTXYZYAW.goalstateid && bVerbose){
        SBPL_FPRINTF(fOut, "the state is a goal state\n");
    }

    if(bVerbose){
        SBPL_FPRINTF(fOut, "X=%f Y=%f Z=%f Yaw=%f VXY=%f VZ=%f W=%f \n", 
        HashEntry->X, HashEntry->Y, HashEntry->Z, HashEntry->Yaw, HashEntry->VXY, HashEntry->VZ, HashEntry->W);

        output<< HashEntry->X <<" "<< HashEntry->Y << " "<< HashEntry->Z << " " << HashEntry->Yaw << std::endl;

    }else{
        SBPL_FPRINTF(fOut, "%.3f %.3f %.3f %.3f\n", HashEntry->X,
                HashEntry->Y, HashEntry->Z, HashEntry->Yaw);
    }

}   // PrintState


void EnvironmentCONTXYZYAW::ConvertStateIDPathintoXYZYawPath(
        std::vector<int>* stateIDPath, std::vector<sbpl_xyz_rpy_pt_t>* xyzyawPath){

    std::vector<EnvCONTXYZYAWAction_t*> actionV;
    std::vector<int> CostV;
    std::vector<int> SuccIDV;

    double targetx_c, targety_c, targetz_c, targetyaw_c, targetvxy_c, targetvz_c, targetw_c;
    double sourcex_c, sourcey_c, sourcez_c, sourceyaw_c, sourcevxy_c, sourcevz_c, sourcew_c;

    SBPL_PRINTF("checks=%ld\n", checks);

    xyzyawPath->clear();

#if DEBUG
    SBPL_FPRINTF(fDeb, "converting stateid path into coordinates:\n");
#endif

    for (int pind = 0; pind < (int)(stateIDPath->size()) - 1; pind++) {
        int sourceID = stateIDPath->at(pind);
        int targetID = stateIDPath->at(pind + 1);

#if DEBUG
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcez_c, 
                sourceyaw_c, sourcevxy_c, sourcevz_c, sourcew_c);
#endif

        // get successors and pick the target via the cheapest action
        SuccIDV.clear();
        CostV.clear();
        actionV.clear();
        GetSuccs(sourceID, &SuccIDV, &CostV, &actionV);

        int bestcost = INFINITECOST;
        int bestsind = -1;

#if DEBUG
        GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcez_c, 
                sourceyaw_c, sourcevxy_c, sourcevz_c, sourcew_c);
        GetCoordFromState(targetID, targetx_c, targety_c, targetz_c,
                targetyaw_c, targetvxy_c, targetvz_c, targetw_c);
        /*
         *if(EuclideanDistance_m(sourcex_c, sourcey_c, sourcez_c, targetx_c, targety_c, targetz_c) > 0.035){
         *    std::cout << "states not continue.\n";
         *    SBPL_FPRINTF(fDeb,"here\n");
         *}
         */

        SBPL_FPRINTF(fDeb, "looking for %.4f %.4f %.4f %.4f %.4f -> %.4f %.4f %.4f %.4f %.4f (numofsuccs=%d)\n", sourcex_c, sourcey_c, sourcez_c, sourceyaw_c, sourcevxy_c, targetx_c, targety_c, targetz_c, targetyaw_c, targetvxy_c, (int)SuccIDV.size());
#endif

        for (int sind = 0; sind < (int)SuccIDV.size(); sind++) {
#if DEBUG
            double x_c, y_c, z_c, yaw_c, vxy_c, vz_c, w_c;
            GetCoordFromState(SuccIDV[sind], x_c, y_c, z_c, yaw_c, vxy_c, vz_c, w_c);
            SBPL_FPRINTF(fDeb, "succ: %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n", 
                    x_c, y_c, z_c, yaw_c, vxy_c, vz_c, w_c);
#endif
            if (SuccIDV[sind] == targetID && CostV[sind] <= bestcost) {
                bestcost = CostV[sind];
                bestsind = sind;
            }
        }
        if (bestsind == -1) {
            SBPL_ERROR("ERROR: successor not found for transition");
            GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcez_c, 
                    sourceyaw_c, sourcevxy_c, sourcevz_c, sourcew_c);
            GetCoordFromState(targetID, targetx_c, targety_c, targetz_c,
                    targetyaw_c, targetvxy_c, targetvz_c, targetw_c);

            SBPL_PRINTF("%.4f %.4f %.4f -> %.4f %.4f %.4f\n", sourcex_c, sourcey_c, sourcez_c, sourceyaw_c,
                    targetx_c, targety_c, targetz_c, targetyaw_c);
            throw SBPL_Exception("ERROR: successor not found for transition");
        }

        // now push in the actual path
        //GetCoordFromState(sourceID, sourcex_c, sourcey_c, sourcez_c, sourceyaw_c, sourcevxy_c, sourcevz_c, sourcew_c);

        /*
         *double sourcex, sourcey;
         *sourcex = DISCXY2CONT(sourcex_c, EnvNAVXYTHETALATCfg.cellsize_m);
         *sourcey = DISCXY2CONT(sourcey_c, EnvNAVXYTHETALATCfg.cellsize_m);
         */

        // TODO - when there are no motion primitives we should still print source state
        sbpl_xyz_rpy_pt_t intermpt;
        for (int ipind = 0; ipind < ((int)actionV[bestsind]->intermptV.size()) - 1; ipind++) {
            // translate appropriately
            intermpt = actionV[bestsind]->intermptV[ipind];
            //intermpt.x += sourcex;
            //intermpt.y += sourcey;
            //intermpt.z += sourcez;

/*
 *#if DEBUG
 *            int nx = CONTXY2DISC(intermpt.x, EnvNAVXYTHETALATCfg.cellsize_m);
 *            int ny = CONTXY2DISC(intermpt.y, EnvNAVXYTHETALATCfg.cellsize_m);
 *            int ntheta;
 *            ntheta = ContTheta2DiscNew(intermpt.theta);
 *
 *            SBPL_FPRINTF(fDeb, "%.3f %.3f %.3f (%d %d %d cost=%d) ", intermpt.x, intermpt.y, intermpt.theta, nx, ny, ntheta,  EnvNAVXYTHETALATCfg.Grid2D[nx][ny]);
 *
 *            if (ipind == 0) {
 *                SBPL_FPRINTF(fDeb, "first (heur=%d)\n", GetStartHeuristic(sourceID));
 *            }
 *            else {
 *                SBPL_FPRINTF(fDeb, "\n");
 *            }
 *#endif
 */
            // store
            xyzyawPath->push_back(intermpt);
        }

    }
}       // ConvertStateIDPathintoXYZYawPath


int EnvironmentCONTXYZYAW::SizeofCreatedEnv(){
    return (int)StateID2CoordTable.size();
}

void EnvironmentCONTXYZYAW::PrintEnv_Config(FILE* fOut){
    // implement this if the planner needs to print out EnvCONTXYZYAW.configuartion
    throw SBPL_Exception("ERROR in EnvCONTXYZYAW... function: PrintEnv_Config is undefined");
}

void EnvironmentCONTXYZYAW::PrintTimeStat(FILE* fOut){
#if TIME_DEBUG
   SBPL_FPRINTF(fOut, "time_neighbor = %f secs, time_gethash = %f secs, time_createhash = %f secs, "
				"time_getsuccs = %f\n",
				time_neighbor/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC,
				time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
   //printf( "time_neighbor = %f secs, time_gethash = %f secs, time_createhash = %f secs, "
                //"time_getsuccs = %f\n",
                //time_neighbor/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC,
                //time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
//#endif
}

void EnvironmentCONTXYZYAW::PrintHashTableHist(FILE* Out){}

void EnvironmentCONTXYZYAW::SetGoalTolerance(double tol_x, double tol_y, double tol_z, double tol_yaw){
    EnvCONTXYZYAWCfg.tol_x = tol_x;
    EnvCONTXYZYAWCfg.tol_y = tol_y;
    EnvCONTXYZYAWCfg.tol_z = tol_z;
    EnvCONTXYZYAWCfg.tol_yaw = tol_yaw;
}

// meter meter meter rad
bool EnvironmentCONTXYZYAW::IsWithinGoalRegion( double x, double y, double z, double yaw){
    return ( std::abs(x-EnvCONTXYZYAWCfg.EndX) < EnvCONTXYZYAWCfg.tol_x &&
        std::abs(y-EnvCONTXYZYAWCfg.EndY) < EnvCONTXYZYAWCfg.tol_y &&
        std::abs(z-EnvCONTXYZYAWCfg.EndZ) < EnvCONTXYZYAWCfg.tol_z &&
        std::abs(yaw - EnvCONTXYZYAWCfg.EndYaw) < EnvCONTXYZYAWCfg.tol_yaw);
}

/*
 *void EnvironmentCONTXYZYAW::UpdateParent(int childStateID, int parentStateID) {
 *    if (childStateID < StateID2CoordTable.size() && 
 *        parentStateID < StateID2CoordTable.size() ) {
 *        auto child = StateID2CoordTable[childStateID];
 *        child->parentStateID = parentStateID;
 *    }
 *}
 */

void EnvironmentCONTXYZYAW::SetupR(double R) {
    EnvCONTXYZYAWCfg.radius = R;
}
