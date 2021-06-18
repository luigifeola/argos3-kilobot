/**
 * @file <dhtf.h>
 *
 * @author Luigi Feola <feola@diag.uniroma1.it>
 *
 * @brief This is the header file of the dhtf ARK Loop Function (ALF), the simulated counterpart of the ARK (Augmented Reality for Kilobots) system.
 *
 */

#ifndef DHTF_H
#define DHTF_H

namespace argos
{
    class CSpace;
    class CFloorEntity;
    class CSimulator;
}

#include <math.h>
#include <iostream>
#include <time.h>
// #include <sys/types.h>
// #include <unistd.h>
// #include <sys/socket.h>
// #include <unistd.h>
// #include <fcntl.h>
// #include <netdb.h>
// #include <arpa/inet.h>
#include <string.h>
#include <string>
#include <cstring>
#include <cmath>
#include <numeric>
#include <array>
#include <random>
#include <algorithm>
#include <vector>
#include <bitset>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/ALF.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

#include <array>

using namespace argos;

class dhtfCALF : public CALF
{

public:
    dhtfCALF();

    virtual ~dhtfCALF() {}

    virtual void Init(TConfigurationNode &t_tree);

    virtual void Reset();

    virtual void Destroy();

    virtual void PostStep();

    virtual void PostExperiment();

    /** Log area pos, type, state (completed or not) */
    void AreaLOG();

    /** Log Kilobot pose and state */
    void KiloLOG();

    /** Get a Vector of all the Kilobots in the space */
    void
    GetKilobotsEntities();

    /** Setup the initial state of the Kilobots in the space */
    void SetupInitialKilobotStates();

    /** Setup the initial state of the kilobot pc_kilobot_entity */
    void SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity);

    /** Setup virtual environment */
    void SetupVirtualEnvironments(TConfigurationNode &t_tree);

    /** Check if tasks are distant enough */
    bool DistantEnoughTasks(CVector2 some_position);

    /** Get experiment variables */
    void GetExperimentVariables(TConfigurationNode &t_tree);

    /** Get the message to send to a Kilobot according to its position */
    void UpdateKilobotState(CKilobotEntity &c_kilobot_entity);

    /** Get the message to send to a Kilobot according to its position */
    void UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity);

    /** Used to plot the Virtual environment on the floor */
    virtual CColor GetFloorColor(const CVector2 &vec_position_on_plane);

    /** 2D vector rotation */
    CVector2 VectorRotation2D(Real angle, CVector2 vec);

    /** Simulate proximity sensor*/
    std::vector<int> Proximity_sensor(CVector2 obstacle_direction, Real kOrientation, int num_sectors);

private:
    /************************************/
    /*  Virtual Environment variables   */
    /************************************/

    /* virtual circular task*/
    struct SVirtualArea
    {
        int id;
        CVector2 position;
        Real radius;
        CColor color;   // type of task, blue:simple red:hard
        int contained;  // how many kilobots are partecipating at the task
        bool completed; //"true" if the task is completed
        Real creationTime = 0.0;
        Real completitionTime = 0.0;
    };

    std::vector<SVirtualArea> multiArea;

    struct SVirtualWalls
    {
        CVector2 Center;
        Real Radius;       // radius for the circular arena
        Real Wall_width;   // wall width
        Real Wall_height;  // wall height
        Real Wall_numbers; // number of walls
    };

    SVirtualWalls m_ArenaStructure;

    typedef enum //kilobot state
    {
        OUTSIDE_AREAS = 0, //looking for a task
        INSIDE_AREA = 1,   //participating a task
        LEAVING = 2,       //elapsed timeout -> leaving the task
    } SRobotState;

    struct FloorColorData //contains components of area color
    {
        UInt8 R;
        UInt8 G;
        UInt8 B;
    };

    UInt32 random_seed;
    int desired_num_of_areas;        //number of task on the field (max 16-4=12 (removing the 4 corners))
    int hard_tasks;                  //the number of hard task (the one which requires more robots to be completed)
    std::vector<int> hard_tasks_vec; //IDs of active hard tasks
    int vSoftRequiredKilobots;       // # robots to complete soft tasks
    int vHardRequiredKilobots;       // # robots to complete hard tasks
    bool adaptive_walk;              //if true, trying to complete more RED tasks as possible (completed RED task -> brownian motion, BLUE task -> persistent motion)
    double kTimerMultiplier;         //multiplicative constant for the timeout study
    double kRespawnTimer;            //once completed, an area will appear again after kRespawnTimer seconds

    /************************************/
    /*       Experiment variables       */
    /************************************/

    /* random number generator */
    CRandom::CRNG *c_rng;

    std::vector<Real> m_vecLastTimeMessaged;
    Real m_fMinTimeBetweenTwoMsg;

    /*Kilobots properties*/
    std::vector<CVector2> m_vecKilobotsPositions;
    std::vector<argos::CColor> m_vecKilobotsColours;
    std::vector<CRadians> m_vecKilobotsOrientations;
    std::vector<SRobotState> m_vecKilobotStates_ALF;
    std::vector<UInt8> m_vecKilobotsTimeRequest; //vector that determines waiting time
    std::vector<int> m_vecKilobotsPositionTask;  // says in which area the KB is: -1 if walking, areaID if inside an area

    /* output LOG files */
    std::ofstream m_kiloOutput;
    std::ofstream m_areaOutput;
    std::ofstream m_taskOutput;
    std::ofstream m_elpsTimeoutOutput;

    /* output file name*/
    std::string m_strKiloOutputFileName;
    std::string m_strAreaOutputFileName;
    std::string m_strTaskOutputFileName;
    std::string m_strElpsTimeoutOutputFileName;

    /* data acquisition frequency in ticks */
    UInt16 m_unDataAcquisitionFrequency;
};

#endif