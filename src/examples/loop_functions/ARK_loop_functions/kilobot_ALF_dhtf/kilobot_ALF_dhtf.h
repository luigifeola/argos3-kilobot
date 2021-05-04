#ifndef DHTF_ALF_H
#define DHTF_ALF_H

#define DEBUGGING

namespace argos
{
    class CSpace;
    class CFloorEntity;
    class CSimulator;
}

#include <math.h>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
#include <cstring>
#include <cmath>
#include <unistd.h>
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

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

using namespace argos;

class CALFClientServer : public CALF
{

public:
    CALFClientServer();

    virtual ~CALFClientServer();

    virtual void Init(TConfigurationNode &t_tree);

    virtual void Reset();

    virtual void PostStep();

    /** Log area pos, type, state (completed or not) */
    void AreaLOG();

    /** Log Kilobot pose and state */
    void KiloLOG();

    virtual void Destroy();

    /** Setup the initial state of the Kilobots in the space */
    void SetupInitialKilobotStates();

    /** Setup the initial state of the kilobot pc_kilobot_entity */
    void SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity);

    /** Experiment configuration methods (From .argos files) */

    /** Setup virtual environment */
    void SetupVirtualEnvironments(TConfigurationNode &t_tree);

    /** Get experiment variables */
    void GetExperimentVariables(TConfigurationNode &t_tree);

    /** Init environment*/
    void Initialise_environment();

    /** Init environment with 4 regions: RR,RB,BR,BB*/
    void Initialise_environment_4_regions();

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

    /** Setup socket*/
    void Initialise_socket();

private:
    /************************************/
    /*  Virtual Environment variables   */
    /************************************/
    /* virtual environment struct*/
    struct SVirtualArea //parameters of the circular areas
    {
#ifdef DEBUGGING
        Real creationTime = 0.0;
        Real completitionTime = 0.0;
#endif
        int id;
        CVector2 Center;
        Real Radius;
        CColor Color;
        int contained;
        bool Completed; //set to "true" after the task is completed
    };
    std::vector<SVirtualArea> multiArea;

    typedef enum //states of the kilobots
    {
        OUTSIDE_AREAS = 0,
        INSIDE_AREA = 1,
        LEAVING = 2,
    } SRobotState;

    typedef enum
    {
        kBLUE = 0,
        kRED = 1,
    } colour;

    struct WaitingTimes
    {
        int BB = 1;
        int BR = 2;
        int RB = 3;
        int RR = 5;
    } waiting_times;

    struct FloorColorData //contains components of area color
    {
        UInt8 R;
        UInt8 G;
        UInt8 B;
    };

    std::string mode;                          //can be SERVER or CLIENT
    std::string IP_ADDR;                       //ip address where to connect
    bool augmented_knowledge;                  //TRUE: ARK knows the color of areas on the other arena; FALSE: ARK knows color of its own areas only; timeout constant are set consequently
    UInt32 random_seed;                        //to reproduce same random tests
    UInt8 desired_num_of_areas;                //number of exploitable areas for the experiment (max 16-4=12 (removing the 4 corners))
    UInt8 hard_tasks;                          //the number of red areas (the ones that require more robots)
    std::vector<int> activated_areas;          //IDs of active areas
    std::vector<int> hard_tasks_vec;           //IDs of hard tasks for the server
    std::vector<int> hard_tasks_client_vec;    //IDs of hard tasks for the client
    bool mixed;                                //if mixed, we will have only red-blue or blue-red areas
    bool adaptive_walk;                        //if true, trying to complete more RED tasks as possible (completed RED task -> brownian motion, BLUE task -> persistent motion)
    bool fourRegions;                          //if true, the combination of client and server arenas give rise to 4 regions: RR,RB,BR,BB
    std::vector<int> otherColor;               //Color of the areas on the other ARK
    bool IsNotZero(int i) { return (i != 0); } //to count how non 0 emelent there are in sending/receiving buffer
    char inputBuffer[30];                      // array containing the message received from the socket e.g.
    std::string initialise_buffer;             // buffer containing setup values (active and type of the task)
    std::string outputBuffer;                  //array  containing the message to send
    char storeBuffer[30];                      //array where to store input message to keep it available
    int bytesReceived;                         //length of received string
    int serverSocket;                          //socket variable
    int clientSocket;                          //socket variable
    UInt8 num_of_areas;                        //initial number of clustering areas i.e. 16, will be reduced to desired_num_of_areas
    double kTimerMultiplier;                   //multiplicative constant for the timeout study
    double kRespawnTimer;                      //when completed, timer starts and when it will expire the area is reactivated
                                               // #ifndef DEBUG
    std::vector<double> vCompletedTime;        //vector with completition time
                                               // #endif
    bool initialised;                          // true when client ACK the initial setup

    /*vectors as long as the number of kilobots*/
    std::vector<UInt8> request; //vector that determines waiting time: 1 for kilobots on blue areas and 3 for the ones on red areas (multiplied times 500 gives the number of cycles before timeout)
    std::vector<SInt8> whereis; // says in which area the KB is: -1 if walking, (index of area) if inside an area

    std::vector<Real> m_vecLastTimeMessaged;
    Real m_fMinTimeBetweenTwoMsg;

    /************************************/
    /*       Experiment variables       */
    /************************************/

    /*Kilobots properties*/
    std::vector<CVector2> m_vecKilobotsPositions;
    std::vector<CRadians> m_vecKilobotsOrientations;
    std::vector<SRobotState> m_vecKilobotStates_ALF;

    /* output LOG files */
    std::ofstream m_kiloOutput;
    std::ofstream m_areaOutput;
    std::ofstream m_taskOutput;

    /* output file name*/
    std::string m_strKiloOutputFileName;
    std::string m_strAreaOutputFileName;
    std::string m_strTaskOutputFileName;

    /* data acquisition frequency in ticks */
    UInt16 m_unDataAcquisitionFrequency;
};

#endif