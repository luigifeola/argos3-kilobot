#ifndef CRE_ALF_H
#define CRE_ALF_H

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

    virtual void PostExperiment();

    /** Log Kilobot pose and state */
    void KiloLOG();

    /** Log area pos, type, state (completed or not) */
    void AreaLOG();

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

    /* Gets the current state of the Kilobots */
    void UpdateKilobotStates();

    /** Get the message to send to a Kilobot according to its position */
    void UpdateKilobotState(CKilobotEntity &c_kilobot_entity);

    /** Get the message to send to a Kilobot according to its position */
    void UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity);

    /** Used to plot the Virtual environment on the floor */
    virtual CColor GetFloorColor(const CVector2 &vec_position_on_plane);

    /*** WALL_AVOIDANCE stuff */
    /** 2D vector rotation */
    CVector2 VectorRotation2D(Real angle, CVector2 vec);

    /** Simulate proximity sensor*/
    std::vector<int> Proximity_sensor(CVector2 obstacle_direction, Real kOrientation, int num_sectors);

private:
    /************************************/
    /*  Virtual Environment variables   */
    /************************************/
    /* virtual environment struct*/
    struct SVirtualArea //parameters of the circular areas
    {
        CVector2 Center;
        Real Radius;
        CColor Color;
        bool Completed; //set to "true" after the task is completed
    };
    std::vector<SVirtualArea> multiArea;

    struct TransmittingKilobot //parameters of the circular areas
    {
        int xCoord;
        int yCoord;
        int commit; //0 uncommitted, 1 committed NORTH, 2 committed SOUTH
    };

    typedef enum // Enum for the robot states
    {
        UNCOMMITTED = 0,
        COMMITTED_N = 1,
        COMMITTED_S = 2,
    } SRobotState;

    std::vector<TransmittingKilobot> multiTransmittingKilobot; // position and commit of flying robots (received from server side)

    struct decisionMessage //structure for decision-making flying robot message
    {
        UInt8 ID;
        UInt8 resource_North;
        UInt8 resource_South;
        UInt8 wall_avoidance_bits;
    };

    std::string mode;
    std::string IP_ADDR; //ip address where to connect
    UInt32 random_seed;
    float vision_range; //range in with a flying robot can see resources
    int desired_North_areas;
    int desired_South_areas;
    float communication_range; // range in with a flying robot can see a ground robot
    char inputBuffer[2000];    // array containing the message received from the socket
    std::string outputBuffer;  // array  containing the message to send
    char storeBuffer[2000];    // array where to store input message to keep it available
    int bytesReceived;         // length of received string
    int serverSocket;
    int clientSocket;
    int num_of_areas = 0; //number of clustering areas

    /*vectors as long as the number of kilobots*/
    std::vector<int> actual_orientation; // vector containing orientation to command ground robot for pointing in the right region
    std::vector<int> command;            // contains informations about actual semiplan direction where the robot are commanded to go (1=NORTH 2=SOUTH)
    std::vector<int> activated_North_areas;
    std::vector<int> activated_South_areas;
    const int max_North_area_id = 49;
    const int max_South_area_id = 99;

    /*vectors as long as the number of areas*/

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
    std::ofstream m_taskOutput;

    /* output file name*/
    std::string m_strKiloOutputFileName;
    std::string m_strTaskOutputFileName;

    /* data acquisition frequency in ticks */
    UInt16 m_unDataAcquisitionFrequency;
};

#endif