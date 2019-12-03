#ifndef CRWLEVY_ALF_H
#define CRWLEVY_ALF_H

namespace argos {
class CSpace;
class CFloorEntity;
class CSimulator;
}

#include <math.h>


#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kilobot/simulator/ALF.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>


#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/ray2.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
// #include <argos3/core/utility/math/range.h>
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

/** Amaury */
// #include <argos3/core/control_interface/ci_controller.h>
// #include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
// #include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
// #include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_communication_actuator.h>
// #include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_communication_sensor.h>
// #include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_controller.h>
// #include <argos3/core/utility/configuration/argos_configuration.h>
// #include <algorithm>
// #include <vector>
// #include <argos3/plugins/simulator/entities/box_entity.h>
// #include <sys/types.h>
// #include <dirent.h>


using namespace argos;



class CCrwlevyALF : public CALF
{

public:

    CCrwlevyALF();

    virtual ~CCrwlevyALF();

    virtual void Init(TConfigurationNode& t_tree);

    virtual void Reset();

    virtual void Destroy();

    /** Setup the initial state of the Kilobots in the space */
    void SetupInitialKilobotStates();

    /** Setup the initial state of the kilobot pc_kilobot_entity */
    void SetupInitialKilobotState(CKilobotEntity& c_kilobot_entity);

    /** Experiment configuration methods (From .argos files) */

    /** Setup virtual environment */
    void SetupVirtualEnvironments(TConfigurationNode& t_tree);

    /** Get experiment variables */
    void GetExperimentVariables(TConfigurationNode& t_tree);

    /** Virtual environment visualization updating */


    /** Get the message to send to a Kilobot according to its position */
    void UpdateKilobotState(CKilobotEntity& c_kilobot_entity);


    /** Get the message to send to a Kilobot according to its position */
    void UpdateVirtualSensor(CKilobotEntity& c_kilobot_entity);

    /** Store fpt, fit data on file */
    void UpdateResults();

    /** Used to plot the Virtual environment on the floor */
    virtual CColor GetFloorColor(const CVector2& vec_position_on_plane);

private:

    UInt32 num_robots_with_discovery;
    UInt32 num_robots_with_info;
    
    /************************************/
    /*  Virtual Environment variables   */
    /************************************/
    /* virtual environment struct*/
    struct SVirtualArea
    {
        CVector2 Center;
        Real Radius;
        CColor Color;
    };

    SVirtualArea m_sClusteringHub;
    
    struct SVirtualPerimeter
    {
        // circular arena variables
        Real circular_arena_radius; // radius for the circular arena
        Real circular_arena_width;  // wall width
        Real circular_arena_height; // wall height
        Real circular_arena_walls;  // number of walls
    };

    SVirtualPerimeter m_WallStructure;

    typedef enum
    {
        NOT_TARGET_FOUND=0,
        TARGET_FOUND=1,
        TARGET_COMMUNICATED=2,
    } SRobotState;

    /* used to store the state of each kilobot */
    std::vector<SRobotState> m_vecKilobotStates;

    /* used to initialize the position of each kilobot uniformly */
    std::vector<CVector2> m_vecKilobotsPositions;
    std::vector<CVector3> m_vecDesInitKilobotPose; //x,y,theta
    
    /** Structure to contain data to evaluate first passage time and
     *  convergence time of the experiment
     *  single item example: <kilo_id, <f_p_t, f_i_t>> 
     * */
    std::map<UInt16,std::pair<UInt32,UInt32>> m_KilobotResults; 

    /* used to store the last message sent to each kilobot */
    std::vector<Real> m_vecLastTimeMessaged;
    Real m_fMinTimeBetweenTwoMsg;

    /************************************/
    /*       Experiment variables       */
    /************************************/

    /* crwlevy exponents */
    Real crw_exponent;
    Real levy_exponent;

    /* output file for data acquisition */
    std::ofstream m_cOutput;

    /* output file name*/
    std::string m_strOutputFileName;

    /* data acquisition frequency in ticks */
    UInt16 m_unDataAcquisitionFrequency;

    UInt32 m_unFullDiscoveryTime;
    UInt32 m_unFullInformationTime;
    Real m_fFractionWithInformation;
    Real m_fFractionWithDiscovery;
};

#endif
