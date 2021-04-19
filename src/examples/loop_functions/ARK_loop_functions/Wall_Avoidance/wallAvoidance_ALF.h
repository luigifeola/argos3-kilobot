/**
 * @file <wallAvoidance_ALF.h>
 *
 * @author Luigi Feola <feola@diag.uniroma1.it>
 *
 * @brief This is the header file of the ARK Loop Function (ALF), the simulated counterpart of the ARK (Augmented Reality for Kilobots) system. Here, is implemented
 * a wall avoidance procedure.
 *
 */

#ifndef WALLAVOIDANCE_ALF_H
#define WALLAVOIDANCE_ALF_H

namespace argos
{
    class CSpace;
    class CFloorEntity;
    class CSimulator;
}

#include <math.h>
#include <numeric>

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

#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

#include <array>

using namespace argos;

class WallavoidanceCALF : public CALF
{

public:
    WallavoidanceCALF();

    virtual ~WallavoidanceCALF() {}

    virtual void Init(TConfigurationNode &t_tree);

    virtual void Reset();

    virtual void Destroy();

    /** Get a Vector of all the Kilobots in the space */
    void GetKilobotsEntities();

    /** Setup the initial state of the Kilobots in the space */
    void SetupInitialKilobotStates();

    /** Setup the initial state of the kilobot pc_kilobot_entity */
    void SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity);

    /** Setup virtual environment */
    void SetupVirtualEnvironments(TConfigurationNode &t_tree);

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
    std::vector<Real> m_vecLastTimeMessaged;
    Real m_fMinTimeBetweenTwoMsg;

    /* data acquisition frequency in ticks */
    UInt16 m_unDataAcquisitionFrequency;

    // environment setup
    double vArena_size = 1.0;
    const double kKiloDiameter = 0.033;
    double vDistance_threshold = vArena_size / 2.0 - 2.0 * kKiloDiameter;

    // wall avoidance stuff
    const CVector2 up_direction = CVector2(0.0, -1.0);
    const CVector2 down_direction = CVector2(0.0, 1.0);
    const CVector2 left_direction = CVector2(1.0, 0.0);
    const CVector2 right_direction = CVector2(-1.0, 0.0);

    // be careful to update proximity_bits also in the kilobot behaviour
    int proximity_bits;
};

#endif // WALLAVOIDANCE_ALF_H
