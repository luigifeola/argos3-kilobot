#include "wallAvoidance_ALF.h"

WallavoidanceCALF::WallavoidanceCALF() : m_unDataAcquisitionFrequency(20)
{
}

WallavoidanceCALF::~WallavoidanceCALF()
{
}

void WallavoidanceCALF::Init(TConfigurationNode &t_node)
{
    CALF::Init(t_node);
}

void WallavoidanceCALF::SetupInitialKilobotStates()
{
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);

    for (UInt16 it = 0; it < m_tKilobotEntities.size(); it++)
    {
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }
}

void WallavoidanceCALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecLastTimeMessaged[unKilobotID] = -1;
}

void WallavoidanceCALF::SetupVirtualEnvironments(TConfigurationNode &t_tree)
{
    /* Read arena parameters */
    vArena_size = argos::CSimulator::GetInstance().GetSpace().GetArenaSize().GetX();
    vDistance_threshold = vArena_size / 2.0 - 2.0 * kKiloDiameter;
}

void WallavoidanceCALF::GetExperimentVariables(TConfigurationNode &t_tree)
{
    TConfigurationNode &tExperimentVariablesNode = GetNode(t_tree, "variables");
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "proximity_bits", proximity_bits, proximity_bits);
}

// #ifdef WALL_AVOIDANCE
CVector2 WallavoidanceCALF::VectorRotation2D(Real angle, CVector2 vec)
{
    Real kx = (cos(angle) * vec.GetX()) + (-1.0 * sin(angle) * vec.GetY());
    Real ky = (sin(angle) * vec.GetX()) + (cos(angle) * vec.GetY());
    CVector2 rotated_vector(kx, ky);
    return rotated_vector;
}

std::vector<int> WallavoidanceCALF::Proximity_sensor(CVector2 obstacle_direction, Real kOrientation, int num_sectors)
{
    double sector = M_PI_2 / (num_sectors / 2.0);
    std::vector<int> proximity_values;

    for (int i = 0; i < num_sectors; i++)
    {
        CVector2 sector_dir_a = VectorRotation2D((kOrientation + M_PI_2 - i * sector), left_direction);
        CVector2 sector_dir_b = VectorRotation2D((kOrientation + M_PI_2 - (i + 1) * sector), left_direction);

        if (obstacle_direction.DotProduct(sector_dir_a) >= 0.0 || obstacle_direction.DotProduct(sector_dir_b) >= 0.0)
        {
            proximity_values.push_back(0);
        }
        else
        {
            proximity_values.push_back(1);
        }
    }

    return proximity_values;
}
// #endif

void WallavoidanceCALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity)
{
    m_tALFKilobotMessage tKilobotMessage, tEmptyMessage, tMessage;
    bool bMessageToSend = false;
    UInt8 unKilobotID = GetKilobotId(c_kilobot_entity);
    const CVector2 kiloPos = GetKilobotPosition(c_kilobot_entity);
    const Real kiloOri = GetKilobotOrientation(c_kilobot_entity).GetValue();

    /********* WALL AVOIDANCE STUFF *************/
    UInt8 proximity_sensor_dec = 0; //8 bit proximity sensor as decimal
    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID] < m_fMinTimeBetweenTwoMsg)
    {
        return;
    }

    if (fabs(kiloPos.GetX()) > vDistance_threshold ||
        fabs(kiloPos.GetY()) > vDistance_threshold)

    {
        std::vector<int> proximity_vec;

        if (kiloPos.GetX() > vDistance_threshold)
        {
            // std::cerr<<"RIGHT\n";
            proximity_vec = Proximity_sensor(right_direction, kiloOri, proximity_bits);
        }
        else if (kiloPos.GetX() < -1.0 * vDistance_threshold)
        {
            // std::cerr<<"LEFT\n";
            proximity_vec = Proximity_sensor(left_direction, kiloOri, proximity_bits);
        }

        if (kiloPos.GetY() > vDistance_threshold)
        {
            // std::cerr<<"UP\n";
            if (proximity_vec.empty())
                proximity_vec = Proximity_sensor(up_direction, kiloOri, proximity_bits);
            else
            {
                std::vector<int> prox = Proximity_sensor(up_direction, kiloOri, proximity_bits);
                std::vector<int> elementwiseOr;
                elementwiseOr.reserve(prox.size());
                std::transform(proximity_vec.begin(), proximity_vec.end(), prox.begin(), std::back_inserter(elementwiseOr), std::logical_or<>());

                proximity_vec = elementwiseOr;
            }
        }
        else if (kiloPos.GetY() < -1.0 * vDistance_threshold)
        {
            // std::cerr<<"DOWN\n";
            if (proximity_vec.empty())
                proximity_vec = Proximity_sensor(down_direction, kiloOri, proximity_bits);
            else
            {
                std::vector<int> prox = Proximity_sensor(up_direction, kiloOri, proximity_bits);
                std::vector<int> elementwiseOr;
                elementwiseOr.reserve(prox.size());
                std::transform(proximity_vec.begin(), proximity_vec.end(), prox.begin(), std::back_inserter(elementwiseOr), std::logical_or<>());

                proximity_vec = elementwiseOr;
            }
        }

        proximity_sensor_dec = std::accumulate(proximity_vec.begin(), proximity_vec.end(), 0, [](int x, int y) { return (x << 1) + y; });
        // To turn off the wall avoidance decomment this
        //proximity_sensor_dec = 0;

        m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
        bMessageToSend = true;

        tKilobotMessage.m_sID = unKilobotID;
        tKilobotMessage.m_sData = proximity_sensor_dec;
    }

    if (bMessageToSend)
    {
        for (int i = 0; i < 9; ++i)
        {
            m_tMessages[unKilobotID].data[i] = 0;
        }
        tEmptyMessage.m_sID = 1023;
        tEmptyMessage.m_sType = 0;
        tEmptyMessage.m_sData = 0;

        for (int i = 0; i < 3; ++i)
        {
            /* Packing the message */
            if (i == 0)
            {
                tMessage = tKilobotMessage;
            }
            else
            {
                tMessage = tEmptyMessage;
            }
            /* Packing the message */
            m_tMessages[unKilobotID].data[i * 3] = (tMessage.m_sID >> 2);
            m_tMessages[unKilobotID].data[1 + i * 3] = (tMessage.m_sID << 6);
            m_tMessages[unKilobotID].data[1 + i * 3] = m_tMessages[unKilobotID].data[1 + i * 3] | (tMessage.m_sType << 2);
            m_tMessages[unKilobotID].data[1 + i * 3] = m_tMessages[unKilobotID].data[1 + i * 3] | (tMessage.m_sData >> 8);
            m_tMessages[unKilobotID].data[2 + i * 3] = tMessage.m_sData;
            //std::cout<<" robot "<<tMessage.m_sID<<" "<<tMessage.m_sType<<std::endl;
        }
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity, &m_tMessages[unKilobotID]);
    }
    else
    {
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity, NULL);
    }
}

CColor WallavoidanceCALF::GetFloorColor(const CVector2 &vec_position_on_plane)
{
    CColor cColor = CColor::WHITE;

    // Top border for wall avoidance
    if (vec_position_on_plane.GetY() < vDistance_threshold + 0.005 && vec_position_on_plane.GetY() > vDistance_threshold - 0.005)
    {
        cColor = CColor::ORANGE;
    }
    // Bottom border for wall avoidance
    if (vec_position_on_plane.GetY() < -1.0 * (vDistance_threshold - 0.005) && vec_position_on_plane.GetY() > -1 * (vDistance_threshold + 0.005))
    {
        cColor = CColor::ORANGE;
    }
    // Right border for wall avoidance
    if (vec_position_on_plane.GetX() < vDistance_threshold + 0.005 && vec_position_on_plane.GetX() > vDistance_threshold - 0.005)
    {
        cColor = CColor::ORANGE;
    }
    // Left border for wall avoidance
    if (vec_position_on_plane.GetX() < -1.0 * (vDistance_threshold - 0.005) && vec_position_on_plane.GetX() > -1 * (vDistance_threshold + 0.005))
    {
        cColor = CColor::ORANGE;
    }

    // // y-axis
    // if(vec_position_on_plane.GetX() < 0.01 && vec_position_on_plane.GetX()> -0.01 ){
    //     cColor = CColor::BLUE;
    // }

    return cColor;
}

REGISTER_LOOP_FUNCTIONS(WallavoidanceCALF, "ALF_WallAvoidance_loop_function")