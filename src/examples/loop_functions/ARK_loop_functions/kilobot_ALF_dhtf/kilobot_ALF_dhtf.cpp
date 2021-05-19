#include "kilobot_ALF_dhtf.h"

namespace
{
    // environment setup
    const double kArena_size = 1.0;
    const double kKiloDiameter = 0.033;
    const double kDistance_threshold = kArena_size / 2.0 - 2.0 * kKiloDiameter;
    const int max_area_id = 15;
    const int kSoftTask = 2;
    const int kHardTask = 6;

    // avoid to choose corner areas
    const std::vector<int> vForbidden({0, 3, 12, 15});

    // wall avoidance stuff
    const CVector2 up_direction(0.0, -1.0);
    const CVector2 down_direction(0.0, 1.0);
    const CVector2 left_direction(1.0, 0.0);
    const CVector2 right_direction(-1.0, 0.0);
    const int proximity_bits = 8;

    //for sampling data
    int internal_counter = 0;

    // 4 regions division, RR, RB, BR, BB
    const int kNumOfRegions = 4;
    const int kAreaPerRegion = 2;
    //area ids for each region
    const std::vector<int> top_left({1, 4, 5});
    const std::vector<int> top_right({2, 6, 7});
    const std::vector<int> bottom_left({8, 9, 13});
    const std::vector<int> bottom_right({10, 11, 14});
}

CALFClientServer::CALFClientServer() : m_unDataAcquisitionFrequency(20)
{
}

CALFClientServer::~CALFClientServer()
{
}

void CALFClientServer::Init(TConfigurationNode &t_node)
{
    CALF::Init(t_node);

    /* Read parameters from .argos*/
    TConfigurationNode &tModeNode = GetNode(t_node, "extra_parameters");
    GetNodeAttribute(tModeNode, "timeout_const", kTimerMultiplier);
    GetNodeAttribute(tModeNode, "timeoutBB", waiting_times.BB);
    GetNodeAttribute(tModeNode, "timeoutBR", waiting_times.BR);
    GetNodeAttribute(tModeNode, "timeoutRB", waiting_times.RB);
    GetNodeAttribute(tModeNode, "timeoutRR", waiting_times.RR);
    GetNodeAttributeOrDefault(tModeNode, "adaptive", adaptive_walk, false);
}

/**
 * ENVIRONMENT INITIALISATION
 */
void CALFClientServer::Initialise_environment_4_regions()
{
    /* Select areas */
    srand(random_seed);

    /* GENERATE RANDOM IDs AND RANDOM HARD TASK for server and client*/
    std::default_random_engine re;
    re.seed(random_seed);

    /* Active areas ID */
    std::vector<int> vRegionIDs(3); //each region has only 3 possible id activable
    for (int i = 0; i < kNumOfRegions; i++)
    {
        switch (i)
        {
        case 0:
            vRegionIDs.assign(top_left.begin(), top_left.end());
            break;
        case 1:
            vRegionIDs.assign(top_right.begin(), top_right.end());
            break;
        case 2:
            vRegionIDs.assign(bottom_left.begin(), bottom_left.end());
            break;
        case 3:
            vRegionIDs.assign(bottom_right.begin(), bottom_right.end());
            break;

        default:
            std::cerr << "ERROR, OPTION NOT AVAILABLE\n";
            break;
        }
        while (activated_areas.size() < kAreaPerRegion + i * kAreaPerRegion)
        {
            std::uniform_int_distribution<int> distr(0, vRegionIDs.size() - 1);
            int random_number;
            do
            {
                random_number = distr(re);
            } while ((std::find(activated_areas.begin(), activated_areas.end(), vRegionIDs[random_number]) != activated_areas.end()));
            activated_areas.push_back(vRegionIDs[random_number]);

            if (i == 0 || i == 1)
            {
                hard_tasks_vec.push_back(vRegionIDs[random_number]);
            }
        }
    }
    std::sort(activated_areas.begin(), activated_areas.end());
    std::sort(hard_tasks_vec.begin(), hard_tasks_vec.end());

    std::cout << "activated areas:";
    for (auto a : activated_areas)
    {
        std::cout << a << '\t';
    }
    std::cout << std::endl;

    std::cout << "hard tasks:";
    for (auto ht : hard_tasks_vec)
    {
        std::cout << ht << '\t';
    }
    std::cout << std::endl;
}

/**
 * ENVIRONMENT INITIALISATION
 */
void CALFClientServer::Initialise_environment()
{
    /* Select areas */
    srand(random_seed);

    /* GENERATE RANDOM IDs AND RANDOM HARD TASK for server and client*/
    std::default_random_engine re;
    re.seed(random_seed);

    /* Active areas ID */
    while (activated_areas.size() < desired_num_of_areas)
    {
        if (desired_num_of_areas - 1 > max_area_id)
        {
            std::cerr << "Requested more areas then the available ones, WARNING!";
        }

        std::uniform_int_distribution<int> distr(0, max_area_id);
        int random_number;
        do
        {
            random_number = distr(re);
        } while ((std::find(activated_areas.begin(), activated_areas.end(), random_number) != activated_areas.end()) ||
                 (std::find(vForbidden.begin(), vForbidden.end(), random_number) != vForbidden.end()));
        activated_areas.push_back(random_number);
    }
    std::sort(activated_areas.begin(), activated_areas.end());

    /* Hard task for the server */
    while (hard_tasks_vec.size() < hard_tasks)
    {
        std::uniform_int_distribution<int> distr(0, max_area_id);
        int random_number;
        do
        {
            random_number = distr(re);
        } while (std::find(activated_areas.begin(), activated_areas.end(), random_number) == activated_areas.end() ||
                 std::find(hard_tasks_vec.begin(), hard_tasks_vec.end(), random_number) != hard_tasks_vec.end());
        hard_tasks_vec.push_back(random_number);
    }
    std::sort(hard_tasks_vec.begin(), hard_tasks_vec.end());
}

void CALFClientServer::Reset()
{
}

void CALFClientServer::Destroy()
{
}

void CALFClientServer::SetupInitialKilobotStates()
{
    m_vecKilobotStates_ALF.resize(m_tKilobotEntities.size());
    m_vecKilobotsPositions.resize(m_tKilobotEntities.size());
    m_vecKilobotsOrientations.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);

    /* Compute the number of kilobots on the field*/
    for (UInt16 it = 0; it < m_tKilobotEntities.size(); it++)
    {
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }

    /* Initialization of kilobots variables */
    request = std::vector<int>(m_tKilobotEntities.size(), 0);
    whereis = std::vector<SInt8>(m_tKilobotEntities.size(), -1);
}

void CALFClientServer::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
    m_vecLastTimeMessaged[unKilobotID] = -1000;

    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
}

void CALFClientServer::SetupVirtualEnvironments(TConfigurationNode &t_tree)
{
    random_seed = GetSimulator().GetRandomSeed();

    TConfigurationNode &tModeNode = GetNode(t_tree, "extra_parameters");
    GetNodeAttribute(tModeNode, "desired_num_of_areas", desired_num_of_areas);
    GetNodeAttribute(tModeNode, "hard_tasks", hard_tasks);
    GetNodeAttributeOrDefault(tModeNode, "fourRegions", fourRegions, false);
    GetNodeAttribute(tModeNode, "reactivation_timer", kRespawnTimer);

    TConfigurationNode &tVirtualEnvironmentsNode = GetNode(t_tree, "environments");
    TConfigurationNodeIterator itAct;

    /* Compute number of areas on the field*/
    int num_of_areas = 0;
    for (itAct = itAct.begin(&tVirtualEnvironmentsNode); itAct != itAct.end(); ++itAct)
    {
        num_of_areas += 1;
    }

    /* Build the structure with areas data */
    multiArea.resize(num_of_areas);

    int i = 0;
    for (itAct = itAct.begin(&tVirtualEnvironmentsNode); itAct != itAct.end(); ++itAct, i++)
    {
        std::string s = std::to_string(i);
        std::string var = std::string("Area") + std::string(s);
        TConfigurationNode &t_VirtualClusteringHubNode = GetNode(tVirtualEnvironmentsNode, var);
        GetNodeAttribute(t_VirtualClusteringHubNode, "position", multiArea[i].Center);
        GetNodeAttribute(t_VirtualClusteringHubNode, "radius", multiArea[i].Radius);
    }
    /* White set as default color*/
    for (int ai = 0; ai < num_of_areas; ai++)
    {
        multiArea[ai].id = ai;
        multiArea[ai].Color = argos::CColor::WHITE;
        multiArea[ai].contained = 0;
        multiArea[ai].Completed = true;
    }

    if (adaptive_walk)
        std::cout << "Adaptive Walk\n";
    /* Randomly select the desired number of tasks between the available ones, set color and communicate them to the client */

    Initialise_environment();

    // std::cout << "***********Active areas*****************\n";
    // for (int ac_ar : activated_areas)
    // {
    //     std::cout << ac_ar << '\t';
    // }
    // std::cout << std::endl;

    // std::cout << "Hard task id\n";
    // for (int h_t : hard_tasks_vec)
    // {
    //     std::cout << h_t << '\t';
    // }
    // std::cout << std::endl;

    //Remove the extra multiArea loaded from .argos file
    for (int i = 0; i < multiArea.size(); i++)
    {
        // fill own colors
        if (std::find(activated_areas.begin(), activated_areas.end(), multiArea[i].id) == activated_areas.end())
        {
            multiArea.erase(multiArea.begin() + i);
            i -= 1;
        }
        else
        {
            if (std::find(hard_tasks_vec.begin(), hard_tasks_vec.end(), multiArea[i].id) != hard_tasks_vec.end())
            {
                multiArea[i].Color = argos::CColor::RED;
            }
            else
            {
                multiArea[i].Color = argos::CColor::BLUE;
            }

            multiArea[i].creationTime = 0.0;
            multiArea[i].completitionTime = 0.0;
            multiArea[i].Completed = false;
        }
    }

    for (auto area : multiArea)
    {
        std::cout << "Areaid:" << area.id
                  << " Area color" << (area.Color == argos::CColor::RED ? "red" : "blue")
                  << " area completed:" << (area.Completed ? "yes" : "no")
                  << std::endl;
    }

    // Real creationTime = 0.0;
    // Real completitionTime = 0.0;
    // CColor Color;
    // int contained;
    // bool Completed;
}

void CALFClientServer::GetExperimentVariables(TConfigurationNode &t_tree)
{
    TConfigurationNode &tExperimentVariablesNode = GetNode(t_tree, "variables");
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);
}

void CALFClientServer::UpdateKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);

    /** WARNING: here you manage completion/respawn of the areas */
    // /* --------- SERVER --------- */
    // if (mode == "SERVER")
    // {
    //     /* Reactivation areas check */
    //     for (int i = 0; i < multiArea.size(); i++)
    //     {
    //         if ((multiArea[i].Completed == true) &&
    //             (m_fTimeInSeconds - multiArea[i].completitionTime >= kRespawnTimer))
    //         {
    //             multiArea[i].Completed = false;
    //             multiArea[i].contained = 0;
    //             multiArea[i].creationTime = m_fTimeInSeconds;
    //         }
    //     }
    //     /* Task completeness check */
    //     if (storeBuffer[0] == 84) //84 is the ASCII binary for "T"
    //     {
    //         for (int j = 0; j < multiArea.size(); j++)
    //         {
    //             if ((storeBuffer[j + 1] - 48 == 1) && multiArea[j].Completed == false)
    //             {
    //                 if (otherColor[j] == kRED)
    //                 {
    //                     if ((multiArea[j].Color == argos::CColor::RED) && (multiArea[j].contained >= kHardTask))
    //                     {
    //                         multiArea[j].Completed = true;
    //                         // std::cout << m_fTimeInSeconds << "\tred-red task completed" << std::endl;
    //                     }
    //                     if ((multiArea[j].Color == argos::CColor::BLUE) && (multiArea[j].contained >= kSoftTask))
    //                     {
    //                         multiArea[j].Completed = true;
    //                         // std::cout << m_fTimeInSeconds << "\tblue-red task completed" << std::endl;
    //                     }
    //                 }
    //                 if (otherColor[j] == kBLUE)
    //                 {
    //                     if ((multiArea[j].Color == argos::CColor::RED) && (multiArea[j].contained >= kHardTask))
    //                     {
    //                         multiArea[j].Completed = true;
    //                         // std::cout << m_fTimeInSeconds << "\tred-blue task completed" << std::endl;
    //                     }
    //                     if ((multiArea[j].Color == argos::CColor::BLUE) && (multiArea[j].contained >= kSoftTask))
    //                     {
    //                         multiArea[j].Completed = true;
    //                         // std::cout << m_fTimeInSeconds << "\tblue-blue task completed" << std::endl;
    //                     }
    //                 }
    //                 if (multiArea[j].Completed == true)
    //                 {
    //                     multiArea[j].completitionTime = m_fTimeInSeconds;
    //                     m_taskOutput
    //                         << std::noshowpos
    //                         << std::setw(8) << std::setprecision(4) << std::setfill('0')
    //                         << multiArea[j].completitionTime << '\t'
    //                         << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
    //                         << multiArea[j].id << '\t'
    //                         << std::internal << std::noshowpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
    //                         << multiArea[j].creationTime << '\t'
    //                         << std::internal << std::noshowpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
    //                         << multiArea[j].completitionTime << '\t'
    //                         << std::noshowpos << std::setw(1) << std::setprecision(0)
    //                         << (multiArea[j].Color == argos::CColor::RED ? 1 : 0) << '\t'
    //                         << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
    //                         << multiArea[j].contained
    //                         << std::endl;
    //                 }
    //             }
    //         }
    //     }
    // }

    /** WARNING: here you manage FSM for each robot */
    /* State transition*/
    switch (m_vecKilobotStates_ALF[unKilobotID])
    {
    case OUTSIDE_AREAS:
    {
        /* Check if the kilobot is entered in a task area */
        for (int i = 0; i < multiArea.size(); i++)
        {
            Real fDistance = Distance(m_vecKilobotsPositions[unKilobotID], multiArea[i].Center);
            if ((fDistance < (multiArea[i].Radius)) && (multiArea[i].Completed == false))
            { //*1 is a threshold, to include the boarder increase it
                m_vecKilobotStates_ALF[unKilobotID] = INSIDE_AREA;
                /* Check LED color to understand if the robot is leaving or it is waiting for the task */
                if (GetKilobotLedColor(c_kilobot_entity) != argos::CColor::RED)
                {
                    // std::cout<< "inside area = "<< multiArea[i].id << std::endl;
                    /* Check the area color to understand the requirements of the task */
                    if (multiArea[i].Color == argos::CColor::RED)
                    {
                        request[unKilobotID] = (int)(kTimerMultiplier * waiting_times.RR);
                    }
                    if (multiArea[i].Color == argos::CColor::BLUE)
                    {
                        request[unKilobotID] = (int)(kTimerMultiplier * waiting_times.BB);
                    }
                    whereis[unKilobotID] = i;
                    multiArea[i].contained += 1;
                }
            }
        }
        break;
    }
    case INSIDE_AREA:
    {
        /* Check if the kilobot timer for colaboratos is expired */
        if (GetKilobotLedColor(c_kilobot_entity) == argos::CColor::RED)
        {
            m_vecKilobotStates_ALF[unKilobotID] = LEAVING;
            multiArea[whereis[unKilobotID]].contained -= 1;
        }
        /* Else check if the task has been completed */
        else if (multiArea[whereis[unKilobotID]].Completed == true)
        {
            m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
            multiArea[whereis[unKilobotID]].contained = 0;
            whereis[unKilobotID] = -1;
        }
        else if (Distance(m_vecKilobotsPositions[unKilobotID], multiArea[whereis[unKilobotID]].Center) > multiArea[whereis[unKilobotID]].Radius)
        {
            m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
            multiArea[whereis[unKilobotID]].contained -= 1;
            whereis[unKilobotID] = -1;
        }
        break;
    }
    case LEAVING:
    {
        /* Case in which the robot is inside an area but it is moving */
        Real fDistance = Distance(m_vecKilobotsPositions[unKilobotID], multiArea[whereis[unKilobotID]].Center);
        /* Check when the robot is back outside  */
        /** WARNING: be carefull on multiarea.completed, uncomment that part*/
        if ((fDistance > multiArea[whereis[unKilobotID]].Radius) /*|| (multiArea[whereis[unKilobotID]].Completed == true)*/)
        {
            m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
            whereis[unKilobotID] = -1;
        }
        break;
    }
    }
}

CVector2 CALFClientServer::VectorRotation2D(Real angle, CVector2 vec)
{
    Real kx = (cos(angle) * vec.GetX()) + (-1.0 * sin(angle) * vec.GetY());
    Real ky = (sin(angle) * vec.GetX()) + (cos(angle) * vec.GetY());
    CVector2 rotated_vector(kx, ky);
    // std::cout << "VectorRotation2D\n";
    return rotated_vector;
}

std::vector<int> CALFClientServer::Proximity_sensor(CVector2 obstacle_direction, Real kOrientation, int num_sectors)
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

    // std::cout << "Proximity_sensor\n";
    return proximity_values;
}

void CALFClientServer::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity)
{
    /** WARNING: here you send state messages to the kilobot simulatin extra sensors */
    /********* WALL AVOIDANCE STUFF *************/
    UInt8 proximity_sensor_dec = 0; //8 bit proximity sensor as decimal
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);

    // std::cerr<<unKilobotID<<'\t'<<m_vecKilobotsPositions[unKilobotID]<<std::endl;
    if (fabs(m_vecKilobotsPositions[unKilobotID].GetX()) > kDistance_threshold ||
        fabs(m_vecKilobotsPositions[unKilobotID].GetY()) > kDistance_threshold)
    {
        std::vector<int> proximity_vec;

        if (m_vecKilobotsPositions[unKilobotID].GetX() > kDistance_threshold)
        {
            // std::cerr << "RIGHT\n";
            proximity_vec = Proximity_sensor(right_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
        }
        else if (m_vecKilobotsPositions[unKilobotID].GetX() < -1.0 * kDistance_threshold)
        {
            // std::cerr << "LEFT\n";
            proximity_vec = Proximity_sensor(left_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
        }

        if (m_vecKilobotsPositions[unKilobotID].GetY() > kDistance_threshold)
        {
            // std::cerr << "UP\n";
            if (proximity_vec.empty())
                proximity_vec = Proximity_sensor(up_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
            else
            {
                std::vector<int> prox = Proximity_sensor(up_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
                std::vector<int> elementwiseOr;
                elementwiseOr.reserve(prox.size());
                std::transform(proximity_vec.begin(), proximity_vec.end(), prox.begin(), std::back_inserter(elementwiseOr), std::logical_or<>());

                proximity_vec = elementwiseOr;
            }
        }
        else if (m_vecKilobotsPositions[unKilobotID].GetY() < -1.0 * kDistance_threshold)
        {
            // std::cerr << "DOWN\n";
            if (proximity_vec.empty())
                proximity_vec = Proximity_sensor(down_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
            else
            {
                std::vector<int> prox = Proximity_sensor(up_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
                std::vector<int> elementwiseOr;
                elementwiseOr.reserve(prox.size());
                std::transform(proximity_vec.begin(), proximity_vec.end(), prox.begin(), std::back_inserter(elementwiseOr), std::logical_or<>());

                proximity_vec = elementwiseOr;
            }
        }

        proximity_sensor_dec = std::accumulate(proximity_vec.begin(), proximity_vec.end(), 0, [](int x, int y)
                                               { return (x << 1) + y; });

        // To turn off the wall avoidance decomment this
        // proximity_sensor_dec = 0;

        /** Print proximity values */
        // std::cerr << "kID:" << unKilobotID << " sensor ";
        // for (int item : proximity_vec)
        // {
        //     std::cerr << item << '\t';
        // }
        // std::cerr << std::endl;

        // std::cerr << "******Prox dec: " << proximity_sensor_dec << std::endl;
    }

    m_tALFKilobotMessage tKilobotMessage, tEmptyMessage, tMessage;
    bool bMessageToSend = false;

    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID] < m_fMinTimeBetweenTwoMsg)
    {
        // std::cerr << "Too many messages\n";
        return;
    }
    else
    {
        // std::cerr << "Mi rompo giusto prima di mandare messaggi\n";
        /* Compose the message for a kilobot */
        tKilobotMessage.m_sID = unKilobotID;                                  //ID of the receiver
        tKilobotMessage.m_sType = (UInt8)m_vecKilobotStates_ALF[unKilobotID]; //state
        tKilobotMessage.m_sData = 0;
        // std::cerr << "Forse il problema è qui\n";

        //entry msg when random walking
        if ((GetKilobotLedColor(c_kilobot_entity) != CColor::BLUE) &&
            (GetKilobotLedColor(c_kilobot_entity) != CColor::RED) &&
            ((int)m_vecKilobotStates_ALF[unKilobotID] == INSIDE_AREA))
        {
            // std::cerr << "sending INSIDE_AREA\n";
            bMessageToSend = true;
            tKilobotMessage.m_sData = (request[unKilobotID] & 0xFF); //requirement (timer) for the area where it is
            if (multiArea[whereis[unKilobotID]].Color == CColor::BLUE && adaptive_walk)
                tKilobotMessage.m_sData = tKilobotMessage.m_sData | (1 << 8);
            if (multiArea[whereis[unKilobotID]].Color == CColor::RED && adaptive_walk)
                tKilobotMessage.m_sData = tKilobotMessage.m_sData | (2 << 8);

            // std::cerr << "Timeout is " << request[unKilobotID] << ", bitset:" << std::bitset<10>(request[unKilobotID]) << std::endl;
            // std::cerr << "m_sData = " << std::bitset<10>(tKilobotMessage.m_sData) << std::endl;
        }

        //exit msg when inside
        if ((GetKilobotLedColor(c_kilobot_entity) == CColor::RED) &&
            ((int)m_vecKilobotStates_ALF[unKilobotID] == OUTSIDE_AREAS))
        {
            bMessageToSend = true;
            // std::cerr << "LEAVING,sending outside from inside\n";
        }

        if ((GetKilobotLedColor(c_kilobot_entity) == CColor::BLUE) && ((int)m_vecKilobotStates_ALF[unKilobotID] == OUTSIDE_AREAS))
        { //exit msg when task completed
            bMessageToSend = true;
            // std::cerr << "Task completed\n";
        }

        if ((fabs(m_vecKilobotsPositions[unKilobotID].GetX()) > kDistance_threshold || fabs(m_vecKilobotsPositions[unKilobotID].GetY()) > kDistance_threshold) &&
            ((int)m_vecKilobotStates_ALF[unKilobotID] != INSIDE_AREA))
        {
            // std::cerr << "sending COLLIDING\n";
            // std::cerr << "tKilobotMessage.m_sData" << (proximity_sensor_dec & 0x00FF) << "\n";
            tKilobotMessage.m_sData = (proximity_sensor_dec & 0x00FF);
            bMessageToSend = true;
        }

        // bMessageToSend = true; //use this line to send msgs always
    }

    std::cerr << (bMessageToSend == true ? "bMessageToSend TRUEEEEEE!!!\n" : "bMessageToSend FALSEEEEEE!!!\n");
    if (bMessageToSend)
    {
        std::cerr << "bMessageToSend TRUE!\n";
        m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;

        for (int i = 0; i < 9; ++i)
        {
            m_tMessages[unKilobotID].data[i] = 0;
        }

        tEmptyMessage.m_sID = 1023;
        tEmptyMessage.m_sType = 0;
        tEmptyMessage.m_sData = 0;
        for (int i = 0; i < 3; ++i)
        {
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
        //std::cout<<"payload: "<<tKilobotMessage.m_sData<<std::endl;
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity, &m_tMessages[unKilobotID]);
    }
    else
    {
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity, NULL);
    }
}

CColor CALFClientServer::GetFloorColor(const CVector2 &vec_position_on_plane)
{
    CColor cColor = CColor::WHITE;

    Real fKiloVision = Distance(vec_position_on_plane, m_vecKilobotsPositions[0]);
    if (fKiloVision < 0.04)
    {
        cColor = CColor(0, 0, 125, 0);
    }

    /* Draw areas until they are needed, once that task is completed the corresponding area disappears */
    for (int i = 0; i < multiArea.size(); i++)
    {
        if (multiArea[i].Completed == false)
        {
            Real fDistance = Distance(vec_position_on_plane, multiArea[i].Center);
            if (fDistance < multiArea[i].Radius)
            {
                cColor = multiArea[i].Color;
            }
        }
    }
    return cColor;
}

REGISTER_LOOP_FUNCTIONS(CALFClientServer, "kilobot_ALF_dhtf_loop_function")