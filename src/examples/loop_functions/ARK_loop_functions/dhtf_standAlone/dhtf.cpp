#include "dhtf.h"

namespace
{

    const double kEpsilon = 0.0001;

    // environment setup
    // const double kScaling = 1.0 / 4.0;      //for no scaling set kScaling=0.5
    const double kKiloDiameter = 0.033;
    double vArena_size = 0.5;
    double vDistance_threshold = vArena_size / 2.0 - 0.04;
    const double kTask_radius = 0.06;
    const int kHard_task_requirement = 4;
    const int kSoft_task_requirement = 2;

    // avoid to choose corner areas
    const std::vector<int> vForbidden({0, 3, 12, 15});

    // wall avoidance stuff
    const CVector2 left_direction(1.0, 0.0);
    const int kProximity_bits = 8;

    int internal_counter = 0;
}

dhtfCALF::dhtfCALF() : m_unDataAcquisitionFrequency(10)
{
    c_rng = CRandom::CreateRNG("argos");
}

void dhtfCALF::Reset()
{
    m_kiloOutput.close();
    m_kiloOutput.open(m_strKiloOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_areaOutput.close();
    m_areaOutput.open(m_strAreaOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_taskOutput.close();
    m_taskOutput.open(m_strTaskOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_elpsTimeoutOutput.close();
    m_elpsTimeoutOutput.open(m_strElpsTimeoutOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

void dhtfCALF::Destroy()
{
    m_kiloOutput.close();
    m_areaOutput.close();
    m_taskOutput.close();
    m_elpsTimeoutOutput.close();
}

void dhtfCALF::Init(TConfigurationNode &t_node)
{
    /* Initialize ALF*/
    CALF::Init(t_node);

    /* Other initializations: Varibales, Log file opening... */
    /*********** LOG FILES *********/
    m_kiloOutput.open(m_strKiloOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_areaOutput.open(m_strAreaOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_taskOutput.open(m_strTaskOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_elpsTimeoutOutput.open(m_strElpsTimeoutOutputFileName, std::ios_base::trunc | std::ios_base::out);

    /* Read parameters from .argos*/
    TConfigurationNode &tModeNode = GetNode(t_node, "extra_parameters");
    GetNodeAttributeOrDefault(tModeNode, "adaptive", adaptive_walk, false);
    GetNodeAttributeOrDefault(tModeNode, "adaptive_timeut", adaptive_timeout, false);

    if (adaptive_walk)
        std::cout << "Adaptive Walk\n";

    if (adaptive_timeout)
    {
        std::cout << "Adaptive timeout\n";
        m_vecKilobotsTimer = std::vector<int>(m_tKilobotEntities.size(), 6);
        // for (auto elem : m_vecKilobotsTimer)
        // {
        //     std::cout << elem << std::endl;
        // }
    }
    random_seed = GetSimulator().GetRandomSeed();
    //GetNodeAttribute(tModeNode, "random_seed", random_seed);
}

void dhtfCALF::SetupInitialKilobotStates()
{
    m_vecKilobotStates_ALF.resize(m_tKilobotEntities.size());
    m_vecKilobotsPositions.resize(m_tKilobotEntities.size());
    m_vecKilobotsColours.resize(m_tKilobotEntities.size());
    m_vecKilobotsOrientations.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);

    m_vecKilobotWalks_ALF.resize(m_tKilobotEntities.size());

    /* Compute the number of kilobots on the field*/
    for (UInt16 it = 0; it < m_tKilobotEntities.size(); it++)
    {
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }

    if (!adaptive_timeout)
    {
        m_vecKilobotsTimer = std::vector<int>(m_tKilobotEntities.size(), kTimerMultiplier);
    }

    /* Initialization of kilobots variables */
    m_vecKilobotsPositionTask = std::vector<int>(m_tKilobotEntities.size(), -1);
}

void dhtfCALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
    m_vecKilobotWalks_ALF[unKilobotID] = CONSTANT;
    m_vecLastTimeMessaged[unKilobotID] = -1000;

    /* Get a non-colliding random position within the circular arena */
    bool distant_enough = false;
    Real rand_angle;
    Real rand_x, rand_y;

    UInt16 maxTries = 999;
    UInt16 tries = 0;

    /* Get a random position and orientation for the kilobot initialized into a square but positioned in the circular arena */
    CQuaternion random_rotation;
    CRadians rand_rot_angle(c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue())));
    random_rotation.FromEulerAngles(rand_rot_angle, CRadians::ZERO, CRadians::ZERO);
    Real radius = m_ArenaStructure.Radius - m_ArenaStructure.Wall_width / 2 - kKiloDiameter / 2 - kEpsilon;
    do
    {
        rand_x = c_rng->Uniform(CRange<Real>(-radius, radius));
        rand_y = c_rng->Uniform(CRange<Real>(-radius, radius));
        distant_enough = MoveEntity(c_kilobot_entity.GetEmbodiedEntity(), CVector3(rand_x, rand_y, 0), random_rotation, false);

        if (tries == maxTries - 1)
        {
            std::cerr << "ERROR: too many tries and not an available spot for the area" << std::endl;
        }
    } while (!distant_enough || (rand_x * rand_x) + (rand_y * rand_y) > radius * radius);

    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsColours[unKilobotID] = argos::CColor::BLACK;
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
}

void dhtfCALF::SetupVirtualEnvironments(TConfigurationNode &t_tree)
{
    Real rand_x, rand_y;
    UInt16 maxTries = 999;
    UInt16 tries = 0;

    /* Read arena parameters */
    vArena_size = argos::CSimulator::GetInstance().GetSpace().GetArenaSize().GetX();
    vDistance_threshold = vArena_size / 2.0 - 0.04;
    std::cout << "Arena size: " << vArena_size << "\n";

    TConfigurationNode &tVirtualEnvironmentsNode = GetNode(t_tree, "environments");
    /* Get the node defining the walls parametres*/
    TConfigurationNode &t_VirtualWallsNode = GetNode(tVirtualEnvironmentsNode, "CircularWall");
    GetNodeAttribute(t_VirtualWallsNode, "radius", m_ArenaStructure.Radius);
    GetNodeAttribute(t_VirtualWallsNode, "width", m_ArenaStructure.Wall_width);
    GetNodeAttribute(t_VirtualWallsNode, "height", m_ArenaStructure.Wall_height);
    GetNodeAttribute(t_VirtualWallsNode, "walls", m_ArenaStructure.Wall_numbers);

    /* Get the node defining the task parameters*/
    TConfigurationNode &t_VirtualTaskNode = GetNode(tVirtualEnvironmentsNode, "VirtualTask");
    GetNodeAttribute(t_VirtualTaskNode, "desired_num_of_areas", desired_num_of_areas);
    GetNodeAttribute(t_VirtualTaskNode, "hard_tasks", hard_tasks);
    GetNodeAttribute(t_VirtualTaskNode, "reactivation_timer", kRespawnTimer);
    GetNodeAttribute(t_VirtualTaskNode, "soft_requirement", vSoftRequiredKilobots);
    GetNodeAttribute(t_VirtualTaskNode, "hard_requirement", vHardRequiredKilobots);
    GetNodeAttribute(t_VirtualTaskNode, "timeout_const", kTimerMultiplier);
    GetNodeAttributeOrDefault(t_VirtualTaskNode, "region_division", region_division, false);

    std::ostringstream entity_id;
    CRadians wall_angle = CRadians::TWO_PI / m_ArenaStructure.Wall_numbers;
    CVector3 wall_size(m_ArenaStructure.Wall_width, 2.0 * m_ArenaStructure.Radius * Tan(CRadians::PI / m_ArenaStructure.Wall_numbers), m_ArenaStructure.Wall_height);

    /* Wall positioning */
    for (UInt32 i = 0; i < m_ArenaStructure.Wall_numbers; i++)
    {
        entity_id.str("");
        entity_id << "wall_" << i;

        CRadians wall_rotation = wall_angle * i;
        // CVector3 wall_position((m_ArenaStructure.Radius) * Cos(wall_rotation), (m_ArenaStructure.Radius) * Sin(wall_rotation), 0);
        CVector3 wall_position(m_ArenaStructure.Radius * Cos(wall_rotation), m_ArenaStructure.Radius * Sin(wall_rotation), 0);
        CQuaternion wall_orientation;
        wall_orientation.FromEulerAngles(wall_rotation, CRadians::ZERO, CRadians::ZERO);

        CBoxEntity *wall = new CBoxEntity(entity_id.str(), wall_position, wall_orientation, false, wall_size);
        AddEntity(*wall);
    }

    /* Task positioning */
    std::cout << "desired_num_of_areas: " << desired_num_of_areas << "\n";
    multiArea.clear();

    for (int i = 0; i < desired_num_of_areas / 2; i++)
    {
        do
        {
            /** WARNING: remove constant and put variables */
            rand_x = c_rng->Uniform(CRange<Real>(0.07, 0.465));
            // rand_y = c_rng->Uniform(CRange<Real>(-0.475, 0.475));
            rand_y = c_rng->Uniform(CRange<Real>(-1.0 * (0.95 / 2.0 - 0.1), +1.0 * (0.95 / 2.0 - 0.1)));

            if (tries == maxTries - 1)
            {
                std::cerr << "ERROR: too many tries and not an available spot for the area" << std::endl;
            }
            tries += 1;
        } while (((rand_x * rand_x) + (rand_y * rand_y) > (m_ArenaStructure.Radius - 0.1) * (m_ArenaStructure.Radius - 0.1)) || !DistantEnoughTasks(CVector2(rand_x, rand_y)));

        multiArea.push_back(new Area(i, HARD_TASK, CVector2(rand_x, rand_y), kTask_radius, vHardRequiredKilobots, kRespawnTimer, kTimerMultiplier));

        multiArea.push_back(new Area(i + desired_num_of_areas / 2, SOFT_TASK, CVector2(-rand_x, -rand_y), kTask_radius, vSoftRequiredKilobots, kRespawnTimer, kTimerMultiplier));
    }

    if (!region_division)
    {
        hard_tasks_vec.clear();
        while (hard_tasks_vec.size() < hard_tasks)
        {
            UInt8 hardID;
            do
            {
                hardID = c_rng->Uniform(CRange<UInt32>(0, desired_num_of_areas));
            } while (std::find(hard_tasks_vec.begin(), hard_tasks_vec.end(), hardID) != hard_tasks_vec.end());

            hard_tasks_vec.push_back(hardID);
        }

        std::cout << "Hard task ids: ";
        for (auto elem : hard_tasks_vec)
        {
            std::cout << elem << ", ";
        }
        std::cout << "\n";

        for (int i = 0; i < multiArea.size(); i++)
        {
            if (std::find(hard_tasks_vec.begin(), hard_tasks_vec.end(), multiArea.at(i)->id) != hard_tasks_vec.end())
            {
                multiArea.at(i)->type = HARD_TASK;
                multiArea.at(i)->color = argos::CColor::RED;
                multiArea.at(i)->task_requirement = vHardRequiredKilobots;
            }
            else
            {
                multiArea.at(i)->type = SOFT_TASK;
                multiArea.at(i)->color = argos::CColor::BLUE;
                multiArea.at(i)->task_requirement = vSoftRequiredKilobots;
            }
        }
    }
}

bool dhtfCALF::DistantEnoughTasks(CVector2 some_position)
{
    for (size_t i = 0; i < multiArea.size(); i++)
    {
        Real fDistance = Distance(multiArea[i]->position, some_position);
        if (fDistance < (2.0 * multiArea[i]->radius + 0.02))
        {
            return false;
        }
    }

    return true;
}

void dhtfCALF::GetExperimentVariables(TConfigurationNode &t_tree)
{
    TConfigurationNode &tExperimentVariablesNode = GetNode(t_tree, "variables");
    GetNodeAttribute(tExperimentVariablesNode, "kilo_filename", m_strKiloOutputFileName);
    GetNodeAttribute(tExperimentVariablesNode, "area_filename", m_strAreaOutputFileName);
    GetNodeAttribute(tExperimentVariablesNode, "task_filename", m_strTaskOutputFileName);
    GetNodeAttribute(tExperimentVariablesNode, "timeout_filename", m_strElpsTimeoutOutputFileName);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);
}

CVector2 dhtfCALF::VectorRotation2D(Real angle, CVector2 vec)
{
    Real kx = (cos(angle) * vec.GetX()) + (-1.0 * sin(angle) * vec.GetY());
    Real ky = (sin(angle) * vec.GetX()) + (cos(angle) * vec.GetY());
    CVector2 rotated_vector(kx, ky);
    return rotated_vector;
}

std::vector<int> dhtfCALF::Proximity_sensor(CVector2 obstacle_direction, Real kOrientation, int num_sectors)
{
    double sector = M_PI_2 / (num_sectors / 2.0);
    std::vector<int> proximity_values;

    for (int i = 0; i < num_sectors; i++)
    {
        CVector2 sector_dir_start = VectorRotation2D((kOrientation + M_PI_2 - i * sector), left_direction);
        CVector2 sector_dir_end = VectorRotation2D((kOrientation + M_PI_2 - (i + 1) * sector), left_direction);

        if (obstacle_direction.DotProduct(sector_dir_start) >= 0.0 || obstacle_direction.DotProduct(sector_dir_end) >= 0.0)
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

void dhtfCALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
    m_vecKilobotsColours[unKilobotID] = GetKilobotLedColor(c_kilobot_entity);

    /** Print kilo state*/
    // for (int kID; kID < m_vecKilobotStates_ALF.size(); kID++)
    // {
    //     std::cout << "kID:" << kID << " state: ";
    //     switch (m_vecKilobotStates_ALF[kID])
    //     {
    //     case OUTSIDE_AREAS:
    //     {
    //         std::cout << "outside\n";
    //         break;
    //     }
    //     case INSIDE_AREA:
    //     {
    //         std::cout << "inside area\n";
    //         break;
    //     }
    //     case LEAVING:
    //     {
    //         std::cout << "leaving\n";
    //         break;
    //     }
    //     default:
    //         std::cout << "Error no state";
    //         break;
    //     }
    // }
}

void dhtfCALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity)
{
    /** WARNING:  delete from .h  and m_vecKilobotsPositionTask*/
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_tALFKilobotMessage tKilobotMessage, tEmptyMessage, tMessage;
    bool bMessageToSend = false;

    // for (int kID = 0; kID < m_vecKilobotsTimer.size(); kID++)
    // {
    //     std::cout << "kID:" << kID << " t:" << m_vecKilobotsTimer[kID] << '\t';
    // }
    // std::cout << std::endl;

    bool found = false;
    for (int i = 0; i < multiArea.size(); i++)
    {

        Area *completed_task = new Area(1000, 0, CVector2(1, 1), 1.0, 0, 0.0, 0); // random values

        if (multiArea[i]->completed || multiArea[i]->isCompleted(m_fTimeInSeconds, completed_task))
        {
            if (std::fabs(m_fTimeInSeconds - completed_task->completed_time) < 0.0000001)
            {

                m_taskOutput
                    << std::noshowpos
                    << std::setw(8) << std::setprecision(4) << std::setfill('0')
                    << completed_task->completed_time << '\t'
                    << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
                    << completed_task->id << '\t'
                    << std::internal << std::noshowpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
                    << completed_task->creation_time << '\t'
                    << std::internal << std::noshowpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
                    << completed_task->completed_time << '\t'
                    << std::noshowpos << std::setw(1) << std::setprecision(0)
                    << (completed_task->color == argos::CColor::RED ? 1 : 0) << '\t'
                    << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
                    << completed_task->kilobots_in_area.size() << '\t';
                for (size_t k = 0; k < completed_task->kilobots_in_area.size(); k++)
                {
                    m_taskOutput << completed_task->kilobots_in_area[k] << ",";

                    //area completed -> decrement kilobot internal timeout
                    if (adaptive_timeout && m_vecKilobotsTimer[completed_task->kilobots_in_area[k]] > 1)
                        m_vecKilobotsTimer[completed_task->kilobots_in_area[k]] -= 1;
                }
                m_taskOutput << std::endl;

                // /*Print completed area */
                // std::cout << "Area ID:" << completed_task->id << ", kilo_on_top: ";
                // for (auto kilobot : completed_task->kilobots_in_area)
                // {
                //     std::cout << kilobot << ", ";
                // }
                // std::cout << "\n";
            }
            /* Reactivation area check */
            if (!multiArea[i]->Respawn(m_fTimeInSeconds))
                continue;
        }

        if (multiArea[i]->isInside(m_vecKilobotsPositions[unKilobotID]))
        {
            if (m_vecKilobotsColours[unKilobotID] == CColor::RED)
            {
                // std::cout << "kID:" << unKilobotID;
                // switch (m_vecKilobotStates_ALF[unKilobotID])
                // {
                // case OUTSIDE_AREAS:
                // {
                //     std::cout << "outside\n";
                //     break;
                // }
                // case INSIDE_AREA:
                // {
                //     std::cout << "inside area\n";
                //     break;
                // }
                // case LEAVING:
                // {
                //     std::cout << "leaving\n";
                //     break;
                // }
                // default:
                //     std::cout << "Error no state";
                //     break;
                // }

                if (m_vecKilobotStates_ALF[unKilobotID] == INSIDE_AREA)
                {
                    m_elpsTimeoutOutput
                        << std::noshowpos
                        << std::setw(8) << std::setprecision(4) << std::setfill('0')
                        << m_fTimeInSeconds << '\t'
                        << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
                        << unKilobotID << '\t'
                        << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
                        << multiArea[i]->id << '\t'
                        << std::noshowpos << std::setw(1) << std::setprecision(0)
                        << (multiArea[i]->color == argos::CColor::RED ? 1 : 0)
                        << std::endl;

                    if (adaptive_timeout)
                        m_vecKilobotsTimer[unKilobotID] += 1;
                    // std::cout
                    //     << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
                    //     << unKilobotID << '\t'
                    //     << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
                    //     << multiArea[i]->id << '\t'
                    //     << std::noshowpos << std::setw(1) << std::setprecision(0)
                    //     << (multiArea[i]->color == argos::CColor::RED ? 1 : 0)
                    //     << std::endl;
                    // std::cout << unKilobotID << " elapsed LOG\n";
                }

                m_vecKilobotStates_ALF[unKilobotID] = LEAVING;
                multiArea[i]->kilobots_in_area.erase(std::remove(multiArea[i]->kilobots_in_area.begin(), multiArea[i]->kilobots_in_area.end(), unKilobotID),
                                                     multiArea[i]->kilobots_in_area.end());

                found = true;
                break;
            }
            else
            {
                if (std::find(multiArea[i]->kilobots_in_area.begin(), multiArea[i]->kilobots_in_area.end(), unKilobotID) == multiArea[i]->kilobots_in_area.end())
                    multiArea[i]->kilobots_in_area.push_back(unKilobotID);

                m_vecKilobotStates_ALF[unKilobotID] = INSIDE_AREA;
                m_vecKilobotWalks_ALF[unKilobotID] = (multiArea[i]->color == CColor::RED ? BROWNIAN : PERSISTENT);
                found = true;
                break;
            }
        }
        // outside
        else
        {
            multiArea[i]->kilobots_in_area.erase(std::remove(multiArea[i]->kilobots_in_area.begin(), multiArea[i]->kilobots_in_area.end(), unKilobotID),
                                                 multiArea[i]->kilobots_in_area.end());
        }
    }

    // if in no area
    if (!found)
    {
        m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
    }

    /********************************************/
    /********* WALL AVOIDANCE STUFF *************/
    /********************************************/
    UInt8 proximity_sensor_dec = 0; //8 bit proximity sensor as decimal
    Real fDistance = Distance(m_vecKilobotsPositions[unKilobotID], m_ArenaStructure.Center);
    if (fDistance > vDistance_threshold)
    {
        std::vector<int> proximity_vec;
        CRadians collision_angle = ATan2(m_vecKilobotsPositions[unKilobotID].GetY(), m_vecKilobotsPositions[unKilobotID].GetX());
        CVector2 collision_direction = CVector2(vDistance_threshold * Cos(collision_angle + CRadians(M_PI)), vDistance_threshold * Sin(collision_angle + CRadians(M_PI))).Normalize();

        // std::cout << "collision angle: " << collision_angle << std::endl;
        // std::cout << "collision direction: " << collision_direction << std::endl;
        proximity_vec = Proximity_sensor(collision_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), kProximity_bits);

        proximity_sensor_dec = std::accumulate(proximity_vec.begin(), proximity_vec.end(), 0, [](int x, int y)
                                               { return (x << 1) + y; });
        /* To turn off the wall avoidance decomment the following line */
        //proximity_sensor_dec = 0;

        /** Print proximity values */
        // std::cerr << "kID:" << unKilobotID << " sensor ";
        // for (int item : proximity_vec)
        // {
        //     std::cerr << item << '\t';
        // }
        // std::cerr << std::endl;

        // std::cout<<"******Prox dec: "<<proximity_sensor_dec<<std::endl;
    }

    // /********************************************/
    // /********* SENDING MESSAGE ******************/
    // /********************************************/
    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID] < m_fMinTimeBetweenTwoMsg)
    {
        return;
    }
    else
    {
        /* Compose the message for a kilobot */
        tKilobotMessage.m_sID = unKilobotID;                                //ID of the receiver
        tKilobotMessage.m_sType = (int)m_vecKilobotStates_ALF[unKilobotID]; //state
        tKilobotMessage.m_sData = 0;

        //entry msg when random walking
        if ((m_vecKilobotsColours[unKilobotID] != CColor::BLUE) &&
            (m_vecKilobotsColours[unKilobotID] != CColor::RED) &&
            ((int)m_vecKilobotStates_ALF[unKilobotID] == INSIDE_AREA))
        {
            bMessageToSend = true;
            tKilobotMessage.m_sData = (m_vecKilobotsTimer[unKilobotID] & 0xFF); //requirement (timer) for the area where it is
            if (adaptive_walk)
            {
                tKilobotMessage.m_sData = tKilobotMessage.m_sData | (m_vecKilobotWalks_ALF[unKilobotID] << 8);
            }
            // if (multiArea[m_vecKilobotsPositionTask[unKilobotID]]->color == CColor::BLUE && adaptive_walk)
            //     tKilobotMessage.m_sData = tKilobotMessage.m_sData | (1 << 8);
            // if (multiArea[m_vecKilobotsPositionTask[unKilobotID]]->color == CColor::RED && adaptive_walk)
            //     tKilobotMessage.m_sData = tKilobotMessage.m_sData | (2 << 8);
            // // std::cerr << "Timeout is " << request[unKilobotID] << "\n";
            // // std::cout << "m_sData = " << std::bitset<10>(tKilobotMessage.m_sData) << std::endl;
        }

        //exit msg when inside
        if ((m_vecKilobotsColours[unKilobotID] == CColor::RED) &&
            ((int)m_vecKilobotStates_ALF[unKilobotID] == OUTSIDE_AREAS))
        {
            bMessageToSend = true;
            // std::cerr<<"sending outside from inside\n";
        }

        if ((m_vecKilobotsColours[unKilobotID] == CColor::BLUE) && ((int)m_vecKilobotStates_ALF[unKilobotID] == OUTSIDE_AREAS))
        { //exit msg when task completed
            bMessageToSend = true;
            // std::cerr<<"sending outside from leaving\n";
        }

        Real fDistance = Distance(m_vecKilobotsPositions[unKilobotID], m_ArenaStructure.Center);
        if (fDistance > vDistance_threshold && ((int)m_vecKilobotStates_ALF[unKilobotID] != INSIDE_AREA))
        {
            tKilobotMessage.m_sData = proximity_sensor_dec;
            bMessageToSend = true;
            // std::cerr<<"sending COLLIDING\n";
        }
        // bMessageToSend=true;      //use this line to send msgs always
    }

    if (bMessageToSend)
    {

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
void dhtfCALF::PostExperiment()
{
    std::cout << "END\n";
}
void dhtfCALF::PostStep()
{
    // std::cout << "Time: " << m_fTimeInSeconds << std::endl;
    internal_counter += 1;
    if (internal_counter % m_unDataAcquisitionFrequency == 0 || internal_counter <= 1)
    {
        KiloLOG();
        AreaLOG();
    }
}
void dhtfCALF::AreaLOG()
{
    // std::cerr << "Logging arePosition\n";
    m_areaOutput
        << std::noshowpos << std::setw(4) << std::setprecision(0) << std::setfill('0')
        << m_fTimeInSeconds;
    for (size_t areaID = 0; areaID < multiArea.size(); areaID++)
    {
        m_areaOutput
            << '\t'
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << multiArea[areaID]->id << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << multiArea[areaID]->position.GetX() << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << multiArea[areaID]->position.GetY() << '\t'
            << std::noshowpos << std::setw(1) << std::setprecision(0)
            << (multiArea[areaID]->color == argos::CColor::RED ? 1 : 0) << '\t'
            << (multiArea[areaID]->completed == true ? 1 : 0) << '\t'
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << multiArea[areaID]->kilobots_in_area.size();
    }
    m_areaOutput << std::endl;
}
void dhtfCALF::KiloLOG()
{
    // std::cerr << "Logging kiloPosition\n";

    m_kiloOutput
        << std::noshowpos << std::setw(4) << std::setprecision(0) << std::setfill('0')
        << m_fTimeInSeconds;
    for (size_t kID = 0; kID < m_vecKilobotsPositions.size(); kID++)
    {
        m_kiloOutput
            // << kID << '\t'
            // << m_vecKilobotStates_ALF[kID] << '\t' //TODO: this should be the colour, but for now is the state
            // << m_vecKilobotsPositions[kID].GetX() << '\t'
            // << m_vecKilobotsPositions[kID].GetY() << '\t'
            // << m_vecKilobotsOrientations[kID] << '\t'
            // << m_vecKilobotStates_ALF[kID];

            // << std::noshowpos
            << '\t'
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << kID << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetX() << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetY() << '\t'
            << std::internal << std::showpos << std::setw(6) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsOrientations[kID].GetValue() << '\t'
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << m_vecKilobotsTimer[kID] << '\t'
            << std::noshowpos << std::setw(1) << std::setprecision(0)
            << m_vecKilobotStates_ALF[kID];
    }
    m_kiloOutput << std::endl;
}

CColor dhtfCALF::GetFloorColor(const CVector2 &vec_position_on_plane)
{
    CColor cColor = CColor::WHITE;

    // /** Draw the threshold for wall avoidance */
    // if (SquareDistance(vec_position_on_plane, CVector2(0.0, 0.0)) < pow(vDistance_threshold + 0.005, 2) &&
    //     SquareDistance(vec_position_on_plane, CVector2(0.0, 0.0)) > pow(vDistance_threshold - 0.005, 2))
    // {
    //     cColor = CColor::ORANGE;
    // }
    /* Draw areas until they are needed, once that task is completed the corresponding area disappears */
    for (int i = 0; i < multiArea.size(); i++)
    {
        if (multiArea[i]->completed == false)
        {
            Real fDistance = Distance(vec_position_on_plane, multiArea[i]->position);
            if (fDistance < multiArea[i]->radius)
            {
                cColor = multiArea[i]->color;
            }
        }

        // Real fKiloVision = Distance(vec_position_on_plane, m_vecKilobotsPositions[15]);
        // if (fKiloVision < 0.05)
        // {
        //     cColor = CColor(0, 0, 125, 0);
        // }
    }

    return cColor;
}
REGISTER_LOOP_FUNCTIONS(dhtfCALF, "ALF_dhtf_loop_function")
