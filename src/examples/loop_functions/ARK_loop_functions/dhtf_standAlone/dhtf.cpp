#include "dhtf.h"

namespace
{
    // environment setup
    // const double kScaling = 1.0 / 4.0;      //for no scaling set kScaling=0.5
    const double kKiloDiameter = 0.033;
    double vArena_size = 1.0;
    double vDistance_threshold = vArena_size / 2.0 - 2.0 * kKiloDiameter;
    const int max_area_id = 15;
    // const int kSoftRequiredKilobots = 2;
    // const int kHardRequiredKilobots = 6;

    // avoid to choose corner areas
    const std::vector<int> vForbidden({0, 3, 12, 15});

    // wall avoidance stuff
    const CVector2 up_direction(0.0, -1.0);
    const CVector2 down_direction(0.0, 1.0);
    const CVector2 left_direction(1.0, 0.0);
    const CVector2 right_direction(-1.0, 0.0);
    const int proximity_bits = 8;
    int internal_counter = 0;
}

dhtfCALF::dhtfCALF() : m_unDataAcquisitionFrequency(10)
{
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
    GetNodeAttribute(tModeNode, "timeout_const", kTimerMultiplier);
    GetNodeAttributeOrDefault(tModeNode, "adaptive", adaptive_walk, false);

    if (adaptive_walk)
        std::cout << "Adaptive Walk\n";

    random_seed = GetSimulator().GetRandomSeed();
    //GetNodeAttribute(tModeNode, "random_seed", random_seed);
    GetNodeAttribute(tModeNode, "desired_num_of_areas", desired_num_of_areas);
    GetNodeAttribute(tModeNode, "reactivation_timer", kRespawnTimer);
    GetNodeAttribute(tModeNode, "hard_tasks", hard_tasks);
    GetNodeAttribute(tModeNode, "soft_requirement", vSoftRequiredKilobots);
    GetNodeAttribute(tModeNode, "hard_requirement", vHardRequiredKilobots);

    InitializeVirtualEnvironment();
}

void dhtfCALF::SetupInitialKilobotStates()
{
    m_vecKilobotStates_ALF.resize(m_tKilobotEntities.size());
    m_vecKilobotsPositions.resize(m_tKilobotEntities.size());
    m_vecKilobotsColours.resize(m_tKilobotEntities.size());
    m_vecKilobotsOrientations.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);

    /* Compute the number of kilobots on the field*/
    for (UInt16 it = 0; it < m_tKilobotEntities.size(); it++)
    {
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }

    /* Initialization of kilobots variables */
    m_vecKilobotsTimeRequest = std::vector<UInt8>(m_tKilobotEntities.size(), 0);
    m_vecKilobotsPositionTask = std::vector<int>(m_tKilobotEntities.size(), -1);
}

void dhtfCALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
    m_vecLastTimeMessaged[unKilobotID] = -1000;

    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsColours[unKilobotID] = argos::CColor::BLACK;
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
}

void dhtfCALF::SetupVirtualEnvironments(TConfigurationNode &t_tree)
{
    /* Read arena parameters */
    vArena_size = argos::CSimulator::GetInstance().GetSpace().GetArenaSize().GetX();
    vDistance_threshold = vArena_size / 2.0 - 2.0 * kKiloDiameter;
    // std::cout << "Arena size: " << vArena_size << "\n";

    // std::cout << "SetupVirtualEnvironments\n";
    TConfigurationNode &tVirtualEnvironmentsNode = GetNode(t_tree, "environments");
    TConfigurationNodeIterator itAct;

    /* Compute number of areas on the field*/
    int num_of_areas = 0;
    for (itAct = itAct.begin(&tVirtualEnvironmentsNode); itAct != itAct.end(); ++itAct)
    {
        num_of_areas += 1;
    }

    // std::cout << "Num of areas:" << num_of_areas << std::endl;
    /* Build the structure with areas data */
    multiArea.resize(num_of_areas);

    int i = 0;
    for (itAct = itAct.begin(&tVirtualEnvironmentsNode); itAct != itAct.end(); ++itAct, i++)
    {
        std::string s = std::to_string(i);
        std::string var = std::string("Area") + std::string(s);
        TConfigurationNode &t_VirtualClusteringHubNode = GetNode(tVirtualEnvironmentsNode, var);
        GetNodeAttribute(t_VirtualClusteringHubNode, "position", multiArea[i].position);
        GetNodeAttribute(t_VirtualClusteringHubNode, "radius", multiArea[i].radius);
    }
    /* White set as default color*/
    for (int ai = 0; ai < num_of_areas; ai++)
    {
        multiArea[ai].id = ai;
        multiArea[ai].color = argos::CColor::WHITE;
        multiArea[ai].contained = 0;
        multiArea[ai].completed = false;
    }
}

void dhtfCALF::InitializeVirtualEnvironment()
{

    /************************/
    /* Select desired areas */
    /************************/
    srand(random_seed);

    /* GENERATE RANDOM IDs AND RANDOM HARD TASK */
    std::default_random_engine re;
    re.seed(random_seed);

    std::cout << "Desired num of areas:" << desired_num_of_areas << std::endl;

    /* Generate active areas ID */
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

    /* Generate hard task for the server */
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

    /** Dynamic initialisation */
    // multiArea.clear();

    // double white_space = kScaling * 4.0 * kKiloDiameter;
    // double radius = (((2.0 * kScaling - white_space) / 4.0) - white_space) / 2.0;
    // std::cout << "radius " << radius << std::endl;
    // for (int areaID = 0; areaID < 16; ++areaID)
    // {
    //     if (std::find(activated_areas.begin(), activated_areas.end(), areaID) != activated_areas.end())
    //     {
    //         CVector2 areaPos((1.0 + 2.0 * (areaID % 4)) * radius + (1.0 + (areaID % 4)) * white_space, (1.0 + floor(areaID / 4) * 2.0) * radius + (1.0 + floor(areaID / 4)) * white_space);
    //         areaPos -= CVector2(kArena_size / 2.0, kArena_size / 2.0);
    //         SVirtualArea vArea;
    //         vArea.id = areaID;
    //         vArea.position = areaPos;
    //         vArea.completed = false;
    //         vArea.radius = radius;
    //         vArea.contained = 0;     // how many kilobots are partecipating at the task
    //         vArea.completed = false; //"true" if the task is completed
    //         vArea.creationTime = 0.0;
    //         vArea.completitionTime = 0.0;
    //         if (std::find(hard_tasks_vec.begin(), hard_tasks_vec.end(), areaID) != hard_tasks_vec.end())
    //         {
    //             vArea.color = argos::CColor::RED;
    //         }

    //         else
    //         {
    //             vArea.color = argos::CColor::BLUE;
    //         }

    //         multiArea.push_back(vArea);
    //     }
    // }

    /** Static initialisation */
    //Remove the extra multiArea loaded from .argos file
    for (int i = 0; i < multiArea.size(); i++)
    {
        // fill server own colors
        if (std::find(activated_areas.begin(), activated_areas.end(), multiArea[i].id) == activated_areas.end())
        {
            multiArea.erase(multiArea.begin() + i);
            i -= 1;
        }
        else
        {
            if (std::find(hard_tasks_vec.begin(), hard_tasks_vec.end(), multiArea[i].id) != hard_tasks_vec.end())
            {
                multiArea[i].color = argos::CColor::RED;
            }
            else
            {
                multiArea[i].color = argos::CColor::BLUE;
            }
        }
    }

    for (auto a : multiArea)
    {
        std::cout << "id:" << a.id
                  << ", color:" << (a.color == argos::CColor::BLUE ? "blue" : "red")
                  << ", position:(" << a.position.GetX() << "," << a.position.GetY() << ")\n";
    }
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

void dhtfCALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
    m_vecKilobotsColours[unKilobotID] = GetKilobotLedColor(c_kilobot_entity);

    switch (m_vecKilobotStates_ALF[unKilobotID])
    {
    case OUTSIDE_AREAS:
    {
        /** WARNING: hard and soft tasks have the same time requirement */
        /* Check if the kilobot is entered in a task area */
        for (int i = 0; i < multiArea.size(); i++)
        {
            Real fDistance = Distance(m_vecKilobotsPositions[unKilobotID], multiArea[i].position);
            if ((fDistance < (multiArea[i].radius * 1.0)) && (multiArea[i].completed == false)) //*1.0 is a threshold, to include the boarder increase it
            {
                m_vecKilobotStates_ALF[unKilobotID] = INSIDE_AREA;
                // std::cout<< "inside area = "<< multiArea[i].id << std::endl;
                /* Check the area color to understand the requirements of the task */
                if (multiArea[i].color == argos::CColor::RED)
                {
                    m_vecKilobotsTimeRequest[unKilobotID] = kTimerMultiplier;
                }
                if (multiArea[i].color == argos::CColor::BLUE)
                {
                    m_vecKilobotsTimeRequest[unKilobotID] = kTimerMultiplier;
                }

                m_vecKilobotsPositionTask[unKilobotID] = i;
                multiArea[i].contained += 1;
                // std::cerr << "kID:" << unKilobotID << " entered in " << multiArea[i].id << " which contains:" << multiArea[i].contained << " robots\n";
            }
        }
        break;
    }
    case INSIDE_AREA:
    {
        /* Check if the kilobot timer for colaboratos is expired */
        if (m_vecKilobotsColours[unKilobotID] == argos::CColor::RED)
        {
            m_vecKilobotStates_ALF[unKilobotID] = LEAVING;
            multiArea[m_vecKilobotsPositionTask[unKilobotID]].contained -= 1;

            m_elpsTimeoutOutput
                << std::noshowpos
                << std::setw(8) << std::setprecision(4) << std::setfill('0')
                << m_fTimeInSeconds << '\t'
                << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
                << unKilobotID << '\t'
                << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
                << multiArea[m_vecKilobotsPositionTask[unKilobotID]].id << '\t'
                << std::noshowpos << std::setw(1) << std::setprecision(0)
                << (multiArea[m_vecKilobotsPositionTask[unKilobotID]].color == argos::CColor::RED ? 1 : 0)
                << std::endl;

            // std::cout
            //     << std::noshowpos
            //     << std::setw(8) << std::setprecision(4) << std::setfill('0')
            //     << "time: " << m_fTimeInSeconds << '\t'
            //     << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            //     << "areaID: " << multiArea[m_vecKilobotsPositionTask[unKilobotID]].id << '\t'
            //     << std::noshowpos << std::setw(1) << std::setprecision(0)
            //     << "color: " << (multiArea[m_vecKilobotsPositionTask[unKilobotID]].color == argos::CColor::RED ? "red" : "blue")
            //     << std::endl;
        }
        /* Else check if the task has been completed */
        else if (multiArea[m_vecKilobotsPositionTask[unKilobotID]].completed == true)
        {
            m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
            multiArea[m_vecKilobotsPositionTask[unKilobotID]].contained = 0;
            m_vecKilobotsPositionTask[unKilobotID] = -1;
        }
        else if (Distance(m_vecKilobotsPositions[unKilobotID], multiArea[m_vecKilobotsPositionTask[unKilobotID]].position) > multiArea[m_vecKilobotsPositionTask[unKilobotID]].radius)
        {
            m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
            multiArea[m_vecKilobotsPositionTask[unKilobotID]].contained -= 1;
            m_vecKilobotsPositionTask[unKilobotID] = -1;
        }

        break;
    }
    case LEAVING:
    {
        /* Case in which the robot is inside an area but internal timeout is expired */
        Real fDistance = Distance(m_vecKilobotsPositions[unKilobotID], multiArea[m_vecKilobotsPositionTask[unKilobotID]].position);
        /* Check when the robot is back outside  */
        if (fDistance > (multiArea[m_vecKilobotsPositionTask[unKilobotID]].radius) || multiArea[m_vecKilobotsPositionTask[unKilobotID]].completed == true)
        {
            m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
            m_vecKilobotsPositionTask[unKilobotID] = -1;
        }
        break;
    }
    }

    // switch (m_vecKilobotStates_ALF[unKilobotID]) {
    //     case OUTSIDE_AREAS : {
    //         std::cout<<"Outside\n";
    //     break;
    //     }
    //     case INSIDE_AREA : {
    //         std::cout<<"Inside\n";
    //     break;
    //     }
    //     case LEAVING : {
    //         std::cout<<"Leaving\n";
    //     break;
    //     }
    //     default:
    //         std::cout<<"Error no state";
    //     break;
    // }
}

void dhtfCALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_tALFKilobotMessage tKilobotMessage, tEmptyMessage, tMessage;
    bool bMessageToSend = false;

    /********************************************/
    /********* WALL AVOIDANCE STUFF *************/
    /********************************************/
    UInt8 proximity_sensor_dec = 0; //8 bit proximity sensor as decimal
    // std::cerr<<unKilobotID<<'\t'<<m_vecKilobotsPositions[unKilobotID]<<std::endl;
    if (fabs(m_vecKilobotsPositions[unKilobotID].GetX()) > vDistance_threshold ||
        fabs(m_vecKilobotsPositions[unKilobotID].GetY()) > vDistance_threshold)
    {
        std::vector<int> proximity_vec;

        if (m_vecKilobotsPositions[unKilobotID].GetX() > vDistance_threshold)
        {
            // std::cerr<<"RIGHT\n";
            proximity_vec = Proximity_sensor(right_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
        }
        else if (m_vecKilobotsPositions[unKilobotID].GetX() < -1.0 * vDistance_threshold)
        {
            // std::cerr<<"LEFT\n";
            proximity_vec = Proximity_sensor(left_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), proximity_bits);
        }

        if (m_vecKilobotsPositions[unKilobotID].GetY() > vDistance_threshold)
        {
            // std::cerr<<"UP\n";
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
        else if (m_vecKilobotsPositions[unKilobotID].GetY() < -1.0 * vDistance_threshold)
        {
            // std::cerr<<"DOWN\n";
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
        // To turn off the wall avoidance decomment the following line
        //proximity_sensor_dec = 0;

        /** Print proximity values */
        // std::cerr<<"kID:"<< unKilobotID <<" sensor ";
        // for(int item : proximity_vec)
        // {
        //     std::cerr<< item <<'\t';
        // }
        // std::cerr<<std::endl;

        // std::cout<<"******Prox dec: "<<proximity_sensor_dec<<std::endl;
    }

    /********************************************/
    /********* ENVIRONMENT UPDATE ***************/
    /********************************************/
    /* Reactivation area check */
    for (int i = 0; i < multiArea.size(); i++)
    {
        if ((multiArea[i].completed == true) &&
            (m_fTimeInSeconds - multiArea[i].completitionTime >= kRespawnTimer))
        {
            std::cout << "m_fTimeInSeconds " << m_fTimeInSeconds
                      << " multiArea[i].completitionTime " << multiArea[i].completitionTime
                      << " kRespawnTimer " << kRespawnTimer
                      << std::endl;
            std::cout << "Respawn area " << multiArea[i].id << std::endl;
            multiArea[i].completed = false;
            multiArea[i].contained = 0;
            multiArea[i].creationTime = m_fTimeInSeconds;
        }
    }

    /* Task completeness check */
    for (int j = 0; j < multiArea.size(); j++)
    {
        // std::cout << "Area " << multiArea[j].id << "contains " << multiArea[j].contained << "robots" << std::endl;
        if (multiArea[j].completed == false)
        {
            if ((multiArea[j].color == argos::CColor::RED) && (multiArea[j].contained >= vHardRequiredKilobots))
            {
                multiArea[j].completed = true;
                std::cout << m_fTimeInSeconds << "\tareaID " << multiArea[j].id << "\tRED task completed" << std::endl;
            }
            if ((multiArea[j].color == argos::CColor::BLUE) && (multiArea[j].contained >= vSoftRequiredKilobots))
            {
                multiArea[j].completed = true;
                std::cout << m_fTimeInSeconds << "\tareaID " << multiArea[j].id << "\tBLUE task completed" << std::endl;
            }

            if (multiArea[j].completed == true)
            {
                multiArea[j].completitionTime = m_fTimeInSeconds;
                m_taskOutput
                    << std::noshowpos
                    << std::setw(8) << std::setprecision(4) << std::setfill('0')
                    << multiArea[j].completitionTime << '\t'
                    << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
                    << multiArea[j].id << '\t'
                    << std::internal << std::noshowpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
                    << multiArea[j].creationTime << '\t'
                    << std::internal << std::noshowpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
                    << multiArea[j].completitionTime << '\t'
                    << std::noshowpos << std::setw(1) << std::setprecision(0)
                    << (multiArea[j].color == argos::CColor::RED ? 1 : 0) << '\t'
                    << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
                    << multiArea[j].contained
                    << std::endl;
            }
        }
    }

    /********************************************/
    /********* SENDING MESSAGE ******************/
    /********************************************/
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
            tKilobotMessage.m_sData = (m_vecKilobotsTimeRequest[unKilobotID] & 0xFF); //requirement (timer) for the area where it is
            if (multiArea[m_vecKilobotsPositionTask[unKilobotID]].color == CColor::BLUE && adaptive_walk)
                tKilobotMessage.m_sData = tKilobotMessage.m_sData | (1 << 8);
            if (multiArea[m_vecKilobotsPositionTask[unKilobotID]].color == CColor::RED && adaptive_walk)
                tKilobotMessage.m_sData = tKilobotMessage.m_sData | (2 << 8);
            // std::cerr << "Timeout is " << m_vecKilobotsTimeRequest[unKilobotID] << "\n";
            // std::cout << "m_sData = " << std::bitset<10>(tKilobotMessage.m_sData) << std::endl;
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

        if ((fabs(m_vecKilobotsPositions[unKilobotID].GetX()) > vDistance_threshold || fabs(m_vecKilobotsPositions[unKilobotID].GetY()) > vDistance_threshold) &&
            ((int)m_vecKilobotStates_ALF[unKilobotID] != INSIDE_AREA))
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
        << m_fTimeInSeconds << '\t';
    for (size_t areaID = 0; areaID < multiArea.size(); areaID++)
    {
        m_areaOutput
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << multiArea[areaID].id << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << multiArea[areaID].position.GetX() << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << multiArea[areaID].position.GetY() << '\t'
            << std::noshowpos << std::setw(1) << std::setprecision(0)
            << (multiArea[areaID].color == argos::CColor::RED ? 1 : 0) << '\t'
            << (multiArea[areaID].completed == true ? 1 : 0) << '\t'
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << multiArea[areaID].contained;
        // << multiArea[areaID].
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
            << std::noshowpos << std::setw(1) << std::setprecision(0)
            << m_vecKilobotStates_ALF[kID] << '\t' //TODO: this should be the colour, but for now is the state
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetX() << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetY() << '\t'
            << std::internal << std::showpos << std::setw(6) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsOrientations[kID].GetValue() << '\t'
            << std::noshowpos << std::setw(1) << std::setprecision(0)
            << m_vecKilobotStates_ALF[kID];
    }
    m_kiloOutput << std::endl;
}

CColor dhtfCALF::GetFloorColor(const CVector2 &vec_position_on_plane)
{
    CColor cColor = CColor::WHITE;
    /* Draw areas until they are needed, once that task is completed the corresponding area disappears */
    for (int i = 0; i < multiArea.size(); i++)
    {
        if (multiArea[i].completed == false)
        {
            Real fDistance = Distance(vec_position_on_plane, multiArea[i].position);
            if (fDistance < multiArea[i].radius)
            {
                cColor = multiArea[i].color;
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
