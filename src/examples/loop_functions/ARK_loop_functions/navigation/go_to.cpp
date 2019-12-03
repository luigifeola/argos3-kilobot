#include "go_to.h"

/****************************************/
/****************************************/

CNavigationALF::CNavigationALF() :
    m_unDataAcquisitionFrequency(10),
    num_robots_in_int_pose(0),
    num_robots_with_discovery(0),
    num_robots_with_info(0)
    {
        c_rng = CRandom::CreateRNG("argos");
    }

/****************************************/
/****************************************/

CNavigationALF::~CNavigationALF(){
}

/****************************************/
/****************************************/

void CNavigationALF::Init(TConfigurationNode& t_node) {
    /* Initialize ALF*/
    CALF::Init(t_node);
    /* Other initializations: Varibales, Log file opening... */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void CNavigationALF::Reset() {
    /* Close data file */
    m_cOutput.close();
    /* Reopen the file, erasing its contents */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void CNavigationALF::Destroy() {
    /* Close data file */
    m_cOutput.close();
}

/****************************************/
/****************************************/

void CNavigationALF::SetupInitialKilobotStates() {
    m_vecKilobotStates.resize(m_tKilobotEntities.size());
    arrived.resize(m_tKilobotEntities.size());
    /* Variables for go_to initial position */
    // index_vector.resize(m_tKilobotEntities.size());
    m_vecKilobotsPositions.resize(m_tKilobotEntities.size());
    m_vecDesInitKilobotPosition.resize(m_tKilobotEntities.size());
    m_vecDesInitKilobotOrientation.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);

    /*********************************************
     * 
     * BE CAREFUL HERE
     * 
     *********************************************/
    m_fMinTimeBetweenTwoMsg = 0;
    m_vecLastTimeMessaged.clear();
    
    
    
    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        /* Setup the virtual states of a kilobot(e.g. has food state)*/
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }
    
    GreedyAssociation(m_vecKilobotsPositions, m_vecDesInitKilobotPosition);


    message_t m_tArkBroadcastMessage;// = new message_t();
    /* Prepare the inividual kilobot's message */
    m_tArkBroadcastMessage.type = 255;  // using type 255 to signal ark broadcast messages
    /* Fill up the ARK message */
    m_tArkBroadcastMessage.data[0] = (UInt8)(crw_exponent*100); 
    m_tArkBroadcastMessage.data[1] = (UInt8)(levy_exponent*100); 
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(m_tKilobotEntities,&m_tArkBroadcastMessage);
}

/****************************************/
/****************************************/

void CNavigationALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    /* The kilobots begins outside the clustering hub*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    m_vecKilobotStates[unKilobotID] = NOT_TARGET_FOUND;
    arrived[unKilobotID] = false;
    m_vecLastTimeMessaged[unKilobotID] = -1000;

    /* Get a non-colliding random position within the circular arena */
    bool distant_enough = false;
    Real rand_angle;
    CVector2 rand_displacement, rand_init_pos;
    CVector3 rand_pos;

    UInt16 maxTries = 999;
    UInt16 tries = 0;

    /* Get a random orientation for the kilobot */
    CQuaternion random_rotation;
    CRadians rand_rot_angle(c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue())));
    random_rotation.FromEulerAngles(rand_rot_angle, CRadians::ZERO, CRadians::ZERO);

    do {
        rand_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
        rand_displacement.SetX(c_rng->Uniform(CRange<Real>(0, m_WallStructure.circular_arena_radius)));
        rand_displacement.SetY(c_rng->Uniform(CRange<Real>(0, m_WallStructure.circular_arena_radius)));
        rand_pos = CVector3(rand_displacement.GetX()*sin(rand_angle),rand_displacement.GetY()*cos(rand_angle),0);
        distant_enough = MoveEntity(c_kilobot_entity.GetEmbodiedEntity(), rand_pos, random_rotation, false);
        
        if(tries == maxTries-1) {
        std::cerr << "ERROR: too many tries and not an available spot for the area" << std::endl;
        }
    } while(!distant_enough);
    
    /**
     * 
     * RANDOM GENERATED INITIAL DESIRED POSITION
     * 
     */
    
    rand_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
    // do
    rand_displacement.SetX(c_rng->Uniform(CRange<Real>(0, m_WallStructure.circular_arena_radius)));
    rand_displacement.SetY(c_rng->Uniform(CRange<Real>(0, m_WallStructure.circular_arena_radius)));
    rand_init_pos = CVector2(rand_displacement.GetX()*sin(rand_angle),rand_displacement.GetY()*cos(rand_angle));
    // while each pos distant enough each others
    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecDesInitKilobotPosition[unKilobotID] = rand_init_pos;
    rand_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));     // Angle in [-pi,pi]
    m_vecDesInitKilobotOrientation[unKilobotID] = CRadians(rand_angle);

    // GreedyAssociation(m_vecKilobotsPositions, m_vecDesInitKilobotPosition);


    SVirtualArea temp_area2;
    temp_area2.Center = CVector2(rand_init_pos.GetX(), rand_init_pos.GetY());
    temp_area2.Radius = 0.05;
    temp_area2.Color = CColor::GREEN;
    m_TargetAreas.push_back(temp_area2);
}

/****************************************/
/****************************************/

void CNavigationALF::SetupVirtualEnvironments(TConfigurationNode& t_tree){
    // generate circular arena
    /* Get the virtual environments node from .argos file*/
    TConfigurationNode& tVirtualEnvironmentsNode = GetNode(t_tree,"environments");
    /* Get the node defining the walls parametres*/
    TConfigurationNode& t_VirtualWallsNode = GetNode(tVirtualEnvironmentsNode,"Perimeter");
    GetNodeAttribute(t_VirtualWallsNode, "radius", m_WallStructure.circular_arena_radius);
    GetNodeAttribute(t_VirtualWallsNode, "width", m_WallStructure.circular_arena_width);
    GetNodeAttribute(t_VirtualWallsNode, "height", m_WallStructure.circular_arena_height);
    GetNodeAttribute(t_VirtualWallsNode, "walls", m_WallStructure.circular_arena_walls);
    

    std::ostringstream entity_id;
    CRadians wall_angle = CRadians::TWO_PI / m_WallStructure.circular_arena_walls;
    CVector3 wall_size(m_WallStructure.circular_arena_width, 2.0 * m_WallStructure.circular_arena_radius * Tan(CRadians::PI / m_WallStructure.circular_arena_walls), m_WallStructure.circular_arena_height);

    for (UInt32 i = 0; i < m_WallStructure.circular_arena_walls; i++) {
        entity_id.str("");
        entity_id << "wall_" << i;

        CRadians wall_rotation = wall_angle * i;
        CVector3 wall_position(m_WallStructure.circular_arena_radius * Cos(wall_rotation), m_WallStructure.circular_arena_radius * Sin(wall_rotation), 0);
        CQuaternion wall_orientation;
        wall_orientation.FromEulerAngles(wall_rotation, CRadians::ZERO, CRadians::ZERO);

        CBoxEntity *wall = new CBoxEntity(entity_id.str(), wall_position, wall_orientation, false, wall_size);
        AddEntity(*wall);
    }

    
    /* Get the node defining the clustering hub parametres*/
    TConfigurationNode& t_VirtualClusteringHubNode = GetNode(tVirtualEnvironmentsNode,"Area");
    GetNodeAttribute(t_VirtualClusteringHubNode, "position", m_sClusteringHub.Center);
    GetNodeAttribute(t_VirtualClusteringHubNode, "radius", m_sClusteringHub.Radius);
    GetNodeAttribute(t_VirtualClusteringHubNode, "color", m_sClusteringHub.Color);

    
}

/****************************************/
/****************************************/

void CNavigationALF::GetExperimentVariables(TConfigurationNode& t_tree){
    /* Get the experiment variables node from the .argos file */
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");
    /* Get the crwlevy exponents */
    GetNodeAttribute(tExperimentVariablesNode, "crw", crw_exponent);
    GetNodeAttribute(tExperimentVariablesNode, "levy", levy_exponent);
    /* Get the output datafile name and open it */
    GetNodeAttribute(tExperimentVariablesNode, "datafilename", m_strOutputFileName);
    /* Get the frequency of data saving */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
    /* Get the frequency of updating the environment plot */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);
    /* Get the time for one kilobot message */
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);
}


/****************************************/
/****************************************/

void CNavigationALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
    /* Update the state of the kilobots (target not found, found directly or communicated)*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    CVector2 cKilobotPosition=GetKilobotPosition(c_kilobot_entity);
    Real fDistance = Distance(cKilobotPosition, m_sClusteringHub.Center);

    if(m_vecKilobotStates[unKilobotID] == TARGET_FOUND){
        return;
    }

    if(fDistance<(m_sClusteringHub.Radius)){//*0.9
        m_vecKilobotStates[unKilobotID]=TARGET_FOUND;
        std::map<UInt16,std::pair<UInt32,UInt32>>::iterator itr = m_KilobotResults.find(unKilobotID);
        if (itr==m_KilobotResults.end())
        {
            UInt32 simclock = GetSpace().GetSimulationClock();
            m_KilobotResults.insert(std::pair<UInt16,std::pair<UInt32,UInt32>> (unKilobotID,std::pair<UInt32,UInt32>(simclock,simclock)));
            num_robots_with_discovery+=1;
            num_robots_with_info+=1;
        }
        else{
            num_robots_with_discovery+=1;
            if(itr->second.first == 0){
                itr->second.first = GetSpace().GetSimulationClock();
            }
        }
        
        
    }
    else if(GetKilobotLedColor(c_kilobot_entity) == CColor::GREEN){
        m_vecKilobotStates[unKilobotID]=TARGET_COMMUNICATED;
        std::map<UInt16,std::pair<UInt32,UInt32>>::const_iterator itr = m_KilobotResults.find(unKilobotID);
        if (itr!=m_KilobotResults.end()){
            return;
        }
        else{
            UInt32 simclock = GetSpace().GetSimulationClock();
            m_KilobotResults.insert(std::pair<UInt16,std::pair<UInt32,UInt32>> (unKilobotID,std::pair<UInt32,UInt32>(0,simclock)));
            num_robots_with_info+=1;
        }
        
    }
    
}

/****************************************/
/****************************************/
void CNavigationALF::GreedyAssociation(std::vector<CVector2> actual_pos, std::vector<CVector2> desired_pos)
{
    // std::cerr<<"KBOTs POSITION\n";
    // for(int k = 0; k<actual_pos.size(); k++)
    // {
    //     std::cerr << actual_pos[k]<< std::endl;
    // }    
    // std::cerr<<"\n\n\n\n\n";

    // std::cerr<<"DESIRED INITIAL POSITION\n";
    // for(int y = 0; y<desired_pos.size(); y++)
    // {
    //     std::cerr << desired_pos[y]<< std::endl;
    // }    
    // std::cerr<<"\n\n\n\n\n";

    std::vector<Real> v_distances;
    
    do
    {
        Real distance = 1000;
        std::pair <UInt32,UInt32> idxs;
        for(UInt32 i = 0; i < actual_pos.size(); i++ )
        {
            for (UInt32 j=0; j < desired_pos.size(); j++)
            {
                // std::cerr<<"Distances k_pos, des_pos: "<<i<<","<<j<<"\t"<< SquareDistance(actual_pos[i], desired_pos[j])<<std::endl;
                if (distance > SquareDistance(actual_pos[i], desired_pos[j]))
                {
                    distance = SquareDistance(actual_pos[i], desired_pos[j]);
                    idxs = std::make_pair(i,j);
                }

            }

        }
            index_vector.push_back(idxs);
            v_distances.push_back(distance);

            actual_pos[idxs.first].SetX(std::numeric_limits<Real>::max());
            actual_pos[idxs.first].SetY(std::numeric_limits<Real>::max());
            desired_pos[idxs.second].SetX(-1*std::numeric_limits<Real>::max());
            desired_pos[idxs.second].SetY(-1*std::numeric_limits<Real>::max());
    } while (index_vector.size() != actual_pos.size());

    std::sort(index_vector.begin(), index_vector.end());

    for(int z = 0; z<index_vector.size(); z++)
    {
        std::cerr << index_vector[z].first<<","<<index_vector[z].second<<"\t"<<v_distances[z]<< std::endl;
    }    
    std::cerr<<"\n\n\n\n\n";

}

/****************************************/
/****************************************/

void CNavigationALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity)
{    
    /* Get the kilobot ID and state (Only Position in this example) */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);

    /* check if enough time has passed from the last message otherwise*/
    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID]< m_fMinTimeBetweenTwoMsg)
    {
        return; 
    }

    /*Motion command to send to the kilobot*/
    command cmd;
    message_t msg;
    msg.type = 254;
        
    /* Variable for Kilobots position and orientation */
    CVector2 kiloPos = GetKilobotPosition(c_kilobot_entity);
    CVector2 init_d_position = m_vecDesInitKilobotPosition[index_vector[unKilobotID].second];

    CRadians kiloOrientation = GetKilobotOrientation(c_kilobot_entity);
    CRadians desired_orientation = m_vecDesInitKilobotOrientation[index_vector[unKilobotID].second];
    CRadians angle_offset = desired_orientation - kiloOrientation;               //because both angles are defined in [-pi,pi]
    

    if (!arrived[unKilobotID])
    {
        if(SquareDistance(init_d_position , kiloPos) <= distThreshold)
        {
            cmd = STOP;
            arrived[unKilobotID] = true;            
        }

        else 
        {
            CRadians pathOrientation = ATan2(init_d_position.GetY() - kiloPos.GetY(), init_d_position.GetX()-kiloPos.GetX()) - GetKilobotOrientation(c_kilobot_entity);
            if(pathOrientation.GetAbsoluteValue() < 0.01)
            {
                cmd = FORWARD;
            }
            else
            {
                pathOrientation.SignedNormalize(); //map angle in [-pi,pi]
                
                if (pathOrientation < CRadians::ZERO)
                {
                    cmd = RIGHT;
                } 
                else 
                {
                    cmd = LEFT;
                }      

                // std::cout<<"Command: "<<cmd<<std::endl;
            }

        }
    }

    else
    {
        /* Print arrived kilobot */
        // std::cout<<"Kid:"<<unKilobotID<<" arrived"<<std::endl; 
        // std::cout<<"arrived kilobots: ";
        // for(int i=0; i<arrived.size(); i++)
        // {
        //     std::cout<<arrived[i]<<", ";
        // }
        // std::cout<<std::endl;

        if(unKilobotID == 1)
        {
            std::cout<<"SquareDistance: "<<SquareDistance(init_d_position , kiloPos)<<std::endl;
            std::cout<<"Kilobot position: "<<kiloPos<<std::endl;
            std::cout<<"Desired position: "<<init_d_position<<std::endl;
            std::cout<<"Kilobot orientation: "<<kiloPos<<std::endl;
            std::cout<<"Desired orientation: "<<kiloOrientation<<std::endl;
            std::cout<<"\n\n";
        }

        if(SquareDistance(init_d_position , kiloPos) > distThreshold)
        {
            arrived[unKilobotID] = false;
            cmd = STOP;
        }
    }
    

    msg.data[0] = uint8_t (cmd);
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&msg);
}

/****************************************/
/****************************************/
CColor CNavigationALF::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;

    for(auto m_target: m_TargetAreas)
    {
        if (SquareDistance(vec_position_on_plane, m_target.Center) < pow(m_target.Radius, 2))
        {
            cColor = m_target.Color;
        }
    }

    if (SquareDistance(vec_position_on_plane, m_sClusteringHub.Center) < pow(m_sClusteringHub.Radius, 2))
    {
        cColor = m_sClusteringHub.Color;
    }
    
    return cColor;
}



REGISTER_LOOP_FUNCTIONS(CNavigationALF, "ALF_navigation_loop_function")
