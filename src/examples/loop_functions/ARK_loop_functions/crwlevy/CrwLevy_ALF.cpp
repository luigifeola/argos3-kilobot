#include "CrwLevy_ALF.h"

/****************************************/
/****************************************/

CCrwlevyALF::CCrwlevyALF() :
m_unDataAcquisitionFrequency(10),
num_robots_with_discovery(0),
num_robots_with_info(0){}

/****************************************/
/****************************************/

CCrwlevyALF::~CCrwlevyALF(){
}

/****************************************/
/****************************************/

void CCrwlevyALF::Init(TConfigurationNode& t_node) {
    /* Initialize ALF*/
    CALF::Init(t_node);
    /* Other initializations: Varibales, Log file opening... */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void CCrwlevyALF::Reset() {
    /* Close data file */
    m_cOutput.close();
    /* Reopen the file, erasing its contents */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void CCrwlevyALF::Destroy() {
    /* Close data file */
    m_cOutput.close();
}

/****************************************/
/****************************************/

void CCrwlevyALF::SetupInitialKilobotStates() {
    m_vecKilobotStates.resize(m_tKilobotEntities.size());
    m_vecKilobotsPositions.resize(m_tKilobotEntities.size());
    m_vecDesInitKilobotPose.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);
    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        /* Setup the virtual states of a kilobot(e.g. has food state)*/
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }


    //Fix here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    

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

void CCrwlevyALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    /* The kilobots begins outside the clustering hub*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    m_vecKilobotStates[unKilobotID] = NOT_TARGET_FOUND;
    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecLastTimeMessaged[unKilobotID] = -1000;

    /* Get a random rotation within the circular arena */
    CQuaternion rand_rot;
    CRadians rand_rot_angle(((Real) rand()/(RAND_MAX))*CRadians::PI_OVER_TWO.GetValue());
    rand_rot.FromEulerAngles(rand_rot_angle, CRadians::ZERO, CRadians::ZERO);

    /* Get a non-colliding random position within the circular arena */
    bool distant_enough = false;
    Real rand_angle, rand_displacement_x, rand_displacement_y;
    CVector3 rand_pos;

    UInt16 maxTries = 999;
    UInt16 tries = 0;

    do {
        rand_angle = ((Real) rand()/(RAND_MAX))*CRadians::TWO_PI.GetValue();
        rand_displacement_x = ((Real) rand()/(RAND_MAX))*(m_WallStructure.circular_arena_radius-0.025);
        rand_displacement_y = ((Real) rand()/(RAND_MAX))*(m_WallStructure.circular_arena_radius-0.025);
        rand_pos = CVector3(rand_displacement_x*sin(rand_angle),rand_displacement_y*cos(rand_angle),0);

        distant_enough = MoveEntity(c_kilobot_entity.GetEmbodiedEntity(), rand_pos, rand_rot, false);
        if(tries == maxTries-1) {
        std::cerr << "ERROR: too many tries and not an available spot for the area" << std::endl;
        }
    } while(!distant_enough);
    
    
    rand_angle = ((Real) rand()/(RAND_MAX))*CRadians::TWO_PI.GetValue();
    rand_displacement_x = ((Real) rand()/(RAND_MAX))*(m_WallStructure.circular_arena_radius-0.025);
    rand_displacement_y = ((Real) rand()/(RAND_MAX))*(m_WallStructure.circular_arena_radius-0.025);
    rand_pos = CVector3(rand_displacement_x*sin(rand_angle),rand_displacement_y*cos(rand_angle),0);
    m_vecDesInitKilobotPose[unKilobotID].SetX(rand_displacement_x);
    m_vecDesInitKilobotPose[unKilobotID].SetY(rand_displacement_y);
    m_vecDesInitKilobotPose[unKilobotID].SetZ(rand_angle);
     
}

/****************************************/
/****************************************/

void CCrwlevyALF::SetupVirtualEnvironments(TConfigurationNode& t_tree){
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

void CCrwlevyALF::GetExperimentVariables(TConfigurationNode& t_tree){
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
void CCrwlevyALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
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

void CCrwlevyALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
    
    /* Get the kilobot ID and state (Only Position in this example) */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    
    /* check if enough time has passed from the last message otherwise*/
    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID]< m_fMinTimeBetweenTwoMsg){
        return; // if the time is too short, the kilobot cannot receive a message
    }
    /* ARK sends message to kilobot only when it find the target */
    else if (m_vecKilobotStates[unKilobotID] == TARGET_FOUND){
        /* Prepare the inividual kilobot's message */
        m_tMessages[unKilobotID].type = 0; // using type 0 to signal ark messages
        /* Save time for next messages */
        m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
        /* Fill up the kb message */
        m_tMessages[unKilobotID].data[0] = unKilobotID;
        m_tMessages[unKilobotID].data[1] = (UInt8) m_vecKilobotStates[unKilobotID]; // not_target_found or target_found or target_communicated
        
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
    }
    UpdateResults();
}

/****************************************/
/****************************************/
void CCrwlevyALF::UpdateResults(){
    Reset();
    std::map<UInt16,std::pair<UInt32,UInt32>>::const_iterator itr = m_KilobotResults.begin();
    m_cOutput << "#Kid" << "\t#fpt" << "\t#fit"<<std::endl;
    for (; itr!=m_KilobotResults.end(); ++itr){
        m_cOutput<< itr->first << '\t' << itr->second.first << '\t' << itr->second.second <<std::endl;
    }

    // m_cOutput<<"FRACTION with Discovery:"<<(Real)num_robots_with_discovery / (Real)m_tKilobotEntities.size()<< std::endl;
    // m_cOutput<<"FRACTION with Information:"<<(Real)num_robots_with_info / (Real)m_tKilobotEntities.size()<< std::endl;
    // m_cOutput<<"FRACTION with Information:"<<(Real)m_KilobotResults.size() / (Real)m_tKilobotEntities.size()<< std::endl;
}


/****************************************/
/****************************************/

CColor CCrwlevyALF::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;
    Real fDistance = Distance(vec_position_on_plane,m_sClusteringHub.Center);
    if(fDistance<m_sClusteringHub.Radius){
        cColor=m_sClusteringHub.Color;
    }
    return cColor;
}

REGISTER_LOOP_FUNCTIONS(CCrwlevyALF, "ALF_crwlevy_loop_function")
