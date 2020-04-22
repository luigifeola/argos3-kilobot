#include "crwlevy_with_positioning.h"

/****************************************/
/****************************************/

double KiloDiameter = 0.033;

CCrwlevyALFPositioning::CCrwlevyALFPositioning() :
m_unDataAcquisitionFrequency(10),   //each 10 seconds store kilobots positione
num_robots_with_discovery(0),
num_robots_with_info(0),
internal_counter(0),
start_experiment(false),
start_experiment_time(0),
m_random_seed(0)
    {
        c_rng = CRandom::CreateRNG("argos");
    }

/****************************************/
/****************************************/

CCrwlevyALFPositioning::~CCrwlevyALFPositioning(){
}

/****************************************/
/****************************************/

void CCrwlevyALFPositioning::Init(TConfigurationNode& t_node) {
    /* Initialize ALF*/
    CALF::Init(t_node);
    /* Other initializations: Varibales, Log file opening... */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_cOutputPositions.open(m_strPositionsFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void CCrwlevyALFPositioning::Reset() {
    /* Close data file */
    m_cOutput.close();
    m_cOutputPositions.close();
    /* Reopen the file, erasing its contents */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_cOutputPositions.open(m_strPositionsFileName, std::ios_base::trunc | std::ios_base::out);

    //TODO : update reset function
    // std::fill(m_vecKilobotStates.begin(), m_vecKilobotStates.end(), NOT_TARGET_FOUND);
    // std::fill(m_vecLastTimeMessaged.begin(), m_vecLastTimeMessaged.end(), -1000);

    // for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
    //     /* Setup the virtual states of a kilobot(e.g. has food state)*/
    //     SetupInitialKilobotState(*m_tKilobotEntities[it]);
    // }
    
    // m_vecKilobotsPositionsHistory.push_back(m_vecKilobotsPositions);

}

/****************************************/
/****************************************/

void CCrwlevyALFPositioning::Destroy() {
    /* Close data file */
    m_cOutput.close();
    m_cOutputPositions.close();
}

/****************************************/
/****************************************/

void CCrwlevyALFPositioning::SetupInitialKilobotStates() {
    m_vecKilobotStates.resize(m_tKilobotEntities.size());
    m_vecKilobotStatesLog.resize(m_tKilobotEntities.size());
    
    
    m_vecKilobotsPositions.resize(m_tKilobotEntities.size());
    m_vecKilobotsPositionsHistory.reserve(m_tKilobotEntities.size());
    
    m_vecKilobotsOrientations.resize(m_tKilobotEntities.size());
    m_vecKilobotsBiasAngle.resize(m_tKilobotEntities.size());
    
    v_recivedCoefficients.resize(m_tKilobotEntities.size());
    std::fill(v_recivedCoefficients.begin(), v_recivedCoefficients.end(), false);
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);

    for(UInt16 it=0;it< m_tKilobotEntities.size();it++){
        /* Setup the virtual states of a kilobot(e.g. has food state)*/
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }
    
    //Dopo che hai piazzato i Kilobot, ti salvi la posizione iniziale
    // PrintVecPos(m_vecKilobotsPositions);
    m_vecKilobotsPositionsHistory.push_back(m_vecKilobotsPositions);

    //The experiment must start with no kilobot on top of the target
    //In the following there is the collision check between target and each kilobot
    Real c_random_angle;
    CVector2 & c_position = m_sClusteringHub.Center;
    Real & c_radius = m_sClusteringHub.Radius;
    std::vector<CVector2>::iterator kilobot_on_the_top;
    
    if (m_WallStructure.circular_arena_walls != 0)
    {
        // Target positioned in the arena, avoiding positions where there are kilobots
        do{
            c_random_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
            c_position.SetX(c_rng->Uniform(CRange<Real>(0, m_WallStructure.circular_arena_radius - c_radius)) * sin(c_random_angle));
            c_position.SetY(c_rng->Uniform(CRange<Real>(0, m_WallStructure.circular_arena_radius - c_radius)) * cos(c_random_angle));
            
            //check if there is some kilobot on top of the area
            kilobot_on_the_top = 
            std::find_if(m_vecKilobotsPositions.begin(), m_vecKilobotsPositions.end(), [&c_position, &c_radius](CVector2 const &position) {
                return Distance(c_position, position) < (c_radius + 0.001) ;    //WARNING: 1mm costant
                });
            
        }while(kilobot_on_the_top != m_vecKilobotsPositions.end());

        // CLOSE SPACE EXPERIMENT, NO BIAS NEEDED
        bias_prob = 0.0;
    }

    else
    {
        //Target positioned at 50 cm from the origin
        c_random_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
        c_position.SetX(0.5 * sin(c_random_angle));
        c_position.SetY(0.5 * cos(c_random_angle));
        // std::cout<<"Distance from the origin : "<< pow(c_position.GetX(),2) + pow(c_position.GetY(),2) << std::endl;
    }
    
     
    

    
}

/****************************************/
/****************************************/

void CCrwlevyALFPositioning::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    /* The kilobots begins outside the clustering hub*/
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    m_vecKilobotStates[unKilobotID] = NOT_TARGET_FOUND;
    m_vecKilobotStatesLog[unKilobotID] = NOT_TARGET_FOUND;
    m_vecLastTimeMessaged[unKilobotID] = -1000;

    /* Get a non-colliding random position within the circular arena */
    bool distant_enough = false;
    Real rand_angle, random_dist;
    CVector2 rand_displacement, rand_init_pos;
    CVector3 rand_pos;

    UInt16 maxTries = 999;
    UInt16 tries = 0;

    /*Check for possible border robot position*/
    // CQuaternion random_rotation;
    // rand_angle = CRadians::ZERO.GetValue();/*CRadians::PI_OVER_TWO.GetValue();*/
    // random_dist = 0.46769;//676999999999;//m_WallStructure.circular_arena_radius-0.1;
    // rand_pos = CVector3(random_dist*sin(rand_angle),random_dist*cos(rand_angle),0);
    
    // CRadians rand_rot_angle = -CRadians::PI_OVER_TWO;
    // random_rotation.FromEulerAngles(rand_rot_angle, CRadians::ZERO, CRadians::ZERO);
    
    // distant_enough = MoveEntity(c_kilobot_entity.GetEmbodiedEntity(), rand_pos, random_rotation, false);

    // if(distant_enough)
    // {
    //     std::cout<<"Piazzato!!!!!\n";
    // }
    // else
    // {
    //     std::cout<<"Problema il robot non può essere messo lì\n";
    // }
    /*A qui*/
    
    /* Get a random orientation for the kilobot */
    CQuaternion random_rotation;
    CRadians rand_rot_angle(c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue())));
    random_rotation.FromEulerAngles(rand_rot_angle, CRadians::ZERO, CRadians::ZERO);

    do {
        rand_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
	    random_dist = c_rng->Uniform(CRange<Real>(0, m_WallStructure.circular_arena_radius - m_WallStructure.circular_arena_width/2 - KiloDiameter/2 - 0.0001));
	    rand_pos = CVector3(random_dist*sin(rand_angle),random_dist*cos(rand_angle),0);
        
        distant_enough = MoveEntity(c_kilobot_entity.GetEmbodiedEntity(), rand_pos, random_rotation, false);
        
        if(tries == maxTries-1) {
            std::cerr << "ERROR: too many tries and not an available spot for the area" << std::endl;
        }
    } while(!distant_enough);

    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);

}

/****************************************/
/****************************************/

void CCrwlevyALFPositioning::SetupVirtualEnvironments(TConfigurationNode& t_tree){
    /*Simulator variables*/
    CSimulator &simulator = GetSimulator();
    m_random_seed = simulator.GetRandomSeed();

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

    // wall positioning
    for (UInt32 i = 0; i < m_WallStructure.circular_arena_walls; i++) {
        entity_id.str("");
        entity_id << "wall_" << i;

        CRadians wall_rotation = wall_angle * i;
        // CVector3 wall_position((m_WallStructure.circular_arena_radius) * Cos(wall_rotation), (m_WallStructure.circular_arena_radius) * Sin(wall_rotation), 0);
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

    /* Show origin position */
    SVirtualArea temp_area2;
    temp_area2.Center = CVector2(0,0);
    temp_area2.Radius = 0.0165;
    temp_area2.Color = CColor::MAGENTA;
    m_TargetAreas.push_back(temp_area2);
}

/****************************************/
/****************************************/

void CCrwlevyALFPositioning::GetExperimentVariables(TConfigurationNode& t_tree){
    /* Get the experiment variables node from the .argos file */
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");
    /* Get the crwlevy exponents */
    GetNodeAttribute(tExperimentVariablesNode, "crw", crw_exponent);
    GetNodeAttribute(tExperimentVariablesNode, "levy", levy_exponent);
    GetNodeAttribute(tExperimentVariablesNode, "bias_prob", bias_prob);
    /* Get the positions datafile name to store Kilobot positions in time */
    GetNodeAttribute(tExperimentVariablesNode, "positionsfilename", m_strPositionsFileName);
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
void CCrwlevyALFPositioning::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
    /* Update the state of the kilobots (target not found, found directly or communicated)*/
    // std::cerr<<"UpdateKilobotState\n";
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    // TODO: attento prima di essere TARGET_FOUND deve essere READY
    if(start_experiment)
    {
        m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
        m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
        // std::cout<<"Angle "<< m_vecKilobotsOrientations[unKilobotID] << std::endl;

        switch (m_vecKilobotStates[unKilobotID])
        {
        case TARGET_FOUND:
            {
                if(GetKilobotLedColor(c_kilobot_entity) == CColor::WHITE)
                {
                    m_vecKilobotStates[unKilobotID] = BIASING;
                }
                break;
            }
        case NOT_TARGET_FOUND:
        case TARGET_COMMUNICATED:
            {
                if(GetKilobotLedColor(c_kilobot_entity) == CColor::WHITE)
                {
                    m_vecKilobotStates[unKilobotID] = BIASING;
                }

                Real fDistance = Distance(m_vecKilobotsPositions[unKilobotID], m_sClusteringHub.Center);

                //If the kilobot is on the target area
                if(fDistance<(m_sClusteringHub.Radius)){//*0.9
                    m_vecKilobotStates[unKilobotID]=TARGET_FOUND;
                    m_vecKilobotStatesLog[unKilobotID]=TARGET_FOUND;
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
                    m_vecKilobotStatesLog[unKilobotID]=TARGET_COMMUNICATED;
                    std::map<UInt16,std::pair<UInt32,UInt32>>::const_iterator itr = m_KilobotResults.find(unKilobotID);
                    // if c_kilobot_entity already has info about the target no update is needed
                    if (itr!=m_KilobotResults.end()){
                        return;
                    }
                    else{
                        UInt32 simclock = GetSpace().GetSimulationClock();
                        m_KilobotResults.insert(std::pair<UInt16,std::pair<UInt32,UInt32>> (unKilobotID,std::pair<UInt32,UInt32>(0,simclock)));
                        num_robots_with_info+=1;
                    }
                    
                }
                break;
            }
        case BIASING:
            {
                Set_kilobot_bias_angle(c_kilobot_entity);
                if(GetKilobotLedColor(c_kilobot_entity) != CColor::WHITE)
                {
                    // TODO : attento che lo stato e il log dello stato sia aggiornato bene
                    // PrintKilobotState((int)m_vecKilobotStates[unKilobotID]);
                    // PrintKilobotState((int)m_vecKilobotStatesLog[unKilobotID]);
                    // std::cerr<<std::endl;
                    m_vecKilobotStates[unKilobotID] = m_vecKilobotStatesLog[unKilobotID]; 
                }
                break;
            }
        
        default:
            break;
        }
    }
    
}

/****************************************/
/****************************************/

void CCrwlevyALFPositioning::Set_kilobot_bias_angle(CKilobotEntity &c_kilobot_entity){
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);    
    argos::CRadians kiloOrientation = GetKilobotOrientation(c_kilobot_entity);
    CRadians pathOrientation = ATan2(-m_vecKilobotsPositions[unKilobotID].GetY(), 
                                        -m_vecKilobotsPositions[unKilobotID].GetX()) 
                                        - kiloOrientation; //+ CRadians::PI_OVER_TWO;

    
    // normalise the pathOrientation between -pi and pi
    pathOrientation.SignedNormalize(); //map angle in [-pi,pi]
    m_vecKilobotsBiasAngle[unKilobotID] = pathOrientation;
    // std::cerr<<"(UInt8)pathOrientation : "<<(UInt8)pathOrientation.GetAbsoluteValue()<<std::endl;
    // std::cerr<<"pathOrientation.GetAbsoluteValue() = "<<pathOrientation.GetAbsoluteValue()<<std::endl;
    // std::cerr<<"pathOrientation.GetAbsoluteValue() < 0.52"<<(pathOrientation.GetAbsoluteValue() < 0.52)<<std::endl; 
}

/****************************************/
/****************************************/
    
void CCrwlevyALFPositioning::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
    /*Create ARK-type messages variables*/
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    /* Get the kilobot ID and state (Only Position in this example) */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    // // TODO : REMOVE FOLLOWING 2 ROWS
    // argos::CRadians kiloOrientation = GetKilobotOrientation(c_kilobot_entity);
    // std::cout<<"Kilobot orientation : "<< kiloOrientation<<std::endl;

    // /********************************************************************************************/
    // // TODO : REMOVE, just for testing
    // CRadians pathOrientation = ATan2(-m_vecKilobotsPositions[unKilobotID].GetY(), 
    //                                     -m_vecKilobotsPositions[unKilobotID].GetX()) 
    //                                     - kiloOrientation; //+ CRadians::PI_OVER_TWO;

    
    // // normalise the pathOrientation between -pi and pi
    // pathOrientation.SignedNormalize(); //map angle in [-pi,pi]
    // m_vecKilobotsBiasAngle[unKilobotID] = pathOrientation;
    // std::cerr<<"pathOrientation : "<<pathOrientation.GetAbsoluteValue()<<std::endl;
    // /*******************************************************************************************/

    /* check if enough time has passed from the last message otherwise*/
    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID]< m_fMinTimeBetweenTwoMsg){
        return; // if the time is too short, the kilobot cannot receive a message
    }

    tKilobotMessage.m_sID = unKilobotID;



    
    if(!start_experiment)
    {
        // Messagges of parameters (crw, levy, bias_prob)
        UInt8 crw = (UInt8)(crw_exponent*10);
        UInt8 levy = (UInt8)(levy_exponent*10);
        
        m_tMessages[unKilobotID].type = 255;
        // std::cerr<<bias_prob<<std::endl;
        //m_sType is just 4-bit ->[0,16]
        tKilobotMessage.m_sType = bias_prob*10;
        tKilobotMessage.m_sData = (crw << 5);
        tKilobotMessage.m_sData = tKilobotMessage.m_sData | levy;
    }
    // TODO : else if biasing bla bla
    else if(m_vecKilobotStates[unKilobotID] == BIASING)
    {
        
            m_tMessages[unKilobotID].type = 254;
            if(m_vecKilobotsBiasAngle[unKilobotID].GetValue() < 0.0)
            {
                tKilobotMessage.m_sType = RIGHT;    // TURN_RIGHT
            }
            else
            {
                tKilobotMessage.m_sType = LEFT;    //TURN_LEFT
            }
            
            //used to send the actual angle of the kilobot
            UInt8 kilo_bias_angle_8_t = (UInt8) round(255 * m_vecKilobotsBiasAngle[unKilobotID].GetAbsoluteValue() / M_PI);
            tKilobotMessage.m_sData = kilo_bias_angle_8_t;
            // std::cerr<<"kilo_bias_angle_8_t : "<<m_vecKilobotsBiasAngle[unKilobotID].GetAbsoluteValue()<<std::endl;
    }

    else  // Message with kilobot state
    {
        m_tMessages[unKilobotID].type = 0;
        tKilobotMessage.m_sType = (int)m_vecKilobotStates[unKilobotID];
        //used to send the actual angle of the kilobot
        tKilobotMessage.m_sData = 0;//(int)m_vecKilobotsOrientations[unKilobotID];
    }
        
    /*  Set the message sending flag to True */
    m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;
    

    /* Send the message to the kilobot using the ARK messaging protocol (addressing 3 kilobots per one standard kilobot message)*/
    for (int i = 0; i < 9; ++i) {
        m_tMessages[unKilobotID].data[i]=0;
    }
    // Prepare an empty ARK-type message to fill the gap in the full kilobot message
    tEmptyMessage.m_sID=1023;
    tEmptyMessage.m_sType=0;
    tEmptyMessage.m_sData=0;
    // Fill the kilobot message by the ARK-type messages
    for (int i = 0; i < 3; ++i) {
        if( i == 0){
            tMessage = tKilobotMessage;
        } else{
            tMessage = tEmptyMessage;
        }
        m_tMessages[unKilobotID].data[i*3] = (tMessage.m_sID >> 2);
        m_tMessages[unKilobotID].data[1+i*3] = (tMessage.m_sID << 6);
        m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sType << 2);
        m_tMessages[unKilobotID].data[1+i*3] = m_tMessages[unKilobotID].data[1+i*3] | (tMessage.m_sData >> 8);
        m_tMessages[unKilobotID].data[2+i*3] = tMessage.m_sData;
    }
    
    // experiments does not start until all kilobots have received coefficients
    if(!start_experiment)
    {
        v_recivedCoefficients[unKilobotID]=true;
        if(std::all_of(v_recivedCoefficients.begin(), v_recivedCoefficients.end(), [](bool received){return received;}))
        {
            start_experiment = true;
            start_experiment_time = m_fTimeInSeconds;
        }
    }

    else 
    {
        UpdateTimeResults();
    }
    
    
    /* Sending the message */
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
}

/****************************************/
/****************************************/
void CCrwlevyALFPositioning::UpdateTimeResults(){
    /* Close data file */
    m_cOutput.close();
    /* Reopen the file, erasing its contents */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);

    // std::map<UInt16,std::pair<UInt32,UInt32>>::const_iterator itr = m_KilobotResults.begin();
    m_cOutput << "Robot id\tFirst discovery time\tFirst information time" << std::endl;
    
    //for loop to store all the elements even if no info is available
    for(uint i=0; i<m_vecKilobotStates.size(); i++)
    {
        std::map<UInt16,std::pair<UInt32,UInt32>>::const_iterator itr = m_KilobotResults.find(i);
        if(itr != m_KilobotResults.end())
        {
           m_cOutput<< (itr->first)+1 << '\t' << itr->second.first << '\t' << itr->second.second <<std::endl;
        }
        else
        {
            m_cOutput<< i+1 << '\t' << 0.0 << '\t' << 0.0 <<std::endl;
        }
        
    }

    //for loop to store existing elements
    // for (; itr!=m_KilobotResults.end(); ++itr){
    //     m_cOutput<< itr->first << '\t' << itr->second.first << '\t' << itr->second.second <<std::endl;
    // }

    // m_cOutput<<"FRACTION with Discovery:"<<(Real)num_robots_with_discovery / (Real)m_tKilobotEntities.size()<< std::endl;
    // m_cOutput<<"FRACTION with Information:"<<(Real)num_robots_with_info / (Real)m_tKilobotEntities.size()<< std::endl;
    // m_cOutput<<"FRACTION with Information:"<<(Real)m_KilobotResults.size() / (Real)m_tKilobotEntities.size()<< std::endl;
}


/****************************************/
/****************************************/
void CCrwlevyALFPositioning::PostStep()
{

    Real actual_time_experiment;
    actual_time_experiment = m_fTimeInSeconds - start_experiment_time;
    // std::cout<<"Time in seconds:"<<actual_time_experiment<<std::endl;
    
    if(start_experiment)
    {
        // std::cout<<"Qui ci entro\n";
        internal_counter+=1;
        if(internal_counter == m_unDataAcquisitionFrequency)
        {
            // PrintVecPos(m_vecKilobotsPositions);
            m_vecKilobotsPositionsHistory.push_back(m_vecKilobotsPositions);
            internal_counter = 0;   
        }

    }

    // for(const auto& pos : m_vecKilobotsPositions)
    // {
    //     std::cout<<'\t'<<std::fixed<<std::setprecision(3)<<pos;
    // }

    // Real integer_digits = -1;
    // Real floating_digits = std::modf(actual_time_experiment, &integer_digits);
    
    //     // if(!((int)integer_digits%10) && (std::fabs(floating_digits - 0.1) < EPSILON ))
    //     //     std::cout<<"actual_time_experiment:"<<actual_time_experiment<<std::endl;
    
    // if(start_experiment && !((int)integer_digits%10) && (std::fabs(floating_digits - 0.1) < EPSILON ))
    // {
    //     m_cOutputPositions<</*std::fixed<<std::setprecision(1)<<*/actual_time_experiment;
    //     for(const auto& pos : m_vecKilobotsPositions)
    //     {
    //         m_cOutputPositions<<'\t'<<std::fixed<<std::setprecision(3)<<pos;
    //     }
    //     m_cOutputPositions<<std::endl;
    // }
}


/****************************************/
/****************************************/

const std::string currentDateTime()
{
      time_t now = time(0);
      struct tm tstruct;
      char buf[80];
      tstruct = *localtime(&now);
      strftime(buf, sizeof(buf), "%Y%m%d-%X-%M", &tstruct);

      return buf;
}

const std::string currentDate()
{
      time_t now = time(0);
      struct tm tstruct;
      char buf[80];
      tstruct = *localtime(&now);
      strftime(buf, sizeof(buf), "%Y%m%d", &tstruct);

      return buf;
}

void CCrwlevyALFPositioning::PostExperiment()
{
    m_cOutputPositions<< "Robot id";
    for (uint j = 0; j < m_vecKilobotsPositionsHistory.size(); j++)
    {
        m_cOutputPositions << "\tt = " << j*m_unDataAcquisitionFrequency;
    }
    m_cOutputPositions<< std::endl;

    for (uint i = 0; i < m_vecKilobotsPositionsHistory[0].size(); i++)
	{
        m_cOutputPositions << i;
		for (uint j = 0; j < m_vecKilobotsPositionsHistory.size(); j++)
		{
            m_cOutputPositions <<'\t'<< std::fixed<<std::setprecision(3)<< m_vecKilobotsPositionsHistory[j][i];
		}
		m_cOutputPositions<<std::endl;
	}



    // m_cOutputPositions << "Robot id";
    // for (uint j = 0; j < m_vecKilobotsPositionsHistory[1].size(); j++)
    // {
    //         int t = j;
    //         m_cOutputPositions << "\tt = " << t;
    // }
    // m_cOutputPositions<<std::endl;
    // for (uint i = 1; i <= m_vecKilobotsPositions.size(); i++)
    // {
    //         m_cOutputPositions << i;

    //         for (uint j = 0; j < m_vecKilobotsPositionsHistory[i].size(); j++)
    //         {
    //             m_cOutputPositions << '\t' << std::setprecision(3) << (m_vecKilobotsPositionsHistory[i])[j];
    //         }
    //         m_cOutputPositions << std::endl;
    // }
    // std::string dateTime = currentDateTime();
    // std::string date = currentDate();
    // char numRobotStr[10];
    // sprintf(numRobotStr, "%d", m_tKilobotEntities.size());
    // char alpha[10];
    // sprintf(alpha, "%.1f", m_alpha);
    // char rho[10];
    // sprintf(rho, "%.2f", m_rho);

    // std::string folder = "experiments/" + date + "_robots=" + numRobotStr + "_alpha=" + alpha + "_rho=" + rho + "_experiments";
    // if (opendir(folder.c_str()) == NULL)
    // {
    //     const int dir_err = mkdir(folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    //     if (-1 == dir_err)
    //     {
    //             printf("Error creating directory!\n");
    //             exit(1);
    //     }
    // }
    // char randomStr[5];
    // int randomInt = m_pcRNG->Uniform(CRange<int>((int)0, (int)99999));
    // sprintf(randomStr, "_%05d_%d_", randomInt, m_random_seed);
    // std::string prefix = folder + "/" + dateTime + randomStr;

    // std::string position_file = prefix + "position.tsv";
    // std::string time_results_file = prefix + "time_results.tsv";


    
}

/****************************************/
/****************************************/

void CCrwlevyALFPositioning::PrintKilobotState(int state)//, const char*  type_of_state)
{
    switch (state)
    {
        case NOT_TARGET_FOUND:
            std::cerr<<"Kilobot state  : NOT_TARGET_FOUND"<<"\n";
            break;
        case TARGET_FOUND:
            std::cerr<<"Kilobot state  : TARGET_FOUND"<<"\n";
            break;
        case TARGET_COMMUNICATED:
            std::cerr<<"Kilobot state  : TARGET_COMMUNICATED"<<"\n";
            break;
        case BIASING:
            std::cerr<<"Kilobot state  : BIASING"<<"\n";
            break;
        
        default:
            break;
    } 
}

/****************************************/
/****************************************/

void CCrwlevyALFPositioning::PrintKilobotCommand(int command)
{
    switch (command)
    {
        case LEFT:
            std::cout<<"Kilobot command  : LEFT"<<"\n";
            break;
        case RIGHT:
            std::cout<<"Kilobot command  : RIGHT"<<"\n";
            break;
        case STOP:
            std::cout<<"Kilobot command  : STOP"<<"\n";
            break;
        
        default:
            break;
    } 
}


/****************************************/
/****************************************/

void CCrwlevyALFPositioning::PrintVecPos(std::vector<CVector2> vecKilobotsPositions)
{
    std::cout<<"PrintVecPos:\n";
    for(const auto& pos : m_vecKilobotsPositions)
    {
        std::cout<<'\t'<<std::fixed<<std::setprecision(3)<<pos;
    }     
    std::cout<<"\n"; 
}


/****************************************/
/****************************************/

CColor CCrwlevyALFPositioning::GetFloorColor(const CVector2 &vec_position_on_plane) {
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

REGISTER_LOOP_FUNCTIONS(CCrwlevyALFPositioning, "ALF_crwlevy_positioning_loop_function")
