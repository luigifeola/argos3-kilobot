#include "crwlevy_with_positioning.h"

/****************************************/
/****************************************/
namespace{
const double kKiloDiameter = 0.033;
const double kDistanceTargetFromTheOrigin = 0.5;
const double kEpsilon = 0.0001;
const bool kobstacle_avoidance = true; 
}

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
    
    if (m_ArenaStructure.Wall_numbers != 0)
    {
        // Target positioned in the arena, avoiding positions where there are kilobots
        do{
            c_random_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
            c_position.SetX(c_rng->Uniform(CRange<Real>(0, m_ArenaStructure.Radius - c_radius)) * sin(c_random_angle));
            c_position.SetY(c_rng->Uniform(CRange<Real>(0, m_ArenaStructure.Radius - c_radius)) * cos(c_random_angle));
            
            //check if there is some kilobot on top of the area
            kilobot_on_the_top = 
            std::find_if(m_vecKilobotsPositions.begin(), m_vecKilobotsPositions.end(), [&c_position, &c_radius](CVector2 const &position) {
                return Distance(c_position, position) < (c_radius + kEpsilon) ;    //WARNING: 1mm costant
                });
            
        }while(kilobot_on_the_top != m_vecKilobotsPositions.end());

        // CLOSE SPACE EXPERIMENT, NO BIAS NEEDED
        bias_prob = 0.0;
    }

    else
    {
        //Target positioned at 50 cm from the origin
        c_random_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
        c_position.SetX(kDistanceTargetFromTheOrigin * sin(c_random_angle));
        c_position.SetY(kDistanceTargetFromTheOrigin * cos(c_random_angle));
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
    // m_vecLastTimeMessaged[unKilobotID] = -1000;

    /* Get a non-colliding random position within the circular arena */
    bool distant_enough = false;
    Real rand_angle, random_dist;
    Real rand_x, rand_y;
    CVector2 rand_displacement, rand_init_pos;
    CVector3 rand_pos;


    UInt16 maxTries = 999;
    UInt16 tries = 0;

    /******************************************!!!TESTING!!!***********************************************************************/
    // CQuaternion random_rotation = CQuaternion();
    // rand_angle = CRadians::ZERO.GetValue();/*CRadians::PI_OVER_TWO.GetValue();*/
    // random_dist = 0.46769;//676999999999;//m_ArenaStructure.Radius-0.1;
    // rand_pos = CVector3(0.45,0,0);
    
    // CRadians rand_rot_angle = CRadians::PI;
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
    /******************************************END-TESTING***********************************************************************/
    /* Get a random position and orientation for the kilobot initialized into a square but positioned in the circular arena */
    CQuaternion random_rotation;
    CRadians rand_rot_angle(c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue())));
    random_rotation.FromEulerAngles(rand_rot_angle, CRadians::ZERO, CRadians::ZERO);
    Real radius =  m_ArenaStructure.Radius - m_ArenaStructure.Wall_width/2 - kKiloDiameter/2 - kEpsilon;

    do {
        rand_x = c_rng->Uniform(CRange<Real>(-radius,radius));
        rand_y = c_rng->Uniform(CRange<Real>(-radius,radius));
        distant_enough = MoveEntity(c_kilobot_entity.GetEmbodiedEntity(), CVector3(rand_x, rand_y, 0), random_rotation, false);
        
        if(tries == maxTries-1) {
            std::cerr << "ERROR: too many tries and not an available spot for the area" << std::endl;
        }
    } while(!distant_enough || (rand_x*rand_x)+(rand_y*rand_y) > radius*radius );
    
    
    /* Get a random position and orientation for the kilobot not on top of the target */
    // CQuaternion random_rotation;
    // CRadians rand_rot_angle(c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue())));
    // random_rotation.FromEulerAngles(rand_rot_angle, CRadians::ZERO, CRadians::ZERO);

    // do {
    //     rand_angle = c_rng->Uniform(CRange<Real>(-CRadians::PI.GetValue(), CRadians::PI.GetValue()));
	//     random_dist = c_rng->Uniform(CRange<Real>(0, m_ArenaStructure.Radius - m_ArenaStructure.Wall_width/2 - kKiloDiameter/2 - kEpsilon));
	//     rand_pos = CVector3(random_dist*sin(rand_angle),random_dist*cos(rand_angle),0);
        
    //     distant_enough = MoveEntity(c_kilobot_entity.GetEmbodiedEntity(), rand_pos, random_rotation, false);
        
    //     if(tries == maxTries-1) {
    //         std::cerr << "ERROR: too many tries and not an available spot for the area" << std::endl;
    //     }
    // } while(!distant_enough);

    // NOTE : questo cout viene stampato sul terminale e non sull'interfaccia di argos
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
    GetNodeAttribute(t_VirtualWallsNode, "radius", m_ArenaStructure.Radius);
    GetNodeAttribute(t_VirtualWallsNode, "width", m_ArenaStructure.Wall_width);
    GetNodeAttribute(t_VirtualWallsNode, "height", m_ArenaStructure.Wall_height);
    GetNodeAttribute(t_VirtualWallsNode, "walls", m_ArenaStructure.Wall_numbers);

    

    std::ostringstream entity_id;
    CRadians wall_angle = CRadians::TWO_PI / m_ArenaStructure.Wall_numbers;
    CVector3 wall_size(m_ArenaStructure.Wall_width, 2.0 * m_ArenaStructure.Radius * Tan(CRadians::PI / m_ArenaStructure.Wall_numbers), m_ArenaStructure.Wall_height);

    // wall positioning
    for (UInt32 i = 0; i < m_ArenaStructure.Wall_numbers; i++) {
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


    /* Get the node defining the clustering hub parametres*/
    TConfigurationNode& t_VirtualClusteringHubNode = GetNode(tVirtualEnvironmentsNode,"Area");
    GetNodeAttribute(t_VirtualClusteringHubNode, "position", m_sClusteringHub.Center);
    GetNodeAttribute(t_VirtualClusteringHubNode, "radius", m_sClusteringHub.Radius);
    GetNodeAttribute(t_VirtualClusteringHubNode, "color", m_sClusteringHub.Color);

    /* Show origin position */
    // SVirtualArea temp_area2;
    // temp_area2.Center = CVector2(0,0);
    // temp_area2.Radius = 0.0165;
    // temp_area2.Color = CColor::MAGENTA;
    // m_TargetAreas.push_back(temp_area2);
    /* Show some position */
    // SVirtualArea temp_area3;
    // temp_area3.Center = CVector2(0,0.45);
    // temp_area3.Radius = 0.003;
    // temp_area3.Color = CColor::BLACK;
    // m_TargetAreas.push_back(temp_area3);
}

/****************************************/
/****************************************/

void CCrwlevyALFPositioning::GetExperimentVariables(TConfigurationNode& t_tree){
    /* Get the experiment variables node from the .argos file */
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");
    /* Get the crwlevy exponents */
    GetNodeAttribute(tExperimentVariablesNode, "experiment_type", experiment_type);
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
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    
    // PrintKilobotState(c_kilobot_entity);
    // std::cout<<"Actual position: "<<GetKilobotPosition(c_kilobot_entity)<<std::endl;

    //std::cout<<"Distance from the center " << Distance(m_vecKilobotsPositions[unKilobotID],  CVector2(0,0)) << std::endl;
    // TODO: attento prima di essere TARGET_FOUND deve essere READY
    if(start_experiment)
    {
        m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
        m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
        m_vecKilobotsBiasAngle[unKilobotID] = CRadians::ZERO;

        CRadians robot_bearing = GetBearingRobotPosition(c_kilobot_entity);
        bool facing_wall = NormalizedDifference(m_vecKilobotsOrientations[unKilobotID],robot_bearing).GetAbsoluteValue() <=CRadians::PI_OVER_TWO.GetValue();//((m_vecKilobotsOrientations[unKilobotID] >= CRadians::ZERO && robot_bearing >= CRadians::ZERO) || (m_vecKilobotsOrientations[unKilobotID] < CRadians::ZERO && robot_bearing < CRadians::ZERO));
        Real wall_threshold = m_ArenaStructure.Radius-m_ArenaStructure.Wall_width-kKiloDiameter/2;
        // std::cout<<"threshold"<<wall_threshold<<" arena radius:"<<m_ArenaStructure.Radius<<"Wall width:"<<m_ArenaStructure.Wall_width<<" kilosize: "<<kKiloDiameter<<std::endl;

        // std::cout<<"KbAngle "<< ToDegrees( GetKilobotOrientation(c_kilobot_entity));
        // std::cout<<"\tBearing: "<<ToDegrees(robot_bearing)<<std::endl;
        
        // if(facing_wall)
        // {
        //     std::cout<<"Facing Wall:\t";
        //     std::cout<<"KbAngle "<< ToDegrees(m_vecKilobotsOrientations[unKilobotID]);
        //     std::cout<<"\tBearing: "<<ToDegrees(robot_bearing);
        //     std::cout<<"\tDiff: "<<ToDegrees(NormalizedDifference(m_vecKilobotsOrientations[unKilobotID],robot_bearing))<<std::endl;
        // }
        // else
        // {
        //     std::cout<<"Facing HOME!!!!\t";
        //     std::cout<<"KbAngle "<< ToDegrees(m_vecKilobotsOrientations[unKilobotID]);
        //     std::cout<<"\tBearing: "<<ToDegrees(robot_bearing)<<std::endl;
        // }

        // std::cout<<"Distance threshold:"<<Distance(GetKilobotPosition(c_kilobot_entity),CVector2(0,0))<<" SqDistance threshold:"<<SquareDistance(GetKilobotPosition(c_kilobot_entity),CVector2(0,0))<<std::endl;
        // std::cout<<"threshold:"<<wall_threshold<<" sqThreshold:"<<pow(wall_threshold,2)<<std::endl;

        switch (m_vecKilobotStates[unKilobotID]){
        case TARGET_FOUND:
            {
                if(experiment_type == OBSTACLE_AVOIDANCE_EXPERIMENT)
                {
                    if(GetKilobotLedColor(c_kilobot_entity) == CColor::BLUE && bias_prob!=0.0)
                    {
                        m_vecKilobotStates[unKilobotID] = BIASING;
                    }
                    else if(Distance(GetKilobotPosition(c_kilobot_entity),CVector2(0,0)) > wall_threshold && facing_wall && GetKilobotLedColor(c_kilobot_entity) != CColor::BLUE && m_ArenaStructure.Radius != 0.0)
                    {
                        m_vecKilobotStates[unKilobotID] = COLLIDING;
                    }
                }
                break;
            }
        case NOT_TARGET_FOUND:
        case TARGET_COMMUNICATED:
            {
                if(experiment_type == OBSTACLE_AVOIDANCE_EXPERIMENT)
                {
                    if(GetKilobotLedColor(c_kilobot_entity) == CColor::BLUE && bias_prob!=0.0)
                    {
                        m_vecKilobotStates[unKilobotID] = BIASING;
                    }
                    else if(Distance(GetKilobotPosition(c_kilobot_entity),CVector2(0,0)) > wall_threshold && facing_wall && GetKilobotLedColor(c_kilobot_entity) != CColor::BLUE && m_ArenaStructure.Radius != 0.0)
                    {
                        m_vecKilobotStates[unKilobotID] = COLLIDING;
                    }
                }

                else{
                    Real fDistance = Distance(GetKilobotPosition(c_kilobot_entity), m_sClusteringHub.Center);

                    //If the kilobot is on the target area
                    if(fDistance<(m_sClusteringHub.Radius)){
                        m_vecKilobotStatesLog[unKilobotID]=m_vecKilobotStates[unKilobotID];
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
                        m_vecKilobotStatesLog[unKilobotID]=m_vecKilobotStates[unKilobotID];
                        m_vecKilobotStates[unKilobotID]=TARGET_COMMUNICATED;
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
                }
                break;
            }
        case BIASING:
            {
                // Set_kilobot_bias_angle(c_kilobot_entity);
                // Set_kilobot_bias_angle(CKilobotEntity &c_kilobot_entity){
                // UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);    
                argos::CRadians kiloOrientation = GetKilobotOrientation(c_kilobot_entity);
                CVector2 kiloPosition = GetKilobotPosition(c_kilobot_entity);
                CRadians pathOrientation = ATan2(-kiloPosition.GetY(), 
                                                    -kiloPosition.GetX()) 
                                                    - kiloOrientation; //+ CRadians::PI_OVER_TWO;
                // std::cout<<"pathorientation:"<<ToDegrees(pathOrientation)<<std::endl;
                // // normalise the pathOrientation between -pi and pi
                pathOrientation.SignedNormalize(); //map angle in [-pi,pi]
                m_vecKilobotsBiasAngle[unKilobotID] = pathOrientation;
                // // std::cerr<<"(UInt8)pathOrientation : "<<(UInt8)pathOrientation.GetAbsoluteValue()<<std::endl;
                // // std::cerr<<"pathOrientation.GetAbsoluteValue() = "<<pathOrientation.GetAbsoluteValue()<<std::endl;
                // // std::cerr<<"pathOrientation.GetAbsoluteValue() < 0.52"<<(pathOrientation.GetAbsoluteValue() < 0.52)<<std::endl; 

                
                if(GetKilobotLedColor(c_kilobot_entity) != CColor::BLUE)
                {
                    // TODO : attento che lo stato e il log dello stato sia aggiornato bene
                    // PrintKilobotState((int)m_vecKilobotStates[unKilobotID]);
                    // PrintKilobotState((int)m_vecKilobotStatesLog[unKilobotID]);
                    // std::cerr<<std::endl;
                    m_vecKilobotStates[unKilobotID] = m_vecKilobotStatesLog[unKilobotID]; 
                }
                break;
            }
        case COLLIDING:
            {
                if(facing_wall)
                {
                    // experiment_type could be BIAS_EXPERIMENT or OBSTACLE_AVOIDANCE_EXPERIMENT
                    CRadians kiloOrientation = GetKilobotOrientation(c_kilobot_entity);
                    CVector2 kiloPosition = GetKilobotPosition(c_kilobot_entity);
                    
                    
                    /*Get collision point*/
                    
                    Real radius = m_ArenaStructure.Radius-m_ArenaStructure.Wall_width;
                    // Real a = Tan(kiloOrientation);
                    // Real b = -1.0;
                    // Real c = - (kiloPosition.GetX() * a) + kiloPosition.GetY();
                    
                    // std::cout<<"Kilopos: "<<kiloPosition<<std::endl;
                    // std::cout<<"KiloAngle: "<<ATan2(kiloPosition.GetY(),kiloPosition.GetX())<<std::endl;
                    // std::cout<<"a:"<<a<<'\t'<<"b:"<<b<<'\t'<<"c"<<c<<std::endl;
                    
                    // CVector2 collision_point = CircleLineIntersection(radius, a, b, c, kiloPosition);
                    // CRadians bouncing_angle = (CRadians::PI
                    //                             - ATan2(kiloPosition.GetY(),kiloPosition.GetX())
                    //                             - ATan2(collision_point.GetY(),collision_point.GetX())
                    //                             -ATan2(-kiloPosition.GetY(),-kiloPosition.GetX())
                    //                             - kiloOrientation).SignedNormalize();

                    
                    // std::cout<<"\nKilopos:"<<kiloPosition<<std::endl;
                    // std::cout<<"Kilo orientation: "<<ToDegrees(kiloOrientation)<<std::endl;
                    // std::cout<<"Origin Distance:"<<Distance(kiloPosition, CVector2(0,0))<<'\t'<<"R:"<<radius<<std::endl;
                    // std::cout<<"Atan2 kilobot:"<< ATan2(kiloPosition.GetY(),kiloPosition.GetX())<<std::endl;
                    // std::cout<<"Atan2+PI:"<< ATan2(kiloPosition.GetY(),kiloPosition.GetX()) + CRadians::PI <<std::endl;

                    /**************************Bouncing angle***************************/
                    // CRadians alpha = NormalizedDifference(ATan2(kiloPosition.GetY(),kiloPosition.GetX()) + CRadians::PI , kiloOrientation);
                    // // std::cout<<"NormDiff:"<<ToDegrees(alpha) <<std::endl;
                    // // std::cout<<"Sin(alpha):"<<Sin(alpha);
                    // // std::cout<<"Sin(gamma)"<<Distance(kiloPosition, CVector2(0,0)) / radius * Sin(alpha)<<std::endl;
                    // CRadians bouncing_angle = ASin( Distance(kiloPosition, CVector2(0,0)) / radius * Sin(alpha) );
                    // // std::cout<<"Asin(alpha): "<<ToDegrees(bouncing_angle)<<std::endl;

                    // CRadians bias = CRadians::PI - 2.0 * bouncing_angle;
                    // std::cout<<"bias before control:"<<ToDegrees(bias)<<std::endl;
                    // std::cout<<"bias:"<<ToDegrees(bias)<<std::endl;
                    /*********************************************************************************************************/

                    /******************* Bias angle + random in [-90,+90] *****************************/
                    CRadians bias = ATan2(-kiloPosition.GetY(), 
                                                        -kiloPosition.GetX()) 
                                                        - kiloOrientation; //+ CRadians::PI_OVER_TWO;
                    // std::cout<<"pathorientation:"<<ToDegrees(pathOrientation)<<std::endl;
                    /*Random angle in [-Pi,Pi]*/
                    CRadians rand_rot_angle(c_rng->Uniform(CRange<Real>(-CRadians::PI_OVER_TWO.GetValue(), CRadians::PI_OVER_TWO.GetValue())));
                    bias += rand_rot_angle;
                    /***************************************************************************************/
                    
                    bias.SignedNormalize(); //map angle in [-pi,pi]
                    // std::cout<<"biasSignedNormalized:"<<ToDegrees(bias)<<std::endl;
                    m_vecKilobotsBiasAngle[unKilobotID] = bias;
                }

                else
                {
                    // std::cout<<"KID:"<<unKilobotID<<" Sending ZERO!!!!!\n";
                    m_vecKilobotsBiasAngle[unKilobotID] = CRadians::ZERO;
                }
                           

                if(GetKilobotLedColor(c_kilobot_entity) == CColor::BLUE)
                {
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
    
void CCrwlevyALFPositioning::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
    /*Create ARK-type messages variables*/
    m_tALFKilobotMessage tKilobotMessage,tEmptyMessage,tMessage;
    /* Get the kilobot ID and state (Only Position in this example) */
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    
    // Prepare an empty ARK-type message to fill the gap in the full kilobot message
    tEmptyMessage.m_sID=511;
    tEmptyMessage.m_sType=0;
    tEmptyMessage.m_sData=0;


    tKilobotMessage.m_sID = unKilobotID;


    // PrintKilobotState(c_kilobot_entity);

    // se non é cominciato l'esperimento, manda i parametri
    if(!start_experiment)
    {
        // Messagges of parameters (crw, levy, bias_prob)
        UInt8 crw = (UInt8)(crw_exponent*10);
        UInt8 levy = (UInt8)(levy_exponent*10);
        
        m_tMessages[unKilobotID].type = 255;
        // std::cerr<<"bias_prob"<<bias_prob<<std::endl;
        //m_sType is just 4-bit ->[0,16]
        tKilobotMessage.m_sType = bias_prob*10;
        // std::cerr<<"message bias_prob"<<tKilobotMessage.m_sType<<std::endl;

        tKilobotMessage.m_sData = (crw << 5);
        tKilobotMessage.m_sData = tKilobotMessage.m_sData | levy;
    }


    // altrimenti se il kb e' in collisione 
    else if((m_vecKilobotStates[unKilobotID] == BIASING || m_vecKilobotStates[unKilobotID] == COLLIDING) && m_vecKilobotsBiasAngle[unKilobotID]!= CRadians::ZERO)
    {   
        
        if(m_vecKilobotStates[unKilobotID] == COLLIDING)
        {
            // Most significant bit set to 1 (colliding = true)
            tKilobotMessage.m_sID = tKilobotMessage.m_sID | 0x200;
        }
        
        // std::cerr<<"tKilobotMessage.m_sID : " << std::dec << tKilobotMessage.m_sID << '\n';

        m_tMessages[unKilobotID].type = 254;
        
        if(m_vecKilobotsBiasAngle[unKilobotID].GetValue() < 0.0)
        {
            tKilobotMessage.m_sType = RIGHT;    // TURN_RIGHT
            // if(unKilobotID == 3)
            //     std::cout<<"Turn RIGHT ";
        }
        else
        {
            tKilobotMessage.m_sType = LEFT;    //TURN_LEFT
            // if(unKilobotID == 3)
            //     std::cout<<"Turn LEFT ";
        }


        //used to send the actual angle of the kilobot
        UInt8 kilo_bias_angle_8_t = (UInt8) round(255 * m_vecKilobotsBiasAngle[unKilobotID].GetAbsoluteValue() / M_PI);
        tKilobotMessage.m_sData = kilo_bias_angle_8_t;
        // std::cerr<<"kilo_bias_angle_8_t : "<<m_vecKilobotsBiasAngle[unKilobotID]<<std::endl;
            
    }

    // Altrimenti se e' un cambio di stato 
    else if(GetKilobotLedColor(c_kilobot_entity) != CColor::RED && m_vecKilobotStates[unKilobotID]==TARGET_FOUND) 
    {
        m_tMessages[unKilobotID].type = 0;
        tKilobotMessage.m_sType = (int)m_vecKilobotStates[unKilobotID];
        tKilobotMessage.m_sData = 0;//(int)m_vecKilobotsOrientations[unKilobotID];
    }

    //Altrimenti ultimo caso, non hai nulla da mandare
    else
    {
        m_tMessages[unKilobotID].type = 111;
        tKilobotMessage = tEmptyMessage;
        // std::cout<<"Ho fatto la return\n";
        // return;
    }
    

    /* Send the message to the kilobot using the ARK messaging protocol (addressing 3 kilobots per one standard kilobot message)*/
    for (int i = 0; i < 9; ++i) {
        m_tMessages[unKilobotID].data[i]=0;
    }

    /** 
     * 
     * READY TO SEND A MESSAGE
     * 
    **/
    
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
    
    
    GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[unKilobotID]);
}
/****************************************/
/****************************************/

CVector2 CCrwlevyALFPositioning::CircleLineIntersection(Real radius, Real a, Real b, Real c, CVector2 kilo_pos)
{
    double x0 = -a*c/(a*a+b*b), y0 = -b*c/(a*a+b*b);
    if (c*c > radius*radius*(a*a+b*b)+EPSILON)
        // std::puts ("no points");
        std::cerr<<"ERROR!!!! No points\n";
    else if (abs (c*c - radius*radius*(a*a+b*b)) < EPSILON) {
        // std::puts ("1 point");
        std::cerr<<"ERROR!!!! Just one point\n";
        // cout << x0 << ' ' << y0 << '\n';
    }
    else 
    {
        double d = radius*radius - c*c/(a*a+b*b);
        double mult = sqrt (d / (a*a+b*b));
        double x1,x2,y1,y2;
        x1 = x0 + b * mult;
        x2 = x0 - b * mult;
        y1 = y0 - a * mult;
        y2 = y0 + a * mult;
        // std::puts ("2 points");
        // cout << ax << ' ' << ay << '\n' << bx << ' ' << by << '\n';
        CVector2 p1 = CVector2(x1,y1);
        CVector2 p2 = CVector2(x2,y2);
        // std::cerr<<"2 points: "<<p1<<'\t'<<p2<<std::endl;
        if(SquareDistance(p1,kilo_pos) < SquareDistance(p2,kilo_pos)){
            std::cerr<<"Collision point: "<<p1<<std::endl;
            return p1;
        }
        else
        {
            std::cerr<<"Collision point: "<<p2<<std::endl;
            return p2;
        }
        
    }
}

/****************************************/
/****************************************/

CRadians CCrwlevyALFPositioning::GetBearingRobotPosition(CKilobotEntity& c_kilobot_entity)
{
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);    
    CRadians kiloOrientation = GetKilobotOrientation(c_kilobot_entity);
    CRadians robot_bearing = ATan2(GetKilobotPosition(c_kilobot_entity).GetY(), 
                                    GetKilobotPosition(c_kilobot_entity).GetX());

    
    // normalise the pathOrientation between -pi and pi
    robot_bearing.SignedNormalize(); //map angle in [-pi,pi]
    return robot_bearing;
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


    
}

/****************************************/
/****************************************/

void CCrwlevyALFPositioning::PrintKilobotState(CKilobotEntity& c_kilobot_entity)
{
    UInt16 unKilobotID=GetKilobotId(c_kilobot_entity);
    // if(m_vecKilobotStates[unKilobotID] == m_vecKilobotStatesLog[unKilobotID])
    //     return;
    std::cerr<<"LOG state:";
    switch (m_vecKilobotStatesLog[unKilobotID])
    {
        case NOT_TARGET_FOUND:
            std::cerr<<"NOT_TARGET_FOUND"<<"\t";
            break;
        case TARGET_FOUND:
            std::cerr<<"TARGET_FOUND"<<"\t";
            break;
        case TARGET_COMMUNICATED:
            std::cerr<<"TARGET_COMMUNICATED"<<"\t";
            break;
        case BIASING:
            std::cerr<<"BIASING"<<"\t";
            break;
        case COLLIDING:
            std::cerr<<"COLLIDING"<<"\t";
            break;
        
        default:
            break;
    }

    std::cerr<<"Actual state:";
    switch (m_vecKilobotStates[unKilobotID])
    {
        case NOT_TARGET_FOUND:
            std::cerr<<"NOT_TARGET_FOUND"<<"\n";
            break;
        case TARGET_FOUND:
            std::cerr<<"TARGET_FOUND"<<"\n";
            break;
        case TARGET_COMMUNICATED:
            std::cerr<<"TARGET_COMMUNICATED"<<"\n";
            break;
        case BIASING:
            std::cerr<<"BIASING"<<"\n";
            break;
        case COLLIDING:
            std::cerr<<"COLLIDING"<<"\n";
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
    
    // // y-axis
    // if(vec_position_on_plane.GetX() < 0.01 && vec_position_on_plane.GetX()> -0.01 ){
    //     if(vec_position_on_plane.GetY() >= 0)
    //         cColor = CColor::BLUE;
    //     else 
    //         cColor = CColor::GRAY70;
    // }

    // // x-axis
    // if(vec_position_on_plane.GetY() < 0.01 && vec_position_on_plane.GetY()> -0.01){
    //     if(vec_position_on_plane.GetX() >= 0)
    //         cColor = CColor::ORANGE;
    //     else
    //         cColor = CColor::GRAY70;
    // }

    // if(Abs(vec_position_on_plane.GetY() - Tan(CRadians(-2.15391)) * vec_position_on_plane.GetX()) <= 0.01 )
    //     cColor = CColor::RED;

    // /*Draw some line*/
    // double x = vec_position_on_plane.GetX();
    // double y = vec_position_on_plane.GetY();
    // double ak,bk,ck;
    // ak= 11.0;
    // bk=-1;
    // ck = 0.0;
    // ck = 0.3;
    // if(Abs(ak/ck*x + bk/ck*y + 1.0) <= 0.09)
    //     cColor = CColor::BLACK;


    return cColor;
}

REGISTER_LOOP_FUNCTIONS(CCrwlevyALFPositioning, "ALF_crwlevy_positioning_loop_function")
