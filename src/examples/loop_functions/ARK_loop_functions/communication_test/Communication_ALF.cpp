/**
 * This is the source file of ALF, the ARK (Augmented Reality for Kilobots) loop function. Here, we present a simple experiment
 * in which the robots search for a special area. When the robot finds the area, ALF signals him, and he stops moving.
 *
 * Reina, A., Cope, A. J., Nikolaidis, E., Marshall, J. A. R., & Sabo, C. (2017). ARK: Augmented Reality for Kilobots.
 * IEEE Robotics and Automation Letters, 2(3), 1755–1761. https://doi.org/10.1109/LRA.2017.2700059
 *
 * @author Mohamed Salaheddine Talamali <mstalamali1@sheffield.ac.uk>
 */

#include "Communication_ALF.h"

/****************************************/
/****************************************/

CCommunicationALF::CCommunicationALF() :
    m_unDataAcquisitionFrequency(10){
}

/****************************************/
/****************************************/

CCommunicationALF::~CCommunicationALF(){
}

/****************************************/
/****************************************/

void CCommunicationALF::Init(TConfigurationNode& t_node) {
    /* Initialize ALF*/
    CALF::Init(t_node);
    /* Other initializations: Varibales, Log file opening... */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void CCommunicationALF::Reset() {
    /* Close data file */
    m_cOutput.close();
    /* Reopen the file, erasing its contents */
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

/****************************************/
/****************************************/

void CCommunicationALF::Destroy() {
    /* Close data file */
    m_cOutput.close();
}

/****************************************/
/****************************************/

void CCommunicationALF::SetupInitialKilobotStates() {
    
}

/****************************************/
/****************************************/

void CCommunicationALF::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity){
    
}

/****************************************/
/****************************************/

void CCommunicationALF::SetupVirtualEnvironments(TConfigurationNode& t_tree){
    /* Get the virtual environments node from .argos file*/
    TConfigurationNode& tVirtualEnvironmentsNode = GetNode(t_tree,"environments");
    /* Get the node defining the clustering hub parametres*/
    TConfigurationNode& t_VirtualClusteringHubNode = GetNode(tVirtualEnvironmentsNode,"Area");
    GetNodeAttribute(t_VirtualClusteringHubNode, "position", m_sClusteringHub.Center);
    GetNodeAttribute(t_VirtualClusteringHubNode, "radius", m_sClusteringHub.Radius);
    GetNodeAttribute(t_VirtualClusteringHubNode, "color", m_sClusteringHub.Color);
}

/****************************************/
/****************************************/

void CCommunicationALF::GetExperimentVariables(TConfigurationNode& t_tree){
    /* Get the experiment variables node from the .argos file */
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");
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

void CCommunicationALF::UpdateKilobotState(CKilobotEntity &c_kilobot_entity){
    
}

/****************************************/
/****************************************/

void CCommunicationALF::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
    
}

/****************************************/
/****************************************/

CColor CCommunicationALF::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;
    Real fDistance = Distance(vec_position_on_plane,m_sClusteringHub.Center);
    if(fDistance<m_sClusteringHub.Radius){
        cColor=m_sClusteringHub.Color;
    }
    return cColor;
}

REGISTER_LOOP_FUNCTIONS(CCommunicationALF, "ALF_communication_loop_function")
