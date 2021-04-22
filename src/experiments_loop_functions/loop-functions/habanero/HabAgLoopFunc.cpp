/**
  * @file <loop-functions/IcraLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "HabAgLoopFunc.h"

/****************************************/
/****************************************/

HabAgLoopFunction::HabAgLoopFunction() {
    m_unClock = 0;
    m_unStopTime = 0;
    m_unStopBlock = 0;
    m_fObjectiveFunction = 0;
    m_fPheromoneParameter = 0;
}

/****************************************/
/****************************************/

HabAgLoopFunction::HabAgLoopFunction(const HabAgLoopFunction& orig) {
}

/****************************************/
/****************************************/

HabAgLoopFunction::~HabAgLoopFunction() {}

/****************************************/
/****************************************/

void HabAgLoopFunction::Destroy() {

    m_tRobotStates.clear();
    m_tLEDStates.clear();
    ros::shutdown();
}

/****************************************/
/****************************************/

void HabAgLoopFunction::Init(TConfigurationNode& t_tree) {

    CoreLoopFunctions::Init(t_tree);
    TConfigurationNode cParametersNode;
    try {
      cParametersNode = GetNode(t_tree, "params");
      GetNodeAttributeOrDefault(cParametersNode, "maximization", m_bMaximization, (bool) false);
      GetNodeAttributeOrDefault(cParametersNode, "robot_0", m_unRobot[0], (UInt32) 2);
      GetNodeAttributeOrDefault(cParametersNode, "robot_1", m_unRobot[1], (UInt32) 4);
      GetNodeAttributeOrDefault(cParametersNode, "robot_2", m_unRobot[2], (UInt32) 6);
      GetNodeAttributeOrDefault(cParametersNode, "robot_3", m_unRobot[3], (UInt32) 8);
      GetNodeAttributeOrDefault(cParametersNode, "robot_4", m_unRobot[4], (UInt32) 10);
      GetNodeAttributeOrDefault(cParametersNode, "robot_5", m_unRobot[5], (UInt32) 12);
      GetNodeAttributeOrDefault(cParametersNode, "robot_6", m_unRobot[6], (UInt32) 14);
      GetNodeAttributeOrDefault(cParametersNode, "robot_7", m_unRobot[7], (UInt32) 16);
    } catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }

    m_cUVColor.SetRed(128);
    m_cUVColor.SetGreen(0);
    m_cUVColor.SetBlue(128);

    InitRobotStates();
    InitPhormicaState();
    InitMocaState();

    //init ROS
    std::stringstream name;
    name.str("");
    name << "argos_odom_subscriber";
    if(!ros::isInitialized())
    {
      char** argv = NULL;
      int argc = 0;
      ros::init(argc, argv, name.str());
    }

    //ROS access node
    ros::NodeHandle rosNode;
    std::stringstream ss;

    for (int i = 0; i < 8; ++i)
    {
      //init messages
      ss.str("");
      ss << "/epuck" << m_unRobot[i] << "/odom";
      odomSubscriber[i] = rosNode.subscribe(ss.str(), 1, &HabAgLoopFunction::OdomCallback, this);  
    }

}

/****************************************/
/****************************************/

void HabAgLoopFunction::Reset() {
    CoreLoopFunctions::Reset();


    m_pcPhormica->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
    m_unClock = 0;
    m_unStopBlock = 0;
    m_unStopTime = 0;
    m_fObjectiveFunction = 0;

    m_tRobotStates.clear();
    m_tLEDStates.clear();

    InitMocaState();
    InitRobotStates();
    InitPhormicaState();
}

/****************************************/
/****************************************/

void HabAgLoopFunction::PostStep() {

    ros::spinOnce();

    m_unClock = GetSpace().GetSimulationClock();

    ScoreControl();
    UpdatePhormicaState();
    //LOG << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

void HabAgLoopFunction::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{   
    std::stringstream ss;
    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("epuck");
    UInt32 i = 0;
    //odom
    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it) {
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);

        ss << "epuck" << i;

        if (ss.str() == msg->header.frame_id)
        {
            CVector3 cEpuckPosition (msg->pose.pose.position.x, msg->pose.pose.position.y, 0);
            MoveEntity(cEPuck.GetEmbodiedEntity(), cEpuckPosition, CQuaternion(msg->pose.pose.orientation.w,
                                                                                msg->pose.pose.orientation.x, 
                                                                                msg->pose.pose.orientation.y, 
                                                                                msg->pose.pose.orientation.z));
            break;
        }

        i++;
        ss.str("");        
    }
}

/****************************************/
/****************************************/

void HabAgLoopFunction::PostExperiment() {
    if (m_bMaximization == true){
        LOG << -m_fObjectiveFunction << std::endl;
    }
    else {
        LOG << m_fObjectiveFunction << std::endl;
    }
}

/****************************************/
/****************************************/

Real HabAgLoopFunction::GetObjectiveFunction() {
    if (m_bMaximization == true){
        return -m_fObjectiveFunction;
    }
    else {
        return m_fObjectiveFunction;
    }
}

/****************************************/
/****************************************/

void HabAgLoopFunction::MocaControl() {

    if (m_unClock == m_unStopTime) {
        CSpace::TMapPerType& tBlocksMap = GetSpace().GetEntitiesByType("block");
        UInt32 unBlocksID = 0;
        for (CSpace::TMapPerType::iterator it = tBlocksMap.begin(); it != tBlocksMap.end(); ++it) {
            CBlockEntity* pcBlock = any_cast<CBlockEntity*>(it->second);
            if (unBlocksID == m_unStopBlock) {
                pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::GREEN);
                break;
            }
            unBlocksID += 1;
        }
    }
}

/****************************************/
/****************************************/

void HabAgLoopFunction::ScoreControl(){

    m_fObjectiveFunction += GetAggregationScore(); // check this,

}

/****************************************/
/****************************************/

Real HabAgLoopFunction::GetAggregationScore() {

    UpdateRobotPositions();

    Real fScore = 0;
    Real fDistance = 0;
    TRobotStateMap::iterator it;
    TRobotStateMap::iterator jt;
    for (it = m_tRobotStates.begin(); it != m_tRobotStates.end(); ++it) {
        for (jt = it; jt != m_tRobotStates.end(); ++jt) {
            if (it != jt){
                Real d = (it->second.cPosition - jt->second.cPosition).Length();
                fDistance = fDistance + d;
            }
        }
    }

    fScore = fDistance / m_tRobotStates.size();

    return fScore;
}

/****************************************/
/****************************************/

argos::CColor HabAgLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {

    return CColor::WHITE;
}

/****************************************/
/****************************************/

void HabAgLoopFunction::UpdateRobotPositions() {
    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                           pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        m_tRobotStates[pcEpuck].cLastPosition = m_tRobotStates[pcEpuck].cPosition;
        m_tRobotStates[pcEpuck].cPosition = cEpuckPosition;

        // Updating the Pheromone trail width parameter w.r.t ON UV LEDs

        CColor cEpuckUVOne = pcEpuck->GetLEDEquippedEntity().GetLED(11).GetColor();
        CColor cEpuckUVTwo = pcEpuck->GetLEDEquippedEntity().GetLED(12).GetColor();
        CColor cEpuckUVThree = pcEpuck->GetLEDEquippedEntity().GetLED(13).GetColor();

        if (cEpuckUVOne == CColor::BLACK && cEpuckUVTwo == m_cUVColor && cEpuckUVThree == CColor::BLACK){
            //m_fPheromoneParameter = 0.02; // Thin pheromone trail
            m_tRobotStates[pcEpuck].unPheromoneLEDs = 3;
        }
        else if (cEpuckUVOne == m_cUVColor && cEpuckUVTwo == m_cUVColor && cEpuckUVThree == m_cUVColor){
            //m_fPheromoneParameter = 0.045; // Thick pheromone trails
            m_tRobotStates[pcEpuck].unPheromoneLEDs = 9;
        }
        else {
            //m_fPheromoneParameter = 0; // No pheromone trail
            m_tRobotStates[pcEpuck].unPheromoneLEDs = 0;
        }
        //LOG<< m_fPheromoneParameter<< "Robot update LED color" << std::endl;
    }
}

/****************************************/
/****************************************/

void HabAgLoopFunction::InitRobotStates() {

    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                           pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        m_tRobotStates[pcEpuck].cLastPosition = cEpuckPosition;
        m_tRobotStates[pcEpuck].cPosition = cEpuckPosition;
        m_tRobotStates[pcEpuck].unPheromoneLEDs = 0;
        m_tRobotStates[pcEpuck].unItem = 0;
    }
}

/****************************************/
/****************************************/

void HabAgLoopFunction::InitPhormicaState() {

    CSpace::TMapPerType& tPhormicaMap = GetSpace().GetEntitiesByType("phormica");
    CVector2 cLEDPosition(0,0);
    for (CSpace::TMapPerType::iterator it = tPhormicaMap.begin(); it != tPhormicaMap.end(); ++it) {
        CPhormicaEntity* pcPhormica = any_cast<CPhormicaEntity*>(it->second);
        m_pcPhormica = pcPhormica;
        m_pcPhormica->GetLEDEquippedEntity().Enable();
        m_pcPhormica->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
        m_unNumberLEDs = m_pcPhormica->GetLEDEquippedEntity().GetLEDs().size();
        for (UInt32 i = 0; i < m_unNumberLEDs; ++i) {
            cLEDPosition.Set(m_pcPhormica->GetLEDEquippedEntity().GetLED(i).GetPosition().GetX(),
                             m_pcPhormica->GetLEDEquippedEntity().GetLED(i).GetPosition().GetY());
            m_tLEDStates[i].unLEDIndex = i;
            m_tLEDStates[i].cLEDPosition = cLEDPosition;
            m_tLEDStates[i].unTimer = 0;
        }
    }
}

/****************************************/
/****************************************/

void HabAgLoopFunction::UpdatePhormicaState() {

    TLEDStateMap::iterator itLED;
    TRobotStateMap::iterator it;
    for (itLED = m_tLEDStates.begin(); itLED != m_tLEDStates.end(); ++itLED) {

        for (it = m_tRobotStates.begin(); it != m_tRobotStates.end(); ++it) {
            Real d = (itLED->second.cLEDPosition - it->second.cPosition).Length();
            Real fPheromone = 0;

            if (it->second.unPheromoneLEDs <= 0)
                fPheromone = 0;
            else if (it->second.unPheromoneLEDs <= 3)
                fPheromone = 0.02;
            else
                fPheromone = 0.045;

            //if (d <= m_fPheromoneParameter) {
            if (d <= fPheromone) {
                itLED->second.unTimer = 400; // Pheromone decay time
                m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::MAGENTA);
                //LOG << "Colored LED: " << itLED->second.unLEDIndex << std::endl;
            }
        }

        UInt32 unLEDTimer = itLED->second.unTimer;
        if (unLEDTimer == 0){
            m_pcPhormica->GetLEDEquippedEntity().SetLEDColor(itLED->second.unLEDIndex,CColor::BLACK);
        }
        else {
            itLED->second.unTimer = unLEDTimer - 1;
        }
    }
}

/****************************************/
/****************************************/

void HabAgLoopFunction::InitMocaState() {

    CSpace::TMapPerType& tBlocksMap = GetSpace().GetEntitiesByType("block");
    for (CSpace::TMapPerType::iterator it = tBlocksMap.begin(); it != tBlocksMap.end(); ++it) {
        CBlockEntity* pcBlock = any_cast<CBlockEntity*>(it->second);
        pcBlock->GetLEDEquippedEntity().Enable();
        pcBlock->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
    }
}

/****************************************/
/****************************************/

CVector3 HabAgLoopFunction::GetRandomPosition() {
  Real temp;
  Real a = m_pcRng->Uniform(CRange<Real>(-1.0f, 1.0f));
  Real b = m_pcRng->Uniform(CRange<Real>(-1.0f, 1.0f));
  Real c = m_pcRng->Uniform(CRange<Real>(-1.0f, 1.0f));
  Real d = m_pcRng->Uniform(CRange<Real>(-1.0f, 1.0f));
  // If b < a, swap them
  if (b < a) {
    temp = a;
    a = b;
    b = temp;
  }
  m_fDistributionRadius = 0.35;
  Real fPosX = (c * m_fDistributionRadius / 2) + m_fDistributionRadius * cos(2 * -CRadians::PI_OVER_TWO .GetValue() * (a/b));
  Real fPosY = (d * 1.4 * m_fDistributionRadius / 2) + 1.4 * m_fDistributionRadius * sin(2 * -CRadians::PI_OVER_TWO.GetValue() * (a/b));

  return CVector3(fPosX, fPosY, 0);
}

/****************************************/
/****************************************/

UInt32 HabAgLoopFunction::GetRandomTime(UInt32 unMin, UInt32 unMax) {
  UInt32 unStopAt = m_pcRng->Uniform(CRange<UInt32>(unMin, unMax));
  return unStopAt;

}

/****************************************/
/****************************************/

bool HabAgLoopFunction::IsEven(UInt32 unNumber) {
    bool even;
    if((unNumber%2)==0)
       even = true;
    else
       even = false;

    return even;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(HabAgLoopFunction, "hab_ag_loop_function");
