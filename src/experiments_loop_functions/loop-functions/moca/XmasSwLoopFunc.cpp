/**
  * @file <loop-functions/IcraLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "XmasSwLoopFunc.h"

/****************************************/
/****************************************/

XmasSwLoopFunction::XmasSwLoopFunction() {
    m_unClock = 0;
    m_fObjectiveFunction = 0;
    m_fObjectiveFunctionBlue = 0;
    m_fObjectiveFunctionRed = 0;
    m_bBlueFirst = true;
    m_cArenaColor = CColor::BLACK;
    m_cTaskAsignedColorBlue = CColor::BLACK;
    m_cTaskEvalColorBlue = CColor::BLACK;
    m_cTaskAsignedColorRed = CColor::BLACK;
    m_cTaskEvalColorRed = CColor::BLACK;
}

/****************************************/
/****************************************/

XmasSwLoopFunction::XmasSwLoopFunction(const XmasSwLoopFunction& orig) {
}

/****************************************/
/****************************************/

XmasSwLoopFunction::~XmasSwLoopFunction() {}

/****************************************/
/****************************************/

void XmasSwLoopFunction::Destroy() {
    m_tRobotStates.clear();
    m_tSourceItems.clear();
    m_tSourceOperation.clear();
    m_tSourceReparation.clear();
    m_tSourceRestoring.clear();
    m_tArenaPoints.clear();
    RemoveArena();
}

/****************************************/
/****************************************/

void XmasSwLoopFunction::Init(TConfigurationNode& t_tree) {

    CoreLoopFunctions::Init(t_tree);
    TConfigurationNode cParametersNode;

    try {
      cParametersNode = GetNode(t_tree, "params");
      GetNodeAttributeOrDefault(cParametersNode, "build_arena", m_bBuildArena, (bool) false);
      GetNodeAttributeOrDefault(cParametersNode, "number_edges", m_unNumberEdges, (UInt32) 3);
      GetNodeAttributeOrDefault(cParametersNode, "number_boxes_per_edge", m_unNumberBoxes, (UInt32) 1);
      GetNodeAttributeOrDefault(cParametersNode, "lenght_boxes", m_fLenghtBoxes, (Real) 0.25);
    } catch(std::exception e) {
      LOGERR << e.what() << std::endl;
    }

    if (m_bBuildArena == true)
    {
        m_fDistributionRadius = GetArenaRadious();
        PositionArena();
    }

    InitRobotStates();

    InitSources();

    //init ROS
    std::stringstream name;
    name.str("");
    name << "loop";
    if(!ros::isInitialized())
    {
      char** argv = NULL;
      int argc = 0;
      ros::init(argc, argv, name.str());
    }

    //ROS access node
    ros::NodeHandle rosNode;
    std::stringstream ss;
    for (int i = 0; i < 5; ++i)
    {
      //init messages
      ss.str("");
      ss << "/epuck" << i << "/odom";
      odomPublisher[i] = rosNode.advertise<nav_msgs::Odometry>(ss.str(), 10); 
      odomMsg[i].header.frame_id = ss.str();
      
      ss.str("");
      ss << "/epuck" << i << "/base_link";
      odomMsg[i].child_frame_id = ss.str();
      
      ss.str("");
      ss << "/epuck" << i << "/color";
      colorPublisher[i] = rosNode.advertise<std_msgs::ColorRGBA>(ss.str(), 10);
    }

}

/****************************************/
/****************************************/

void XmasSwLoopFunction::Reset() {
    CoreLoopFunctions::Reset();

    m_unClock = 0;
    m_fObjectiveFunction = 0;

    m_tRobotStates.clear();
    m_tSourceItems.clear();
    m_tSourceOperation.clear();
    m_tSourceReparation.clear();
    m_tSourceRestoring.clear();
    m_tArenaPoints.clear();

    InitRobotStates();
    InitSources();

}

/****************************************/
/****************************************/

void XmasSwLoopFunction::PreStep() {

  CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("epuck");
  CColor cEpuckColor;
  UInt8 i = 0;
  //odom
  for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it) {
     CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
     
     odomMsg[i].header.stamp = ros::Time::now();
     odomMsg[i].pose.pose.position.x = cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
     odomMsg[i].pose.pose.position.y = cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
     odomMsg[i].pose.pose.position.z = 0;
     geometry_msgs::Quaternion odomQuat;
     odomQuat.x = cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetX();
     odomQuat.y = cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetY();
     odomQuat.z = cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetZ();
     odomQuat.w = cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetW(); 
     odomMsg[i].pose.pose.orientation = odomQuat;
     odomPublisher[i].publish(odomMsg[i]);

     i++;
  }

  i = 0;

  //color
  for (CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it) {

      CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
      cEpuckColor = pcEpuck->GetLEDEquippedEntity().GetLED(10).GetColor();

      colorMsg[i].r = cEpuckColor.GetRed();
      colorMsg[i].g = cEpuckColor.GetGreen();
      colorMsg[i].b = cEpuckColor.GetBlue();
      colorMsg[i].a = cEpuckColor.GetAlpha();
      colorPublisher[i].publish(colorMsg[i]);

      i++;
      LOG << cEpuckColor << std::endl;
  }

}

/****************************************/
/****************************************/

void XmasSwLoopFunction::PostStep() {

    m_unClock = GetSpace().GetSimulationClock();

    ScoreControl();
    ArenaControl();

}

/****************************************/
/****************************************/

void XmasSwLoopFunction::PostExperiment() {

    LOG << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real XmasSwLoopFunction::GetObjectiveFunction() {

        return m_fObjectiveFunction;

}

/****************************************/
/****************************************/

void XmasSwLoopFunction::ArenaControl() {

    if (m_unClock == 1) {
        m_pcArena->SetArenaColor(CColor::BLACK);
    }
    return;
}

/****************************************/
/****************************************/

void XmasSwLoopFunction::ScoreControl(){

    m_fObjectiveFunction += GetScore(1);
}

/****************************************/
/****************************************/

Real XmasSwLoopFunction::GetScore(UInt32 unTask) {

    Real unScore = 0;
    unScore = -GetTransportScore();

    return unScore;
}

/****************************************/
/****************************************/

Real XmasSwLoopFunction::GetTransportScore() {

    UpdateRobotPositions();

    bool bInNest;
    UInt32 unInSource = 0;
    Real unScore = 0;
    TRobotStateMap::iterator it;

    for (UInt32 unSources=1; unSources < 9; ++unSources) {
        m_tSourceRestoring[unSources] = 0;
    }

    for (it = m_tRobotStates.begin(); it != m_tRobotStates.end(); ++it) {
        unInSource = IsRobotInSourceID(it->second.cPosition);
        if (unInSource != 0){
            m_tSourceRestoring[unInSource] += 1;
        }
    }

    for (UInt32 unSources=1; unSources < 9; ++unSources) {
        if (m_tSourceReparation[unSources] >= 400){
            m_pcArena->SetBoxColor(2,unSources,CColor::GREEN);
        }
        else{
            if (m_tSourceRestoring[unSources] >= 1)
                m_tSourceReparation[unSources] += 1;
            else
                m_tSourceReparation[unSources] = 0;
        }
    }

    for (it = m_tRobotStates.begin(); it != m_tRobotStates.end(); ++it) {

        if (it->second.bItem == true){
            bInNest = IsRobotInNest(it->second.cPosition);
            if (bInNest) {
                unScore+=1;
                it->second.bItem = false;
            }
        }
        else {
            unInSource = IsRobotInSourceID(it->second.cPosition);
            if (unInSource != 0){
                if (m_tSourceReparation[unInSource] >= 400){
                    it->second.bItem = true;
                }
            }
        }
    }

    return unScore;
}

/****************************************/
/****************************************/

argos::CColor XmasSwLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {

    if (c_position_on_plane.GetX() >= -0.125 && c_position_on_plane.GetX() <= 0.125){
        if (c_position_on_plane.GetY() >= 0.65033 || c_position_on_plane.GetY() <= -0.65033)
            return CColor::BLACK;
        else if (c_position_on_plane.GetY() >= -0.125 && c_position_on_plane.GetY() <= 0.125)
            return CColor::WHITE;
    }

    else if (c_position_on_plane.GetY() >= -0.125 && c_position_on_plane.GetY() <= 0.125){
        if (c_position_on_plane.GetX() >= 0.65033 || c_position_on_plane.GetX() <= -0.65033)
            return CColor::BLACK;
    }
    else if (c_position_on_plane.GetY() <=  c_position_on_plane.GetX() + 0.1767766953 &&
             c_position_on_plane.GetY() >=  c_position_on_plane.GetX() - 0.1767766953){
        if ( c_position_on_plane.GetY() >= -c_position_on_plane.GetX() + 0.9267766094 ||
             c_position_on_plane.GetY() <= -c_position_on_plane.GetX() - 0.9267766094)
            return CColor::BLACK;
    }

    else if (c_position_on_plane.GetY() <= -c_position_on_plane.GetX() + 0.1767766953 &&
             c_position_on_plane.GetY() >= -c_position_on_plane.GetX() - 0.1767766953){
        if ( c_position_on_plane.GetY() >=  c_position_on_plane.GetX() + 0.9267766094 ||
             c_position_on_plane.GetY() <=  c_position_on_plane.GetX() - 0.9267766094)
            return CColor::BLACK;
    }

    return CColor::GRAY50;
}

/****************************************/
/****************************************/

bool XmasSwLoopFunction::IsRobotInNest (CVector2 tRobotPosition) {

    if (tRobotPosition.GetX() >= -0.16 && tRobotPosition.GetX() <= 0.16 &&
        tRobotPosition.GetY() >= -0.16 && tRobotPosition.GetY() <= 0.16)
        return true;

    return false;
}

/****************************************/
/****************************************/

UInt32 XmasSwLoopFunction::IsRobotInSourceID (CVector2 tRobotPosition){

    UInt32 unSourceId = 0;

    if (tRobotPosition.Length() >= 0.6) {

        if (tRobotPosition.GetX() >= -0.16 && tRobotPosition.GetX() <= 0.16){
            if (tRobotPosition.GetY() >= 0.61533)
                unSourceId = 2;
            else if (tRobotPosition.GetY() <= -0.61533)
                unSourceId = 6;
        }
        else if (tRobotPosition.GetY() >= -0.16 && tRobotPosition.GetY() <= 0.16){
            if (tRobotPosition.GetX() >= 0.61533)
                unSourceId = 8;
            else if (tRobotPosition.GetX() <= -0.61533)
                unSourceId = 4;
        }
        else if (tRobotPosition.GetY() <=  tRobotPosition.GetX() + 0.22450640303 &&
                 tRobotPosition.GetY() >=  tRobotPosition.GetX() - 0.22450640303){
            if ( tRobotPosition.GetY() >= -tRobotPosition.GetX() + 0.87727913472)
                unSourceId = 1;
            else if (tRobotPosition.GetY() <= -tRobotPosition.GetX() - 0.87727913472)
                unSourceId = 5;
        }

        else if (tRobotPosition.GetY() <= -tRobotPosition.GetX() + 0.22450640303 &&
                 tRobotPosition.GetY() >= -tRobotPosition.GetX() - 0.22450640303){
            if ( tRobotPosition.GetY() >=  tRobotPosition.GetX() + 0.87727913472)
                unSourceId = 3;
            else if (tRobotPosition.GetY() <=  tRobotPosition.GetX() - 0.87727913472)
                unSourceId = 7;
        }
    }

    return unSourceId;
}

/****************************************/
/****************************************/

void XmasSwLoopFunction::UpdateRobotPositions() {
    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                           pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        m_tRobotStates[pcEpuck].cLastPosition = m_tRobotStates[pcEpuck].cPosition;
        m_tRobotStates[pcEpuck].cPosition = cEpuckPosition;
    }
}

/****************************************/
/****************************************/

void XmasSwLoopFunction::InitRobotStates() {

    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                           pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        m_tRobotStates[pcEpuck].cLastPosition = cEpuckPosition;
        m_tRobotStates[pcEpuck].cPosition = cEpuckPosition;
        m_tRobotStates[pcEpuck].cColor = CColor::BLACK;
        m_tRobotStates[pcEpuck].bItem = false;
        m_tRobotStates[pcEpuck].bMaterial = false;
        m_tRobotStates[pcEpuck].bMoving = false;
    }
}

/****************************************/
/****************************************/

void XmasSwLoopFunction::InitSources() {

    for (UInt32 unSources=1; unSources < 9; ++unSources) {
        m_tSourceItems[unSources] = 0;
        m_tSourceOperation[unSources] = 0;
        m_tSourceRestoring[unSources] = 0;
        m_tSourceReparation[unSources] = 0;
    }

    m_tSourceOperation[2] = 0;

}

/****************************************/
/****************************************/

CVector3 XmasSwLoopFunction::GetRandomPosition() {

Real temp;
Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
Real b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
Real c = m_pcRng->Uniform(CRange<Real>(-1.0f, 1.0f));
Real d = m_pcRng->Uniform(CRange<Real>(-1.0f, 1.0f));
// If b < a, swap them
if (b < a) {
  temp = a;
  a = b;
  b = temp;
}
m_fDistributionRadius = 0.4;
Real fPosX = (c * m_fDistributionRadius / 2) + m_fDistributionRadius * cos(2 * CRadians::PI.GetValue() * (a/b));
Real fPosY = (d * m_fDistributionRadius / 2) + m_fDistributionRadius * sin(2 * CRadians::PI.GetValue() * (a/b));

return CVector3(fPosX, fPosY, 0);
}

/****************************************/
/****************************************/

void XmasSwLoopFunction::PositionArena() {
  CArenaEntity* pcArena;

  pcArena = new CArenaEntity("arena",
                             CVector3(0,0,0),
                             CQuaternion().FromEulerAngles(CRadians::ZERO,CRadians::ZERO,CRadians::ZERO), // TODO
                             CVector3(0.01,m_fLenghtBoxes,0.1),
                             "leds",
                             m_unNumberBoxes,
                             m_unNumberEdges,
                             0.125f,
                             1.0f);

  AddEntity(*pcArena);
  m_pcArena = pcArena;
}

/****************************************/
/****************************************/

void XmasSwLoopFunction::RemoveArena() {
    std::ostringstream id;
    id << "arena";
    RemoveEntity(id.str().c_str());
}

/****************************************/
/****************************************/

Real XmasSwLoopFunction::GetArenaRadious() {

    Real fRadious;
    fRadious =  (m_fLenghtBoxes*m_unNumberBoxes) / (2 * Tan(CRadians::PI / m_unNumberEdges));
    //fRadious = fRadious - 0.10; // Avoids to place robots close the walls.
    fRadious = fRadious - 0.65; // Reduced cluster at the begining

    return fRadious;
}

/****************************************/
/****************************************/

bool XmasSwLoopFunction::IsEven(UInt32 unNumber) {
    bool even;
    if((unNumber%2)==0)
       even = true;
    else
       even = false;

    return even;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(XmasSwLoopFunction, "xmas_sw_loop_function");
