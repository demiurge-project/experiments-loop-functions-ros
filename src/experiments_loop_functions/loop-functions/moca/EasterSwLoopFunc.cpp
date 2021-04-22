/**
  * @file <loop-functions/IcraLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "EasterSwLoopFunc.h"

/****************************************/
/****************************************/

EasterSwLoopFunction::EasterSwLoopFunction() {
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

    m_bTimeout = false;
    m_unTimer = 0;

    m_fRadius = 0.06;
    m_fSecRadius = 0.095;
    m_unActiveEgg = 0;
    m_tEggPositions.resize(23);
    m_tEggPositions[1] = CVector2(0.0,0.75);
    m_tEggPositions[2] = CVector2(0.0,0.50);
    m_tEggPositions[3] = CVector2(0.0,0.25);
    m_tEggPositions[4] = CVector2(0.0,0.0);
    m_tEggPositions[5] = CVector2(0.0,-0.25);
    m_tEggPositions[6] = CVector2(0.0,-0.50);
    m_tEggPositions[7] = CVector2(0.0,-0.75);
    m_tEggPositions[8] = CVector2(0.25,0.75);
    m_tEggPositions[9] = CVector2(0.25,0.50);
    m_tEggPositions[10] = CVector2(0.25,0.25);
    m_tEggPositions[11] = CVector2(0.25,0.0);
    m_tEggPositions[12] = CVector2(0.25,-0.25);
    m_tEggPositions[13] = CVector2(0.25,-0.50);
    m_tEggPositions[14] = CVector2(0.25,-0.75);
    m_tEggPositions[15] = CVector2(0.50,0.50);
    m_tEggPositions[16] = CVector2(0.50,0.25);
    m_tEggPositions[17] = CVector2(0.50,0.0);
    m_tEggPositions[18] = CVector2(0.50,-0.25);
    m_tEggPositions[19] = CVector2(0.50,-0.50);
    m_tEggPositions[20] = CVector2(0.75,0.25);
    m_tEggPositions[21] = CVector2(0.75,0.0);
    m_tEggPositions[22] = CVector2(0.75,-0.25);

    m_vEggOrder.resize(23);
    m_vEggOrder[0] = 0;
    m_vEggOrder[1] = 17;
    m_vEggOrder[2] = 3;
    m_vEggOrder[3] = 16;
    m_vEggOrder[4] = 5;
    m_vEggOrder[5] = 18;
    m_vEggOrder[6] = 22;
    m_vEggOrder[7] = 8;
    m_vEggOrder[8] = 10;
    m_vEggOrder[9] = 2;
    m_vEggOrder[10] = 14;
    m_vEggOrder[11] = 15;
    m_vEggOrder[12] = 12;
    m_vEggOrder[13] = 20;
    m_vEggOrder[14] = 21;
    m_vEggOrder[15] = 19;
    m_vEggOrder[16] = 6;
    m_vEggOrder[17] = 4;
    m_vEggOrder[18] = 13;
    m_vEggOrder[19] = 7;
    m_vEggOrder[20] = 11;
    m_vEggOrder[21] = 1;
    m_vEggOrder[22] = 9;

}

/****************************************/
/****************************************/

EasterSwLoopFunction::EasterSwLoopFunction(const EasterSwLoopFunction& orig) {
}

/****************************************/
/****************************************/

EasterSwLoopFunction::~EasterSwLoopFunction() {}

/****************************************/
/****************************************/

void EasterSwLoopFunction::Destroy() {
    m_tRobotStates.clear();
    m_tSourceItems.clear();
    m_tSourceOperation.clear();
    m_tSourceReparation.clear();
    m_tSourceRestoring.clear();
    m_tArenaPoints.clear();
    m_tEggPositions.clear();
    RemoveArena();
}

/****************************************/
/****************************************/

void EasterSwLoopFunction::Init(TConfigurationNode& t_tree) {

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
    for (int i = 0; i < 6; ++i)
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

void EasterSwLoopFunction::Reset() {
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

void EasterSwLoopFunction::PreStep() {

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
      //LOG << cEpuckColor << std::endl;
  }

}


void EasterSwLoopFunction::PostStep() {

    m_unClock = GetSpace().GetSimulationClock();

    ScoreControl();
    ArenaControl();

    LOG << "Current score: " << m_fObjectiveFunction << std::endl;

}

/****************************************/
/****************************************/

void EasterSwLoopFunction::PostExperiment() {

    LOG << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real EasterSwLoopFunction::GetObjectiveFunction() {

        return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

void EasterSwLoopFunction::ArenaControl() {

    if (m_unClock == 1) {
        m_pcArena->SetArenaColor(CColor::BLACK);
        m_pcArena->SetWallColor(2,CColor::BLUE);
        m_pcArena->SetWallColor(1,CColor::BLUE);
        m_pcArena->SetWallColor(8,CColor::BLUE);
        m_pcArena->SetWallColor(7,CColor::BLUE);
        m_pcArena->SetWallColor(6,CColor::BLUE);

        LOG << "Eggs appeared in spot: " << m_vEggOrder[m_unActiveEgg] << std::endl; 
    }
    return;

    

}

/****************************************/
/****************************************/

void EasterSwLoopFunction::ScoreControl(){

    m_fObjectiveFunction += GetScore(1);
}

/****************************************/
/****************************************/

Real EasterSwLoopFunction::GetScore(UInt32 unTask) {

    Real unScore = 0;
    unScore = -GetTransportScore();

    return unScore;
}

/****************************************/
/****************************************/

Real EasterSwLoopFunction::GetTransportScore() {

    UpdateRobotPositions();

    bool bInNest;
    UInt32 unInSource = 0;
    Real unScore = 0;
    TRobotStateMap::iterator it;

    if (m_bTimeout == false)
    {
        for (UInt32 unSources=1; unSources < 23; ++unSources) {
        m_tSourceRestoring[unSources] = 0;
        }

        for (it = m_tRobotStates.begin(); it != m_tRobotStates.end(); ++it) {
            unInSource = IsRobotInSourceID(it->second.cPosition);

            if (m_unActiveEgg < 23 && it->second.bItem == false)
            {
                if (unInSource == m_vEggOrder[m_unActiveEgg]){
                    m_tSourceRestoring[unInSource] += 1;
                }
            }   
        }

        for (UInt32 unSources=1; unSources < 23; ++unSources) {
            if (m_tSourceReparation[unSources] >= 120){
                SetEggColor(m_vEggOrder[m_unActiveEgg],CColor::GREEN);
                LOG << "Eggs delivered in spot: " << m_vEggOrder[m_unActiveEgg] << std::endl;
                m_tSourceReparation[unSources] += 1;
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
                    LOG << "Robot droping an egg: " << std::distance(m_tRobotStates.begin(), it) << std::endl;
                }
            }
            else {
                unInSource = IsRobotInSourceID(it->second.cPosition);

                if (m_unActiveEgg < 23) {
                    if (unInSource == m_vEggOrder[m_unActiveEgg]) {

                        if (m_tSourceReparation[unInSource] >= 121) {
                            it->second.bItem = true;
                            LOG << "Robot getting an egg: " << std::distance(m_tRobotStates.begin(), it) << std::endl;
                            m_bTimeout = true;                         
                        }
                    }
                } 
            }
        }
    }
    else
    {
        if (m_unTimer < 20)
        {
            m_unTimer += 1;
        }
        else
        {
            m_unTimer = 0;
            m_bTimeout = false;
            SetEggsOff();
            m_tSourceReparation[m_vEggOrder[m_unActiveEgg]] = 0;
            m_unActiveEgg += 1;            
            if (m_unActiveEgg < 23)
            {                
                SetEggColor(m_vEggOrder[m_unActiveEgg],CColor::RED);
                LOG << "Eggs appeared in spot: " << m_vEggOrder[m_unActiveEgg] << std::endl;
            }
            
        }        
    }

    return unScore;
}

/****************************************/
/****************************************/

argos::CColor EasterSwLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {

    CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());

    Real fdistance = 0;

    for (UInt32 unEggs=1; unEggs < 23; ++unEggs) {
        fdistance = (m_tEggPositions[unEggs] - vCurrentPoint).Length();
        if (fdistance <= m_fRadius) {
        return CColor::BLACK;
        }
    }

    if (c_position_on_plane.GetX() <= -0.625){
        return CColor::WHITE;            
    }
  


    // if (c_position_on_plane.GetX() >= -0.125 && c_position_on_plane.GetX() <= 0.125){
    //     if (c_position_on_plane.GetY() >= 0.65033 || c_position_on_plane.GetY() <= -0.65033)
    //         return CColor::BLACK;
    //     else if (c_position_on_plane.GetY() >= -0.125 && c_position_on_plane.GetY() <= 0.125)
    //         return CColor::WHITE;
    // }

    // else if (c_position_on_plane.GetY() >= -0.125 && c_position_on_plane.GetY() <= 0.125){
    //     if (c_position_on_plane.GetX() >= 0.65033 || c_position_on_plane.GetX() <= -0.65033)
    //         return CColor::BLACK;
    // }
    // else if (c_position_on_plane.GetY() <=  c_position_on_plane.GetX() + 0.1767766953 &&
    //          c_position_on_plane.GetY() >=  c_position_on_plane.GetX() - 0.1767766953){
    //     if ( c_position_on_plane.GetY() >= -c_position_on_plane.GetX() + 0.9267766094 ||
    //          c_position_on_plane.GetY() <= -c_position_on_plane.GetX() - 0.9267766094)
    //         return CColor::BLACK;
    // }

    // else if (c_position_on_plane.GetY() <= -c_position_on_plane.GetX() + 0.1767766953 &&
    //          c_position_on_plane.GetY() >= -c_position_on_plane.GetX() - 0.1767766953){
    //     if ( c_position_on_plane.GetY() >=  c_position_on_plane.GetX() + 0.9267766094 ||
    //          c_position_on_plane.GetY() <=  c_position_on_plane.GetX() - 0.9267766094)
    //         return CColor::BLACK;
    // }

    return CColor::GRAY50;
}

/****************************************/
/****************************************/

bool EasterSwLoopFunction::IsRobotInNest (CVector2 tRobotPosition) {

    if (tRobotPosition.GetX() <= -0.66)
        return true;

    // if (tRobotPosition.GetX() >= -0.16 && tRobotPosition.GetX() <= 0.16 &&
    //     tRobotPosition.GetY() >= -0.16 && tRobotPosition.GetY() <= 0.16)
    //     return true;

    return false;
}

/****************************************/
/****************************************/

UInt32 EasterSwLoopFunction::IsRobotInSourceID (CVector2 tRobotPosition){

    UInt32 unSourceId = 0;
    Real fdistance = 0;

    if (tRobotPosition.Length() >= -0.15) {

        for (UInt32 unEggs=1; unEggs < 23; ++unEggs) {
            fdistance = (m_tEggPositions[unEggs] - tRobotPosition).Length();
            if (fdistance <= m_fSecRadius ) {
                unSourceId = unEggs;
            }
        }
    }

    // if (tRobotPosition.Length() >= 0.6) {

    //     if (tRobotPosition.GetX() >= -0.16 && tRobotPosition.GetX() <= 0.16){
    //         if (tRobotPosition.GetY() >= 0.61533)
    //             unSourceId = 2;
    //         else if (tRobotPosition.GetY() <= -0.61533)
    //             unSourceId = 6;
    //     }
    //     else if (tRobotPosition.GetY() >= -0.16 && tRobotPosition.GetY() <= 0.16){
    //         if (tRobotPosition.GetX() >= 0.61533)
    //             unSourceId = 8;
    //         else if (tRobotPosition.GetX() <= -0.61533)
    //             unSourceId = 4;
    //     }
    //     else if (tRobotPosition.GetY() <=  tRobotPosition.GetX() + 0.22450640303 &&
    //              tRobotPosition.GetY() >=  tRobotPosition.GetX() - 0.22450640303){
    //         if ( tRobotPosition.GetY() >= -tRobotPosition.GetX() + 0.87727913472)
    //             unSourceId = 1;
    //         else if (tRobotPosition.GetY() <= -tRobotPosition.GetX() - 0.87727913472)
    //             unSourceId = 5;
    //     }

    //     else if (tRobotPosition.GetY() <= -tRobotPosition.GetX() + 0.22450640303 &&
    //              tRobotPosition.GetY() >= -tRobotPosition.GetX() - 0.22450640303){
    //         if ( tRobotPosition.GetY() >=  tRobotPosition.GetX() + 0.87727913472)
    //             unSourceId = 3;
    //         else if (tRobotPosition.GetY() <=  tRobotPosition.GetX() - 0.87727913472)
    //             unSourceId = 7;
    //     }
    // }

    return unSourceId;
}

/****************************************/
/****************************************/

void EasterSwLoopFunction::UpdateRobotPositions() {
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

void EasterSwLoopFunction::InitRobotStates() {

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

void EasterSwLoopFunction::InitSources() {

    for (UInt32 unSources=1; unSources < 23; ++unSources) {
        m_tSourceItems[unSources] = 0;
        m_tSourceOperation[unSources] = 0;
        m_tSourceRestoring[unSources] = 0;
        m_tSourceReparation[unSources] = 0;
    }

    SetEggsOff();
    m_unActiveEgg = 1;
    SetEggColor(m_vEggOrder[m_unActiveEgg], CColor::RED);

}

/****************************************/
/****************************************/

CVector3 EasterSwLoopFunction::GetRandomPosition() {

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
m_fDistributionRadius = 0.3;
Real fPosX = -abs((c * m_fDistributionRadius / 2) + m_fDistributionRadius * cos(2 * CRadians::PI.GetValue() * (a/b))) - 0.15;
Real fPosY = (d * m_fDistributionRadius / 2) + m_fDistributionRadius * sin(2 * CRadians::PI.GetValue() * (a/b));

return CVector3(fPosX, fPosY, 0);
}

/****************************************/
/****************************************/

void EasterSwLoopFunction::PositionArena() {
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

void EasterSwLoopFunction::RemoveArena() {
    std::ostringstream id;
    id << "arena";
    RemoveEntity(id.str().c_str());
}

/****************************************/
/****************************************/

Real EasterSwLoopFunction::GetArenaRadious() {

    Real fRadious;
    fRadious =  (m_fLenghtBoxes*m_unNumberBoxes) / (2 * Tan(CRadians::PI / m_unNumberEdges));
    //fRadious = fRadious - 0.10; // Avoids to place robots close the walls.
    fRadious = fRadious - 0.65; // Reduced cluster at the begining

    return fRadious;
}

/****************************************/
/****************************************/

bool EasterSwLoopFunction::IsEven(UInt32 unNumber) {
    bool even;
    if((unNumber%2)==0)
       even = true;
    else
       even = false;

    return even;
}

/****************************************/
/****************************************/

void EasterSwLoopFunction::SetEggsOff() {
    CSpace::TMapPerType& tLEDMap = GetSpace().GetEntitiesByType("box");
    for (CSpace::TMapPerType::iterator it = tLEDMap.begin(); it != tLEDMap.end(); ++it) {
        CBoxEntity* pcBox = any_cast<CBoxEntity*>(it->second);
        pcBox->GetLEDEquippedEntity().SetAllLEDsColors(CColor::BLACK);
    }
}

/****************************************/
/****************************************/

void EasterSwLoopFunction::SetEggColor(UInt32 unEggNumber, CColor ccEggColor) {
    CSpace::TMapPerType& tLEDMap = GetSpace().GetEntitiesByType("box");
    for (CSpace::TMapPerType::iterator it = tLEDMap.begin(); it != tLEDMap.end(); ++it) {
        CBoxEntity* pcBox = any_cast<CBoxEntity*>(it->second);
        pcBox->GetLEDEquippedEntity().SetLEDColor(unEggNumber-1,ccEggColor);
    }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(EasterSwLoopFunction, "easter_sw_loop_function");
