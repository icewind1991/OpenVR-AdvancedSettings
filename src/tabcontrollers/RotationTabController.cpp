#include "RotationTabController.h"
#include <QQuickWindow>
#include "../overlaycontroller.h"
#include "../settings/settings.h"
#include "../utils/Matrix.h"
#include "../quaternion/quaternion.h"
#include <cmath>

// application namespace
namespace advsettings
{
void RotationTabController::initStage1()
{ // TODO adjust update rate
  // m_chaperoneSettingsUpdateCounter
  //   = utils::adjustUpdateRate( k_chaperoneSettingsUpdateCounter );
}

void RotationTabController::initStage2( OverlayController* var_parent )
{
    const auto autoturnOverlayKey
        = std::string( application_strings::applicationKey )
          + ".autoturnnotification";

    const auto overlayError
        = vr::VROverlay()->CreateOverlay( autoturnOverlayKey.c_str(),
                                          autoturnOverlayKey.c_str(),
                                          &m_autoturnValues.overlayHandle );
    if ( overlayError != vr::VROverlayError_None )
    {
        LOG( ERROR ) << "Could not create autoturn notification overlay: "
                     << vr::VROverlay()->GetOverlayErrorNameFromEnum(
                            overlayError );

        emit defaultProfileDisplay();

        return;
    }

    constexpr auto autoturnIconFilepath = "/res/img/rotation/autoturn.png";
    constexpr auto noautoturnIconFilepath = "/res/img/rotation/noautoturn.png";
    constexpr auto alignPointOneIconFilepath
        = "/res/img/rotation/autoalign1.png";
    constexpr auto alignPointTwoIconFilepath
        = "/res/img/rotation/autoalign2.png";
    constexpr auto alignPointThreeIconFilepath
        = "/res/img/rotation/autoalign3.png";
    constexpr auto alignPointFourIconFilepath
        = "/res/img/rotation/autoalign4.png";

    const auto autoturnIconFilePath
        = paths::verifyIconFilePath( autoturnIconFilepath );
    const auto noautoturnIconFilePath
        = paths::verifyIconFilePath( noautoturnIconFilepath );
    const auto alignPointOneIconFilePath
        = paths::verifyIconFilePath( alignPointOneIconFilepath );
    const auto alignPointTwoIconFilePath
        = paths::verifyIconFilePath( alignPointTwoIconFilepath );
    const auto alignPointThreeIconFilePath
        = paths::verifyIconFilePath( alignPointThreeIconFilepath );
    const auto alignPointFourIconFilePath
        = paths::verifyIconFilePath( alignPointFourIconFilepath );

    if ( !autoturnIconFilePath.has_value()
         || !noautoturnIconFilePath.has_value() )
    {
        emit defaultProfileDisplay();
        return;
    }

    m_autoturnValues.autoturnPath = *autoturnIconFilePath;
    m_autoturnValues.noautoturnPath = *noautoturnIconFilePath;
    m_autoturnValues.alignPointOnePath = *alignPointOneIconFilePath;
    m_autoturnValues.alignPointTwoPath = *alignPointTwoIconFilePath;
    m_autoturnValues.alignPointThreePath = *alignPointThreeIconFilePath;
    m_autoturnValues.alignPointFourPath = *alignPointFourIconFilePath;

    auto pushToPath = m_autoturnValues.autoturnPath.c_str();

    vr::VROverlay()->SetOverlayFromFile( m_autoturnValues.overlayHandle,
                                         pushToPath );
    vr::VROverlay()->SetOverlayWidthInMeters( m_autoturnValues.overlayHandle,
                                              0.02f );
    vr::HmdMatrix34_t notificationTransform
        = { { { 1.0f, 0.0f, 0.0f, 0.12f },
              { 0.0f, 1.0f, 0.0f, 0.08f },
              { 0.0f, 0.0f, 1.0f, -0.3f } } };
    vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(
        m_autoturnValues.overlayHandle,
        vr::k_unTrackedDeviceIndex_Hmd,
        &notificationTransform );

    emit defaultProfileDisplay();

    this->parent = var_parent;
}

void RotationTabController::eventLoopTick(
    vr::TrackedDevicePose_t* devicePoses )
{
    if ( devicePoses )
    {
        auto leftHandId
            = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(
                vr::TrackedControllerRole_LeftHand );
        if ( vr::k_unTrackedDeviceIndexInvalid
             && devicePoses[leftHandId].bPoseIsValid
             && devicePoses[leftHandId].eTrackingResult
                    == vr::TrackingResult_Running_OK )
        {
            lastLeftHandPose = devicePoses[leftHandId];
        }
        auto rightHandId
            = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(
                vr::TrackedControllerRole_RightHand );
        if ( vr::k_unTrackedDeviceIndexInvalid
             && devicePoses[rightHandId].bPoseIsValid
             && devicePoses[rightHandId].eTrackingResult
                    == vr::TrackingResult_Running_OK )
        {
            lastRightHandPose = devicePoses[rightHandId];
        }

        m_isHMDActive = false;
        std::lock_guard<std::recursive_mutex> lock(
            parent->chaperoneUtils().mutex() );
        auto& poseHmd = devicePoses[vr::k_unTrackedDeviceIndex_Hmd];

        // m_isHMDActive is true when prox sensor OR HMD is moving (~10 seconds
        // to update from OVR)
        // THIS IS A WORK-AROUND Until proper binding support/calls are made
        // availble for prox sensor
        if ( vr::VRSystem()->GetTrackedDeviceActivityLevel(
                 vr::k_unTrackedDeviceIndex_Hmd )
             == vr::k_EDeviceActivityLevel_UserInteraction )
        {
            m_isHMDActive = true;
        }
        if ( poseHmd.bPoseIsValid && poseHmd.bDeviceIsConnected
             && poseHmd.eTrackingResult == vr::TrackingResult_Running_OK )
        {
            auto chaperoneDistances
                = parent->chaperoneUtils().getDistancesToChaperone(
                    { poseHmd.mDeviceToAbsoluteTracking.m[0][3],
                      poseHmd.mDeviceToAbsoluteTracking.m[1][3],
                      poseHmd.mDeviceToAbsoluteTracking.m[2][3] } );

            // Autoturn mode
            if ( RotationTabController::autoTurnEnabled() )
            {
                doAutoTurn( poseHmd, chaperoneDistances );
            }
            // Vestibular motion. Dependent on autoTurn so the playspace
            // doesn't move when you use the keybind
            if ( RotationTabController::vestibularMotionEnabled() )
            {
                doVestibularMotion( poseHmd, chaperoneDistances );
            }

            if ( RotationTabController::viewRatchettingEnabled() )
            {
                doViewRatchetting( poseHmd, chaperoneDistances );
            }

            m_autoTurnLastHmdUpdate = poseHmd.mDeviceToAbsoluteTracking;
            m_autoTurnChaperoneDistancesLast = std::move( chaperoneDistances );
        }
    }
    if ( m_autoTurnNotificationTimestamp )
    {
        auto count = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now()
                         - *m_autoTurnNotificationTimestamp )
                         .count();
        if ( count > 1500 )
        {
            m_autoTurnNotificationTimestamp.reset();
            if ( getNotificationOverlayHandle()
                 != vr::k_ulOverlayHandleInvalid )
            {
                vr::VROverlay()->HideOverlay( getNotificationOverlayHandle() );
            }
        }
        else
        {
            float pct
                = std::min( 1.0f, 3.0f - static_cast<float>( count ) / 500.0f );
            vr::VROverlay()->SetOverlayAlpha( getNotificationOverlayHandle(),
                                              pct );
        }
    }
}

void RotationTabController::doViewRatchetting(
    const vr::TrackedDevicePose_t& poseHmd,
    const std::vector<utils::ChaperoneQuadData>& chaperoneDistances )
{
    if ( m_isHMDActive && poseHmd.bPoseIsValid && poseHmd.bDeviceIsConnected
         && poseHmd.eTrackingResult == vr::TrackingResult_Running_OK
         && !chaperoneDistances.empty() )
    {
        if ( chaperoneDistances.size()
             != m_autoTurnChaperoneDistancesLast.size() )
        {
            m_autoTurnChaperoneDistancesLast = chaperoneDistances;
            m_autoTurnLastHmdUpdate = poseHmd.mDeviceToAbsoluteTracking;
        }

        // Find the (index of the) nearest wall
        size_t nearestWallIdx = 0;
        for ( size_t i = 0; i < chaperoneDistances.size(); i++ )
        {
            if ( chaperoneDistances[i].distance
                 < chaperoneDistances[nearestWallIdx].distance )
            {
                nearestWallIdx = i;
            }
        }
        const auto& nearestWall = chaperoneDistances[nearestWallIdx];

        // Convert pose matrix to quaternion
        auto hmdQuaternion
            = quaternion::fromHmdMatrix34( poseHmd.mDeviceToAbsoluteTracking );

        // Get HMD raw yaw
        double hmdYaw = quaternion::getYaw( hmdQuaternion );

        // Get angle between HMD position and nearest point on
        // wall
        double hmdPositionToWallYaw = static_cast<double>(
            std::atan2( nearestWall.nearestPoint.v[0]
                            - poseHmd.mDeviceToAbsoluteTracking.m[0][3],
                        nearestWall.nearestPoint.v[2]
                            - poseHmd.mDeviceToAbsoluteTracking.m[2][3] ) );

        // Get angle between HMD and wall
        double hmdToWallYaw
            = reduceAngle<>( hmdYaw - hmdPositionToWallYaw, -M_PI, M_PI );

        do
        {
            // if we change walls, ignore
            if ( nearestWallIdx != m_ratchettingLastWall )
            {
                break;
            }

            // Facing away from the wall is 0, |facing towards the wall| is M_PI
            // Ignore turning towards the wall (e.g. change in magnitude is
            // greater than 0)
            if ( std::abs( hmdToWallYaw )
                     - std::abs( m_ratchettingLastHmdRotation )
                 > 0.0 )
            {
                break;
            }
            // Magnify that change
            double delta_degrees
                = reduceAngle<>(
                      hmdToWallYaw - m_ratchettingLastHmdRotation, -M_PI, M_PI )
                  * viewRatchettingPercent();

            parent->m_moveCenterTabController.setRotation(
                parent->m_moveCenterTabController.rotation()
                + static_cast<int>( delta_degrees * k_radiansToCentidegrees ) );
        } while ( false );

        m_ratchettingLastHmdRotation = hmdToWallYaw;
        m_ratchettingLastWall = nearestWallIdx;
    }
}

void RotationTabController::doVestibularMotion(
    const vr::TrackedDevicePose_t& poseHmd,
    const std::vector<utils::ChaperoneQuadData>& chaperoneDistances )
{
    if ( m_isHMDActive && poseHmd.bPoseIsValid && poseHmd.bDeviceIsConnected
         && poseHmd.eTrackingResult == vr::TrackingResult_Running_OK
         && !chaperoneDistances.empty() )
    {
        if ( chaperoneDistances.size()
             != m_autoTurnChaperoneDistancesLast.size() )
        {
            m_autoTurnChaperoneDistancesLast = chaperoneDistances;
            m_autoTurnLastHmdUpdate = poseHmd.mDeviceToAbsoluteTracking;
        }
        // Rotate dist/(2*pi*r) where r is
        // m_autoTurnVestibularMotionRadius, as if we had walked
        // however many inches along a circle and the world was
        // turning to compensate

        do
        {
            // Find the nearest wall
            const auto& nearestWall
                = std::min_element( chaperoneDistances.begin(),
                                    chaperoneDistances.end(),
                                    []( const auto& quadA, const auto& quadB ) {
                                        return quadA.distance < quadB.distance;
                                    } );

            // Convert pose matrix to quaternion
            auto hmdQuaternion = quaternion::fromHmdMatrix34(
                poseHmd.mDeviceToAbsoluteTracking );

            // Get HMD raw yaw
            double hmdYaw = quaternion::getYaw( hmdQuaternion );

            // Get angle between HMD position and nearest point on
            // wall
            double hmdPositionToWallYaw = static_cast<double>(
                std::atan2( poseHmd.mDeviceToAbsoluteTracking.m[0][3]
                                - nearestWall->nearestPoint.v[0],
                            poseHmd.mDeviceToAbsoluteTracking.m[2][3]
                                - nearestWall->nearestPoint.v[2] ) );

            // Get angle between HMD and wall
            double hmdToWallYaw
                = reduceAngle<>( hmdYaw - hmdPositionToWallYaw, -M_PI, M_PI );
            bool turnLeft = hmdToWallYaw > 0.0;

            // Only turn if moving forward
            double hmdMovementDirection = static_cast<double>(
                std::atan2( m_autoTurnLastHmdUpdate.m[0][3]
                                - poseHmd.mDeviceToAbsoluteTracking.m[0][3],
                            m_autoTurnLastHmdUpdate.m[2][3]
                                - poseHmd.mDeviceToAbsoluteTracking.m[2][3] ) );
            if ( std::abs( hmdYaw - hmdMovementDirection )
                     * k_radiansToCentidegrees
                 >= 7000 )
            {
                break;
            }

            // Get the distance between previous and current
            // position
            double distanceChange = static_cast<double>(
                std::hypot( poseHmd.mDeviceToAbsoluteTracking.m[0][3]
                                - m_autoTurnLastHmdUpdate.m[0][3],
                            poseHmd.mDeviceToAbsoluteTracking.m[2][3]
                                - m_autoTurnLastHmdUpdate.m[2][3] ) );

            // Get the arc length between the previous point and current point
            // 2 sin-1( (d/2)/r ) (in radians)
            double arcLength = 2
                               * std::asin( ( distanceChange / 2 )
                                            / vestibularMotionRadius() );
            if ( std::isnan( arcLength ) )
            {
                break;
            }

            double rotationAmount = arcLength * ( turnLeft ? 1 : -1 );
            int newRotationAngle = reduceAngle<>(
                parent->m_moveCenterTabController.rotation()
                    + static_cast<int>( rotationAmount
                                        * k_radiansToCentidegrees ),
                0,
                36000 );

            parent->m_moveCenterTabController.setRotation( newRotationAngle );

        } while ( false );
    }
}

void RotationTabController::doAutoTurn(
    const vr::TrackedDevicePose_t& poseHmd,
    const std::vector<utils::ChaperoneQuadData>& chaperoneDistances )
{
    if ( m_isHMDActive && poseHmd.bPoseIsValid && poseHmd.bDeviceIsConnected
         && poseHmd.eTrackingResult == vr::TrackingResult_Running_OK
         && !chaperoneDistances.empty() )
    {
        auto currentTime = std::chrono::steady_clock::now();

        if ( m_autoTurnWallActive.size() != chaperoneDistances.size() )
        {
            // Chaperone changed
            m_autoTurnWallActive.clear();
            // Initialize all to 'true' so we don't rotate on startup if
            // the user is outside of the play area.
            m_autoTurnWallActive.resize( chaperoneDistances.size(), true );
            m_autoTurnLastUpdate = currentTime;
        }

        // Apply smooth rotation per-frame: get the time since last frame,
        // multiply by speed
        if ( RotationTabController::autoTurnModeType()
                 == AutoTurnModes::LINEAR_SMOOTH_TURN
             && m_autoTurnLinearSmoothTurnRemaining != 0 )
        {
            double deltaSeconds
                = FrameRates::toDoubleSeconds( m_estimatedFrameRate );
            auto miniDeltaAngle = static_cast<int>(
                std::abs(
                    ( deltaSeconds * RotationTabController::autoTurnSpeed() ) )
                * ( m_autoTurnLinearSmoothTurnRemaining < 0 ? -1 : 1 ) );
            if ( std::abs( m_autoTurnLinearSmoothTurnRemaining )
                 < std::abs( miniDeltaAngle ) )
            {
                miniDeltaAngle = m_autoTurnLinearSmoothTurnRemaining;
            }
            int newRotationAngleDeg
                = reduceAngle<>( parent->m_moveCenterTabController.rotation()
                                     + static_cast<int>( miniDeltaAngle ),
                                 0,
                                 36000 );

            parent->m_moveCenterTabController.setRotation(
                newRotationAngleDeg );
            m_autoTurnLinearSmoothTurnRemaining -= miniDeltaAngle;
        }

        for ( size_t i = 0; i < chaperoneDistances.size(); i++ )
        {
            const auto& chaperoneQuad = chaperoneDistances[i];
            if ( chaperoneQuad.distance
                     <= RotationTabController::autoTurnActivationDistance()
                 && !m_autoTurnWallActive[i] )
            {
                // Convert pose matrix to quaternion
                auto hmdQuaternion = quaternion::fromHmdMatrix34(
                    poseHmd.mDeviceToAbsoluteTracking );

                // Get HMD raw yaw
                double hmdYaw = quaternion::getYaw( hmdQuaternion );

                // Get angle between HMD position and nearest point on
                // wall
                double hmdPositionToWallYaw = static_cast<double>(
                    std::atan2( poseHmd.mDeviceToAbsoluteTracking.m[0][3]
                                    - chaperoneQuad.nearestPoint.v[0],
                                poseHmd.mDeviceToAbsoluteTracking.m[2][3]
                                    - chaperoneQuad.nearestPoint.v[2] ) );

                // Get angle between HMD and wall
                double hmdToWallYaw = reduceAngle<>(
                    hmdYaw - hmdPositionToWallYaw, -M_PI, M_PI );

                do
                {
                    // Ignore if the wall we encountered is behind us
                    if ( std::abs( hmdToWallYaw ) >= M_PI / 2 )
                    {
                        LOG( DEBUG ) << "Ignoring turn in opposite "
                                        "direction (angle "
                                     << std::abs( hmdToWallYaw ) << ")";
                        break;
                    }

                    // If the closest corner shares a wall with the last
                    // wall we turned at, turn relative to that corner
                    bool cornerShared
                        = m_autoTurnWallActive[circularIndex(
                              i, true, chaperoneDistances.size() )]
                          || m_autoTurnWallActive[circularIndex(
                              i, false, chaperoneDistances.size() )];

                    bool turnLeft = true;
                    // Turn away from corner
                    if ( cornerShared )
                    {
                        // Turn left or right depending on which corner
                        // it is. If we go based on yaw, we could end up
                        // turning the wrong way if it's large obtuse
                        // angle and we're facing more towards the
                        // previous wall than the left.
                        turnLeft = m_autoTurnWallActive[circularIndex(
                            i, false, chaperoneDistances.size() )];
                        LOG( DEBUG ) << "turning away from shared corner";
                    }
                    // If within m_cordDetanglingAngle degrees of
                    // 'straight at a wall', start in whatever direction
                    // will start untangling your cord
                    else if ( ( std::abs( hmdToWallYaw )
                                <= RotationTabController::cordDetangleAngle() )
                              && ( std::abs( parent->m_moveCenterTabController
                                                 .getHmdYawTotal() )
                                   > RotationTabController::minCordTangle() ) )
                    {
                        turnLeft = ( parent->m_moveCenterTabController
                                         .getHmdYawTotal()
                                     < 0.0 );
                        LOG( DEBUG ) << "turning to detangle cord";
                    }
                    // Turn the closest angle to the wall
                    else
                    {
                        turnLeft = hmdToWallYaw > 0.0;
                        LOG( DEBUG ) << "turning closest angle to wall";
                    }

                    LOG( DEBUG ) << "hmd yaw " << hmdYaw
                                 << ", hmd position to wall angle "
                                 << hmdPositionToWallYaw;
                    LOG( DEBUG ) << "hmd to wall angle " << hmdToWallYaw;
                    // Positive hmd-to-wall is facing left, negative is
                    // facing right (relative to the wall)
                    double delta_degrees
                        = ( ( turnLeft ? M_PI / 2 : -M_PI / 2 ) - hmdToWallYaw )
                          * k_radiansToCentidegrees;
                    if ( RotationTabController::autoTurnUseCornerAngle()
                         && cornerShared )
                    {
                        // Turn the angle of the corner
                        // Three relevant points - the far corner wall
                        // we're currently touching, the far corner on
                        // the wall we've just touched, and the corner
                        // between them
                        const size_t cornerCnt
                            = parent->chaperoneUtils().quadsCount();
                        const size_t middleCornerIdx
                            = turnLeft ? i
                                       : circularIndex( i, true, cornerCnt );
                        const auto& middleCorner
                            = parent->chaperoneUtils().getCorner(
                                middleCornerIdx );
                        const auto& newWallCorner
                            = parent->chaperoneUtils().getCorner( circularIndex(
                                middleCornerIdx, turnLeft, cornerCnt ) );
                        const auto& touchingWallCorner
                            = parent->chaperoneUtils().getCorner( circularIndex(
                                middleCornerIdx, !turnLeft, cornerCnt ) );

                        double newWallAngle = static_cast<double>( std::atan2(
                            middleCorner.v[0] - newWallCorner.v[0],
                            middleCorner.v[2] - newWallCorner.v[2] ) );
                        double touchingWallAngle
                            = static_cast<double>( std::atan2(
                                middleCorner.v[0] - touchingWallCorner.v[0],
                                middleCorner.v[2] - touchingWallCorner.v[2] ) );
                        LOG( DEBUG ) << "twa: " << touchingWallAngle
                                     << ", nwa: " << newWallAngle << ", diff: "
                                     << ( newWallAngle - touchingWallAngle );
                        delta_degrees = ( newWallAngle - touchingWallAngle
                                          + ( turnLeft ? M_PI : -M_PI ) )
                                        * k_radiansToCentidegrees;
                        delta_degrees
                            = reduceAngle<>( delta_degrees, -18000.0, 18000.0 );
                    }
                    LOG( DEBUG ) << "rotating space " << ( delta_degrees / 100 )
                                 << " degrees";
                    switch ( RotationTabController::autoTurnModeType() )
                    {
                    case AutoTurnModes::SNAP:
                        parent->m_moveCenterTabController.setRotation(
                            parent->m_moveCenterTabController.rotation()
                            + static_cast<int>( delta_degrees ) );
                        break;
                    case AutoTurnModes::LINEAR_SMOOTH_TURN:
                        m_autoTurnLinearSmoothTurnRemaining
                            += static_cast<int>( delta_degrees );
                        break;
                    }
                } while ( false );

                m_autoTurnWallActive[i] = true;
                auto delta = currentTime - m_autoTurnLastUpdate;
                if ( delta
                     < ( FrameRates::RATE_144HZ + FrameRates::RATE_120HZ ) / 2 )
                {
                    m_estimatedFrameRate = FrameRates::RATE_144HZ;
                }
                else if ( ( delta >= ( FrameRates::RATE_144HZ
                                       + FrameRates::RATE_120HZ )
                                         / 2 )
                          && ( delta < ( FrameRates::RATE_120HZ
                                         + FrameRates::RATE_90HZ )
                                           / 2 ) )
                {
                    m_estimatedFrameRate = FrameRates::RATE_120HZ;
                }
                else if ( ( delta >= ( FrameRates::RATE_120HZ
                                       + FrameRates::RATE_90HZ )
                                         / 2 )
                          && ( delta < ( FrameRates::RATE_90HZ
                                         + FrameRates::RATE_72HZ )
                                           / 2 ) )
                {
                    m_estimatedFrameRate = FrameRates::RATE_90HZ;
                }
                else if ( ( delta
                            >= ( FrameRates::RATE_90HZ + FrameRates::RATE_72HZ )
                                   / 2 )
                          && ( delta < ( FrameRates::RATE_72HZ
                                         + FrameRates::RATE_60HZ )
                                           / 2 ) )
                {
                    m_estimatedFrameRate = FrameRates::RATE_72HZ;
                }
                else if ( ( delta
                            >= ( FrameRates::RATE_72HZ + FrameRates::RATE_60HZ )
                                   / 2 )
                          && ( delta < ( FrameRates::RATE_60HZ
                                         + FrameRates::RATE_45HZ )
                                           / 2 ) )
                {
                    m_estimatedFrameRate = FrameRates::RATE_60HZ;
                }
                else
                {
                    m_estimatedFrameRate = FrameRates::RATE_45HZ;
                }
            }
            else if ( ( chaperoneQuad.distance
                        > ( RotationTabController::autoTurnActivationDistance()
                            + RotationTabController::
                                autoTurnDeactivationDistance() ) )
                      && m_autoTurnWallActive[i] )
            {
                m_autoTurnWallActive[i] = false;
            }
        }
        m_autoTurnLastUpdate = currentTime;
    }
    else
    {
        m_autoTurnWallActive.clear();
    }
}

void RotationTabController::addAutoAlignPoint( bool rightHanded )
{
    // TODO: State machine: if we have >2 align points, freeze vestibular
    // motion/ratchetting/etc remain frozen until after we move away from the
    // aligned area
    const auto& lastHandPose
        = rightHanded ? lastRightHandPose : lastLeftHandPose;

    LOG( DEBUG ) << "point added: " << autoAlignPoints.size();
    // get the location of hand, push_back onto autoAlignPoints
    vr::HmdVector3_t new_point
        = { lastHandPose.mDeviceToAbsoluteTracking.m[0][3],
            lastHandPose.mDeviceToAbsoluteTracking.m[1][3],
            lastHandPose.mDeviceToAbsoluteTracking.m[2][3] };
    // TODO: Probably smarter to actually just keep the virtual points as
    // virtual until we use them. Then if they drift we don't care.
    vr::HmdVector3_t absolute_point
        = parent->m_moveCenterTabController.relativeToAbsolute( new_point );
    autoAlignPoints.push_back( absolute_point );

    switch ( autoAlignPoints.size() )
    {
    case 1:
        vr::VROverlay()->SetOverlayFromFile(
            m_autoturnValues.overlayHandle,
            m_autoturnValues.alignPointOnePath.c_str() );
        break;
    case 2:
        vr::VROverlay()->SetOverlayFromFile(
            m_autoturnValues.overlayHandle,
            m_autoturnValues.alignPointTwoPath.c_str() );
        break;
    case 3:
        vr::VROverlay()->SetOverlayFromFile(
            m_autoturnValues.overlayHandle,
            m_autoturnValues.alignPointThreePath.c_str() );
        break;
    case 4:
        vr::VROverlay()->SetOverlayFromFile(
            m_autoturnValues.overlayHandle,
            m_autoturnValues.alignPointFourPath.c_str() );
        break;
    }

    // TODO: configure whether auto-align has HUD popup independently
    if ( autoTurnShowNotification()
         && getNotificationOverlayHandle() != vr::k_ulOverlayHandleInvalid )
    {
        vr::VROverlay()->SetOverlayAlpha( getNotificationOverlayHandle(),
                                          1.0f );
        vr::VROverlay()->ShowOverlay( getNotificationOverlayHandle() );
        m_autoTurnNotificationTimestamp.emplace(
            std::chrono::steady_clock::now() );
    }

    // if we have exactly 4 points, go into main loop
    if ( autoAlignPoints.size() == 4 )
    {
        vr::HmdVector3_t realFirstPoint = autoAlignPoints[0];
        vr::HmdVector3_t realSecondPoint = autoAlignPoints[1];
        vr::HmdVector3_t virtualFirstPoint = autoAlignPoints[2];
        vr::HmdVector3_t virtualSecondPoint = autoAlignPoints[3];

        // Rotate the universe to align, pivoting around the real point
        double realEdgeAngle = static_cast<double>(
            std::atan2( realFirstPoint.v[0] - realSecondPoint.v[0],
                        realFirstPoint.v[2] - realSecondPoint.v[2] ) );
        double virtualEdgeAngle = static_cast<double>(
            std::atan2( virtualFirstPoint.v[0] - virtualSecondPoint.v[0],
                        virtualFirstPoint.v[2] - virtualSecondPoint.v[2] ) );
        double delta_degrees
            = ( realEdgeAngle - virtualEdgeAngle ) * k_radiansToCentidegrees;
        int newRotationAngleDeg = static_cast<int>(
            parent->m_moveCenterTabController.rotation() + delta_degrees );

        // these need to be in the new, offset position. TODO: needs to be
        // converted into new relative position?
        vr::HmdVector3_t autoAlignPivot = realFirstPoint;
        autoAlignPivot.v[0] -= realFirstPoint.v[0] - virtualFirstPoint.v[0];
        autoAlignPivot.v[1] -= realFirstPoint.v[1] - virtualFirstPoint.v[1];
        autoAlignPivot.v[2] -= realFirstPoint.v[2] - virtualFirstPoint.v[2];

        // TODO: if centered, use the center of both points as the pivot
        parent->m_moveCenterTabController.setRotationAroundPivot(
            newRotationAngleDeg,
            true,
            parent->m_moveCenterTabController.absoluteToRelative(
                virtualFirstPoint ) );

        // Align the first of VR points (points[2]) with the first of the real
        // points (points[0]) purely in position
        parent->m_moveCenterTabController.displaceUniverse( virtualFirstPoint,
                                                            realFirstPoint );

        // end of main loop, clear autoAlignPoints
        autoAlignPoints.clear();
    }
}
// getters

bool RotationTabController::autoTurnEnabled() const
{
    return settings::getSetting(
        settings::BoolSetting::ROTATION_autoturnEnabled );
}

float RotationTabController::autoTurnActivationDistance() const
{
    return static_cast<float>( settings::getSetting(
        settings::DoubleSetting::ROTATION_activationDistance ) );
}

float RotationTabController::autoTurnDeactivationDistance() const
{
    return static_cast<float>( settings::getSetting(
        settings::DoubleSetting::ROTATION_deactivateDistance ) );
}

bool RotationTabController::autoTurnUseCornerAngle() const
{
    return settings::getSetting(
        settings::BoolSetting::ROTATION_autoturnUseCornerAngle );
}

bool RotationTabController::autoTurnShowNotification() const
{
    return settings::getSetting(
        settings::BoolSetting::ROTATION_autoturnShowNotification );
}

double RotationTabController::cordDetangleAngle() const
{
    return settings::getSetting(
        settings::DoubleSetting::ROTATION_cordDetanglingAngle );
}

double RotationTabController::minCordTangle() const
{
    return settings::getSetting(
        settings::DoubleSetting::ROTATION_autoturnMinCordTangle );
}

int RotationTabController::autoTurnSpeed() const
{
    return settings::getSetting(
        settings::IntSetting::ROTATION_autoturnLinearTurnSpeed );
}

AutoTurnModes RotationTabController::autoTurnModeType() const
{
    return static_cast<AutoTurnModes>(
        settings::getSetting( settings::IntSetting::ROTATION_autoturnMode ) );
}

int RotationTabController::autoTurnMode() const
{
    return settings::getSetting( settings::IntSetting::ROTATION_autoturnMode );
}

bool RotationTabController::vestibularMotionEnabled() const
{
    return settings::getSetting(
        settings::BoolSetting::ROTATION_autoturnVestibularMotionEnabled );
}

double RotationTabController::vestibularMotionRadius() const
{
    return settings::getSetting(
        settings::DoubleSetting::ROTATION_autoturnVestibularMotionRadius );
}

bool RotationTabController::viewRatchettingEnabled() const
{
    return settings::getSetting(
        settings::BoolSetting::ROTATION_autoturnViewRatchettingEnabled );
}

double RotationTabController::viewRatchettingPercent() const
{
    return settings::getSetting(
        settings::DoubleSetting::ROTATION_autoturnViewRatchettingPercent );
}

// Setters

void RotationTabController::setAutoTurnEnabled( bool value, bool notify )
{
    settings::setSetting( settings::BoolSetting::ROTATION_autoturnEnabled,
                          value );

    if ( notify )
    {
        emit autoTurnEnabledChanged( value );
    }

    if ( !value )
    {
        vr::VROverlay()->SetOverlayFromFile(
            m_autoturnValues.overlayHandle,
            m_autoturnValues.noautoturnPath.c_str() );
    }
    else
    {
        vr::VROverlay()->SetOverlayFromFile(
            m_autoturnValues.overlayHandle,
            m_autoturnValues.autoturnPath.c_str() );
    }

    if ( autoTurnShowNotification()
         && getNotificationOverlayHandle() != vr::k_ulOverlayHandleInvalid )
    {
        vr::VROverlay()->SetOverlayAlpha( getNotificationOverlayHandle(),
                                          1.0f );
        vr::VROverlay()->ShowOverlay( getNotificationOverlayHandle() );
        m_autoTurnNotificationTimestamp.emplace(
            std::chrono::steady_clock::now() );
    }
}
void RotationTabController::setAutoTurnShowNotification( bool value,
                                                         bool notify )
{
    settings::setSetting(
        settings::BoolSetting::ROTATION_autoturnShowNotification, value );
    if ( notify )
    {
        emit autoTurnShowNotificationChanged( value );
    }
}
void RotationTabController::setAutoTurnActivationDistance( float value,
                                                           bool notify )
{
    settings::setSetting( settings::DoubleSetting::ROTATION_activationDistance,
                          static_cast<double>( value ) );
    if ( notify )
    {
        emit autoTurnActivationDistanceChanged( value );
    }
}
void RotationTabController::setAutoTurnDeactivationDistance( float value,
                                                             bool notify )
{
    settings::setSetting( settings::DoubleSetting::ROTATION_deactivateDistance,
                          static_cast<double>( value ) );
    if ( notify )
    {
        emit autoTurnDeactivationDistanceChanged( value );
    }
}

void RotationTabController::setAutoTurnUseCornerAngle( bool value, bool notify )
{
    settings::setSetting(
        settings::BoolSetting::ROTATION_autoturnUseCornerAngle, value );
    if ( notify )
    {
        emit autoTurnUseCornerAngleChanged( value );
    }
}

void RotationTabController::setCordDetangleAngle( double value, bool notify )
{
    settings::setSetting( settings::DoubleSetting::ROTATION_cordDetanglingAngle,
                          value );
    if ( notify )
    {
        emit cordDetangleAngleChanged( value );
    }
}

void RotationTabController::setMinCordTangle( double value, bool notify )
{
    settings::setSetting(
        settings::DoubleSetting::ROTATION_autoturnMinCordTangle, value );
    if ( notify )
    {
        emit minCordTangleChanged( value );
    }
}
void RotationTabController::setAutoTurnSpeed( int value, bool notify )
{
    settings::setSetting(
        settings::IntSetting::ROTATION_autoturnLinearTurnSpeed, value );
    if ( notify )
    {
        emit autoTurnSpeedChanged( value );
    }
}
void RotationTabController::setAutoTurnMode( int value, bool notify )
{
    settings::setSetting( settings::IntSetting::ROTATION_autoturnMode, value );
    if ( notify )
    {
        emit autoTurnModeChanged( value );
    }
}
void RotationTabController::setVestibularMotionEnabled( bool value,
                                                        bool notify )
{
    settings::setSetting(
        settings::BoolSetting::ROTATION_autoturnVestibularMotionEnabled,
        value );
    if ( notify )
    {
        emit vestibularMotionEnabledChanged( value );
    }
}
void RotationTabController::setVestibularMotionRadius( double value,
                                                       bool notify )
{
    settings::setSetting(
        settings::DoubleSetting::ROTATION_autoturnVestibularMotionRadius,
        value );
    if ( notify )
    {
        emit vestibularMotionRadiusChanged( value );
    }
}
void RotationTabController::setViewRatchettingEnabled( bool value, bool notify )
{
    settings::setSetting(
        settings::BoolSetting::ROTATION_autoturnViewRatchettingEnabled, value );
    if ( notify )
    {
        emit viewRatchettingEnabledChanged( value );
    }
}
void RotationTabController::setViewRatchettingPercent( double value,
                                                       bool notify )
{
    settings::setSetting(
        settings::DoubleSetting::ROTATION_autoturnViewRatchettingPercent,
        value );
    if ( notify )
    {
        emit viewRatchettingPercentChanged( value );
    }
}

} // namespace advsettings
