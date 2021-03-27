#pragma once

#include <QObject>
#include <memory>
#include <chrono>
#include <thread>
#include <openvr.h>
#include <cmath>
#include <optional>
#include "../utils/FrameRateUtils.h"
#include "../utils/ChaperoneUtils.h"
#include "../settings/settings_object.h"
#include "MoveCenterTabController.h"

class QQuickWindow;
// application namespace
namespace advsettings
{
// forward declaration
class OverlayController;

enum class AutoTurnModes
{
    SNAP,
    LINEAR_SMOOTH_TURN
};

namespace FrameRates
{
    // Each of type std::chrono::time_point::duration
    constexpr auto RATE_45HZ = std::chrono::duration_cast<
        std::chrono::steady_clock::time_point::duration>(
        std::chrono::duration<int, std::ratio<1, 45>>( 1 ) );
    constexpr auto RATE_60HZ = std::chrono::duration_cast<
        std::chrono::steady_clock::time_point::duration>(
        std::chrono::duration<int, std::ratio<1, 60>>( 1 ) );
    constexpr auto RATE_72HZ = std::chrono::duration_cast<
        std::chrono::steady_clock::time_point::duration>(
        std::chrono::duration<int, std::ratio<1, 72>>( 1 ) );
    constexpr auto RATE_90HZ = std::chrono::duration_cast<
        std::chrono::steady_clock::time_point::duration>(
        std::chrono::duration<int, std::ratio<1, 90>>( 1 ) );
    constexpr auto RATE_120HZ = std::chrono::duration_cast<
        std::chrono::steady_clock::time_point::duration>(
        std::chrono::duration<int, std::ratio<1, 120>>( 1 ) );
    constexpr auto RATE_144HZ = std::chrono::duration_cast<
        std::chrono::steady_clock::time_point::duration>(
        std::chrono::duration<int, std::ratio<1, 144>>( 1 ) );

    template <typename T> double toDoubleSeconds( T duration )
    {
        return ( static_cast<double>( duration.count() ) * T::period::num )
               / T::period::den;
    }
} // namespace FrameRates

class RotationTabController : public QObject
{
    Q_OBJECT
    Q_PROPERTY( bool autoTurnEnabled READ autoTurnEnabled WRITE
                    setAutoTurnEnabled NOTIFY autoTurnEnabledChanged )
    Q_PROPERTY( float autoTurnActivationDistance READ autoTurnActivationDistance
                    WRITE setAutoTurnActivationDistance NOTIFY
                        autoTurnActivationDistanceChanged )
    Q_PROPERTY(
        float autoTurnDeactivationDistance READ autoTurnDeactivationDistance
            WRITE setAutoTurnDeactivationDistance NOTIFY
                autoTurnDeactivationDistanceChanged )
    Q_PROPERTY(
        bool autoTurnUseCornerAngle READ autoTurnUseCornerAngle WRITE
            setAutoTurnUseCornerAngle NOTIFY autoTurnUseCornerAngleChanged )
    Q_PROPERTY( double cordDetangleAngle READ cordDetangleAngle WRITE
                    setCordDetangleAngle NOTIFY cordDetangleAngleChanged )
    Q_PROPERTY( double minCordTangle READ minCordTangle WRITE setMinCordTangle
                    NOTIFY minCordTangleChanged )
    Q_PROPERTY( int autoTurnSpeed READ autoTurnSpeed WRITE setAutoTurnSpeed
                    NOTIFY autoTurnSpeedChanged )
    Q_PROPERTY( int autoTurnMode READ autoTurnMode WRITE setAutoTurnMode NOTIFY
                    autoTurnModeChanged )
    Q_PROPERTY(
        bool vestibularMotionEnabled READ vestibularMotionEnabled WRITE
            setVestibularMotionEnabled NOTIFY vestibularMotionEnabledChanged )
    Q_PROPERTY(
        double vestibularMotionRadius READ vestibularMotionRadius WRITE
            setVestibularMotionRadius NOTIFY vestibularMotionRadiusChanged )
    Q_PROPERTY(
        bool viewRatchettingEnabled READ viewRatchettingEnabled WRITE
            setViewRatchettingEnabled NOTIFY viewRatchettingEnabledChanged )
    Q_PROPERTY(
        double viewRatchettingPercent READ viewRatchettingPercent WRITE
            setViewRatchettingPercent NOTIFY viewRatchettingPercentChanged )
    Q_PROPERTY(
        bool autoTurnShowNotification READ autoTurnShowNotification WRITE
            setAutoTurnShowNotification NOTIFY autoTurnShowNotificationChanged )

private:
    OverlayController* parent;

    struct
    {
        vr::VROverlayHandle_t overlayHandle = vr::k_ulOverlayHandleInvalid;
        std::string autoturnPath;
        std::string noautoturnPath;
        std::string alignPointOnePath;
        std::string alignPointTwoPath;
        std::string alignPointThreePath;
        std::string alignPointFourPath;
    } m_autoturnValues;

    virtual vr::VROverlayHandle_t getNotificationOverlayHandle()
    {
        return m_autoturnValues.overlayHandle;
    }

    // Variables
    int m_autoTurnLinearSmoothTurnRemaining = 0;
    std::chrono::steady_clock::time_point m_autoTurnLastUpdate;
    std::vector<bool> m_autoTurnWallActive;
    vr::HmdMatrix34_t m_autoTurnLastHmdUpdate;
    std::vector<utils::ChaperoneQuadData> m_autoTurnChaperoneDistancesLast;
    std::chrono::steady_clock::time_point::duration m_estimatedFrameRate;
    double m_ratchettingLastHmdRotation = 0.0;
    size_t m_ratchettingLastWall = 0;

    std::optional<std::chrono::steady_clock::time_point>
        m_autoTurnNotificationTimestamp;

    std::vector<vr::HmdVector3_t> autoAlignPoints;
    vr::TrackedDevicePose_t lastLeftHandPose, lastRightHandPose;

    bool m_isHMDActive = false;

    void doAutoTurn(
        const vr::TrackedDevicePose_t& poseHmd,
        const std::vector<utils::ChaperoneQuadData>& chaperoneDistances );
    void doVestibularMotion(
        const vr::TrackedDevicePose_t& poseHmd,
        const std::vector<utils::ChaperoneQuadData>& chaperoneDistances );
    void doViewRatchetting(
        const vr::TrackedDevicePose_t& poseHmd,
        const std::vector<utils::ChaperoneQuadData>& chaperoneDistances );

public:
    void initStage1();
    void initStage2( OverlayController* parent );

    void eventLoopTick( vr::TrackedDevicePose_t* devicePoses );

    float boundsVisibility() const;

    bool autoTurnEnabled() const;
    float autoTurnActivationDistance() const;
    float autoTurnDeactivationDistance() const;
    bool autoTurnUseCornerAngle() const;
    bool autoTurnShowNotification() const;
    double cordDetangleAngle() const;
    double minCordTangle() const;
    int autoTurnSpeed() const;
    AutoTurnModes autoTurnModeType() const;
    int autoTurnMode() const;
    bool vestibularMotionEnabled() const;
    double vestibularMotionRadius() const;
    bool viewRatchettingEnabled() const;
    double viewRatchettingPercent() const;

public slots:
    void setAutoTurnEnabled( bool value, bool notify = true );
    void setAutoTurnShowNotification( bool value, bool notify = true );
    void setAutoTurnActivationDistance( float value, bool notify = true );
    void setAutoTurnDeactivationDistance( float value, bool notify = true );
    void setAutoTurnUseCornerAngle( bool value, bool notify = true );
    void setCordDetangleAngle( double value, bool notify = true );
    void setMinCordTangle( double value, bool notify = true );
    void setAutoTurnSpeed( int value, bool notify = true );
    void setAutoTurnMode( int value, bool notify = true );
    void setVestibularMotionEnabled( bool value, bool notify = true );
    void setVestibularMotionRadius( double value, bool notify = true );
    void setViewRatchettingEnabled( bool value, bool notify = true );
    void setViewRatchettingPercent( double value, bool notify = true );

    // Experimental - auto-align
    void addAutoAlignPoint( bool rightHanded );

signals:

    void defaultProfileDisplay();
    void autoTurnEnabledChanged( bool value );
    void autoTurnShowNotificationChanged( bool value );
    void autoTurnActivationDistanceChanged( float value );
    void autoTurnDeactivationDistanceChanged( float value );
    void autoTurnUseCornerAngleChanged( bool value );
    void cordDetangleAngleChanged( double value );
    void minCordTangleChanged( double value );
    void autoTurnSpeedChanged( int value );
    void autoTurnModeChanged( int value );
    void vestibularMotionEnabledChanged( bool value );
    void vestibularMotionRadiusChanged( double value );
    void viewRatchettingEnabledChanged( bool value );
    void viewRatchettingPercentChanged( double value );
};

// Would be nice to do <typename T, T min, T max> but the standard doesn't allow
// for floating point non-types.
template <typename T> inline T reduceAngle( T angle, T min, T max )
{
    while ( angle >= max )
    {
        angle -= ( max - min );
    }
    while ( angle < min )
    {
        angle += ( max - min );
    }
    return angle;
}

// Convienience function for incrementing/decrementing an index in a circular
// buffer.
inline size_t circularIndex( const size_t idx,
                             const bool increment,
                             const size_t modulus ) noexcept
{
    return increment ? ( ( idx + 1 ) % modulus )
                     : ( ( idx == 0 ) ? ( modulus - 1 ) : ( idx - 1 ) );
}

} // namespace advsettings
