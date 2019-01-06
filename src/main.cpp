#include "overlaycontroller.h"
#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQuickView>
#include <QQmlEngine>
#include <QQmlComponent>
#include <QSettings>
#include <QStandardPaths>
#include <openvr.h>
#include <iostream>
#include <easylogging++.h>

INITIALIZE_EASYLOGGINGPP

void myQtMessageHandler( QtMsgType type,
                         const QMessageLogContext& context,
                         const QString& msg )
{
    QByteArray localMsg = msg.toLocal8Bit();
    switch ( type )
    {
    case QtDebugMsg:
        LOG( DEBUG ) << localMsg.constData() << " (" << context.file << ":"
                     << context.line << ")";
        break;
    case QtInfoMsg:
        LOG( INFO ) << localMsg.constData() << " (" << context.file << ":"
                    << context.line << ")";
        break;
    case QtWarningMsg:
        LOG( WARNING ) << localMsg.constData() << " (" << context.file << ":"
                       << context.line << ")";
        break;
    case QtCriticalMsg:
        LOG( ERROR ) << localMsg.constData() << " (" << context.file << ":"
                     << context.line << ")";
        break;
    case QtFatalMsg:
        LOG( FATAL ) << localMsg.constData() << " (" << context.file << ":"
                     << context.line << ")";
        break;
    }
}

void installManifest( bool cleaninstall = false )
{
    auto manifestQPath
        = QDir::cleanPath( QDir( QCoreApplication::applicationDirPath() )
                               .absoluteFilePath( "manifest.vrmanifest" ) );
    if ( !QFile::exists( manifestQPath ) )
    {
        throw std::runtime_error(
            std::string( "Could not find application manifest: " )
            + manifestQPath.toStdString() );
    }

    bool alreadyInstalled = false;
    if ( vr::VRApplications()->IsApplicationInstalled(
             advsettings::OverlayController::applicationKey ) )
    {
        if ( cleaninstall )
        {
            char buffer[1024];
            auto appError = vr::VRApplicationError_None;
            vr::VRApplications()->GetApplicationPropertyString(
                advsettings::OverlayController::applicationKey,
                vr::VRApplicationProperty_WorkingDirectory_String,
                buffer,
                1024,
                &appError );
            if ( appError == vr::VRApplicationError_None )
            {
                auto oldManifestQPath = QDir::cleanPath(
                    QDir( buffer ).absoluteFilePath( "manifest.vrmanifest" ) );
                if ( oldManifestQPath.compare( manifestQPath,
                                               Qt::CaseInsensitive )
                     != 0 )
                {
                    vr::VRApplications()->RemoveApplicationManifest(
                        QDir::toNativeSeparators( oldManifestQPath )
                            .toStdString()
                            .c_str() );
                }
                else
                {
                    alreadyInstalled = true;
                }
            }
        }
        else
        {
            alreadyInstalled = true;
        }
    }
    auto apperror = vr::VRApplications()->AddApplicationManifest(
        QDir::toNativeSeparators( manifestQPath ).toStdString().c_str() );
    if ( apperror != vr::VRApplicationError_None )
    {
        throw std::runtime_error(
            std::string( "Could not add application manifest: " )
            + std::string(
                  vr::VRApplications()->GetApplicationsErrorNameFromEnum(
                      apperror ) ) );
    }
    else if ( !alreadyInstalled || cleaninstall )
    {
        auto app_error = vr::VRApplications()->SetApplicationAutoLaunch(
            advsettings::OverlayController::applicationKey, true );
        if ( app_error != vr::VRApplicationError_None )
        {
            throw std::runtime_error(
                std::string( "Could not set auto start: " )
                + std::string(
                      vr::VRApplications()->GetApplicationsErrorNameFromEnum(
                          apperror ) ) );
        }
    }
}

void removeManifest()
{
    auto manifestQPath
        = QDir::cleanPath( QDir( QCoreApplication::applicationDirPath() )
                               .absoluteFilePath( "manifest.vrmanifest" ) );
    if ( QFile::exists( manifestQPath ) )
    {
        if ( vr::VRApplications()->IsApplicationInstalled(
                 advsettings::OverlayController::applicationKey ) )
        {
            vr::VRApplications()->RemoveApplicationManifest(
                QDir::toNativeSeparators( manifestQPath )
                    .toStdString()
                    .c_str() );
        }
    }
    else
    {
        throw std::runtime_error(
            std::string( "Could not find application manifest: " )
            + manifestQPath.toStdString() );
    }
}

// Sets up the logging library and outputs startup logging data.
// argc and argv are necessary for the START_EASYLOGGINGPP() call.
void setUpLogging( int argc, char* argv[] )
{
    constexpr auto logConfigDefault
        = "* GLOBAL:\n"
          "	FORMAT = \"[%level] %datetime{%Y-%M-%d %H:%m:%s}: %msg\"\n"
          "	FILENAME = \"AdvancedSettings.log\"\n"
          "	ENABLED = true\n"
          "	TO_FILE = true\n"
          "	TO_STANDARD_OUTPUT = true\n"
          "	MAX_LOG_FILE_SIZE = 2097152 ## 2MB\n"
          "* TRACE:\n"
          "	ENABLED = false\n"
          "* DEBUG:\n"
          "	ENABLED = false\n";

    // This allows easylogging++ to parse command line arguments.
    // For a full list go here
    // https://github.com/zuhd-org/easyloggingpp#application-arguments
    // The parsed args are mostly setting the verbosity level or setting the
    // conf file.
    START_EASYLOGGINGPP( argc, argv );

    el::Loggers::addFlag( el::LoggingFlag::DisableApplicationAbortOnFatalLog );

    // If there is a default logging config file present, use that. If not, use
    // the default settings.
    constexpr auto logConfigFileName = "logging.conf";
    const auto logconfigfile
        = QFileInfo( logConfigFileName ).absoluteFilePath();
    el::Configurations conf;
    if ( QFile::exists( logconfigfile ) )
    {
        conf.parseFromFile( logconfigfile.toStdString() );
    }
    else
    {
        conf.parseFromText( logConfigDefault );
    }

    // This places the log file in
    // Roaming/AppData/matzman666/OpenVRAdvancedSettings/AdvancedSettings.log.
    // The log file placement has been broken since at least git tag "v2.7".
    // It was being placed in the working dir of the executable.
    // The change hasn't been documented anywhere, so it is likely that it was
    // unintentional. This fixes the probable regression until a new path is
    // decided on.
    constexpr auto appDataFolders = "/matzman666/OpenVRAdvancedSettings";
    const QString logFilePath = QDir( QStandardPaths::writableLocation(
                                          QStandardPaths::AppDataLocation )
                                      + appDataFolders )
                                    .absoluteFilePath( "AdvancedSettings.log" );
    conf.set( el::Level::Global,
              el::ConfigurationType::Filename,
              QDir::toNativeSeparators( logFilePath ).toStdString() );

    conf.setRemainingToDefault();

    el::Loggers::reconfigureAllLoggers( conf );

    LOG( INFO ) << "Application started (Version "
                << advsettings::OverlayController::applicationVersionString
                << ")";
    LOG( INFO ) << "Log Config: "
                << QDir::toNativeSeparators( logconfigfile ).toStdString();
    LOG( INFO ) << "Log File: " << logFilePath;
}

class MyQApplication : public QApplication
{
public:
    using QApplication::QApplication;

    virtual bool notify( QObject* receiver, QEvent* event ) override
    {
        try
        {
            return QApplication::notify( receiver, event );
        }
        catch ( std::exception& e )
        {
            LOG( ERROR ) << "Exception thrown from an event handler: "
                         << e.what();
        }
        return false;
    }
};

namespace argument
{
// Checks whether a specific string was passed as a launch argument.
bool CheckCommandLineArgument( int argc,
                               char* argv[],
                               const std::string parameter )
{
    // The old way was one giant loop that tested every single parameter.
    // Having an individual loop for all args most likely has a very small
    // performance penalty compared to the original solution, but the gains in
    // readability and ease of extension of not having a hundred line+ for loop
    // are worth it.
    for ( int i = 0; i < argc; i++ )
    {
        if ( std::string( argv[i] ) == parameter )
        {
            return true;
        }
    }
    return false;
}

constexpr auto DESKTOP_MODE = "-desktop";
constexpr auto NO_SOUND = "-nosound";
constexpr auto NO_MANIFEST = "-nomanifest";
constexpr auto INSTALL_MANIFEST = "-installmanifest";
constexpr auto REMOVE_MANIFEST = "-removemanifest";
} // namespace argument

enum ReturnErrorCode
{
    SUCCESS = 0,
    GENERAL_FAILURE = -1,
    OPENVR_INIT_ERROR = -2,
};

int main( int argc, char* argv[] )
{
    setUpLogging( argc, argv );

    const bool desktop_mode = argument::CheckCommandLineArgument(
        argc, argv, argument::DESKTOP_MODE );
    const bool no_sound
        = argument::CheckCommandLineArgument( argc, argv, argument::NO_SOUND );
    const bool no_manifest = argument::CheckCommandLineArgument(
        argc, argv, argument::NO_MANIFEST );
    const bool install_manifest = argument::CheckCommandLineArgument(
        argc, argv, argument::INSTALL_MANIFEST );
    const bool remove_manifest = argument::CheckCommandLineArgument(
        argc, argv, argument::REMOVE_MANIFEST );

    if ( install_manifest || remove_manifest )
    {
        int exit_code = ReturnErrorCode::SUCCESS;
        auto openvr_init_status = vr::VRInitError_None;
        vr::VR_Init( &openvr_init_status, vr::VRApplication_Utility );
        if ( openvr_init_status == vr::VRInitError_None )
        {
            try
            {
                if ( install_manifest )
                {
                    LOG( INFO ) << "Install manifest enabled.";
                    installManifest( true );
                }
                else if ( remove_manifest )
                {
                    LOG( INFO ) << "Remove manifest enabled.";
                    removeManifest();
                }
            }
            catch ( std::exception& e )
            {
                exit_code = ReturnErrorCode::GENERAL_FAILURE;
                std::cerr << e.what() << std::endl;
            }
        }
        else
        {
            exit_code = ReturnErrorCode::OPENVR_INIT_ERROR;
            std::cerr << std::string(
                             "Failed to initialize OpenVR: "
                             + std::string(
                                   vr::VR_GetVRInitErrorAsEnglishDescription(
                                       openvr_init_status ) ) )
                      << std::endl;
        }

        vr::VR_Shutdown();
        exit( exit_code );
    }

    LOG_IF( desktop_mode, INFO ) << "Desktop mode enabled.";
    LOG_IF( no_sound, INFO ) << "Sound effects disabled.";
    LOG_IF( no_manifest, INFO ) << "vrmanifest disabled.";

    try
    {
        MyQApplication a( argc, argv );
        a.setOrganizationName( "matzman666" );
        a.setApplicationName( "OpenVRAdvancedSettings" );
        a.setApplicationDisplayName(
            advsettings::OverlayController::applicationName );
        a.setApplicationVersion(
            advsettings::OverlayController::applicationVersionString );

        qInstallMessageHandler( myQtMessageHandler );

        QSettings appSettings( QSettings::IniFormat,
                               QSettings::UserScope,
                               a.organizationName(),
                               a.applicationName() );
        advsettings::OverlayController::setAppSettings( &appSettings );
        LOG( INFO ) << "Settings File: "
                    << appSettings.fileName().toStdString();

        QQmlEngine qmlEngine;

        advsettings::OverlayController* controller
            = advsettings::OverlayController::createInstance( desktop_mode,
                                                              no_sound );
        controller->Init( &qmlEngine );

        QQmlComponent component(
            &qmlEngine, QUrl::fromLocalFile( "res/qml/mainwidget.qml" ) );
        auto errors = component.errors();
        for ( auto& e : errors )
        {
            LOG( ERROR ) << "QML Error: " << e.toString().toStdString()
                         << std::endl;
        }
        auto quickObj = component.create();
        controller->SetWidget( qobject_cast<QQuickItem*>( quickObj ),
                               advsettings::OverlayController::applicationName,
                               advsettings::OverlayController::applicationKey );

        if ( !desktop_mode && !no_manifest )
        {
            try
            {
                installManifest();
            }
            catch ( std::exception& e )
            {
                LOG( ERROR ) << e.what();
            }
        }

        if ( desktop_mode )
        {
            auto m_pWindow = new QQuickWindow();
            qobject_cast<QQuickItem*>( quickObj )
                ->setParentItem( m_pWindow->contentItem() );
            m_pWindow->setGeometry(
                0,
                0,
                qobject_cast<QQuickItem*>( quickObj )->width(),
                qobject_cast<QQuickItem*>( quickObj )->height() );
            m_pWindow->show();
        }

        return a.exec();
    }
    catch ( const std::exception& e )
    {
        LOG( FATAL ) << e.what();
        return ReturnErrorCode::GENERAL_FAILURE;
    }
}
