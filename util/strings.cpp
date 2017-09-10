#include "util/strings.h"

#include "util/settings.h"

#include <QDir>
#include <QStandardPaths>

const QString Strings::DESKTOP = QStandardPaths::locate(
                                    QStandardPaths::DesktopLocation, "",
                                    QStandardPaths::LocateDirectory);
QString Strings::DIR_FRAMES;
QString Strings::IMG_FILE_TYPE;

void Strings::init()
{
    if (! Settings::NO_SCREENS_NO_VIDEO)
    {
        QDir(Strings::DESKTOP + QString("/frames/")).removeRecursively();
        QDir().mkdir(Strings::DESKTOP + QString("/frames/"));
        DIR_FRAMES
            = QStandardPaths::locate(QStandardPaths::DesktopLocation, "frames/",
                                     QStandardPaths::LocateDirectory);
    }
    IMG_FILE_TYPE = QString::fromStdString(Settings::IMG_FILE_TYPE);
}

Strings::Strings()
{
}
