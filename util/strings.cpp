#include "util/strings.h"

#include <QDir>
#include <QStandardPaths>

const QString Strings::DESKTOP = QStandardPaths::locate(
                                    QStandardPaths::DesktopLocation, "",
                                    QStandardPaths::LocateDirectory);
QString Strings::DIR_FRAMES;

void Strings::init()
{
    QDir(Strings::DESKTOP + QString("/frames/")).removeRecursively();
    QDir().mkdir(Strings::DESKTOP + QString("/frames/"));
    DIR_FRAMES
        = QStandardPaths::locate(QStandardPaths::DesktopLocation, "frames/",
                                 QStandardPaths::LocateDirectory);
}

Strings::Strings()
{
}
