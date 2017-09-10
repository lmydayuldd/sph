#ifndef STRINGS_H
#define STRINGS_H // TODO // also takes care of dirs so name is misleading

#include <QString>

class Strings
{
public:
    static const QString DESKTOP;
    static QString DIR_FRAMES;
    static QString IMG_FILE_TYPE;

    static void init();

    Strings();
};

#endif // STRINGS_H
