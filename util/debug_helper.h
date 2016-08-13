#ifndef DEBUG_HELPER_H
#define DEBUG_HELPER_H

#include <sstream>

#include <QMessageBox>

class DebugHelper
{
public:
    DebugHelper();

    static void showText(const char* text);
    template<typename T> static void showVariable(const char* title, T value);

    template<typename... Args>
    static void showVariables(const char* title, Args... args)
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle(QString(title));
        std::stringstream ss;
        for (unsigned int i = 0; i < sizeof...(args); ++i)
        {
#ifdef DESKTOP_BUILD
            ss << "%" << std::to_string(i+1).c_str();
#else
            ss << "%" << "";
#endif
        }
        QString message = QString(ss.str().c_str()).arg(args...);
        msgBox.setText(message);
        msgBox.exec();
    }
};

#endif // DEBUG_HELPER_H
