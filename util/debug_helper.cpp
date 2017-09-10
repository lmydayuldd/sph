#include "debug_helper.h"

DebugHelper::DebugHelper()
{
}

void DebugHelper::showText(const char *text)
{
    QMessageBox msgBox;
    msgBox.setWindowTitle(QString("DebugHelper"));
    msgBox.setText(QString("DebugHelper: ") + QString(text));
//    msgBox.show();
    msgBox.exec();
}

template void DebugHelper::showVariable<int>(const char*, int);
template void DebugHelper::showVariable<float>(const char*, float);
template void DebugHelper::showVariable<double>(const char*, double);

template<typename T>
void DebugHelper::showVariable(const char *title, T value)
{
    QMessageBox msgBox;
    msgBox.setWindowTitle(QString(title));
    msgBox.setText(QString("%1").arg(value));
    msgBox.exec();
}
