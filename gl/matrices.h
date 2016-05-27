#ifndef MATRICES_H
#define MATRICES_H

#include <QMatrix4x4>

class Matrices {
public:
    static QMatrix4x4 modelMatrix;
    static QMatrix4x4 viewMatrix;
    static QMatrix4x4 modelViewMatrix;
    static QMatrix4x4 projectionMatrix;
    static QMatrix4x4 viewProjectionMatrix;
    static QMatrix4x4 viewProjectionInverted;
    static QMatrix4x4 mvpMatrix;
    static float defaultCam[9];
    static float cam[9];
    static float camRX;
    static float camRY;
    static float camTX;
    static float camTY;
    static float camTZ;

    static void setViewMatrix();
};

#endif // MATRICES_H
