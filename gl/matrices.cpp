#include <gl/matrices.h>

QMatrix4x4 Matrices::modelMatrix            = QMatrix4x4();
QMatrix4x4 Matrices::viewMatrix             = QMatrix4x4();
QMatrix4x4 Matrices::modelViewMatrix        = QMatrix4x4();
QMatrix4x4 Matrices::projectionMatrix       = QMatrix4x4();
QMatrix4x4 Matrices::viewProjectionMatrix   = QMatrix4x4();
QMatrix4x4 Matrices::viewProjectionInverted = QMatrix4x4();
QMatrix4x4 Matrices::mvpMatrix              = QMatrix4x4();
float Matrices::defaultCam[] = {0, 0, 5, 0, 0, 0, 0, 1, 0};
float Matrices::cam[]        = {0, 0, 5, 0, 0, 0, 0, 1, 0};
float Matrices::camRX = 0.f;
float Matrices::camRY = 0.f;
float Matrices::camTX = 0.f;
float Matrices::camTY = 0.f;
float Matrices::camTZ = 0.f;

void Matrices::setViewMatrix() {
//    viewMatrix.lookAt(
//        QVector3D(cam[0], cam[1], cam[2]),
//        QVector3D(cam[3], cam[4], cam[5]),
//        QVector3D(cam[6], cam[7], cam[8])
//    );
    viewMatrix.rotate(camRX, QVector3D(1, 0, 0));
    viewMatrix.rotate(camRY, QVector3D(0, 1, 0));
    viewMatrix.translate(QVector3D(-camTX, -camTY, -camTZ));
}
