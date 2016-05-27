uniform highp mat4 uMatrix;
attribute highp vec4 aPos;
attribute lowp vec4 aCol;
varying lowp vec4 vCol;

void main() {
    gl_PointSize = 16.0;

    vCol = aCol;
    gl_Position = uMatrix * aPos;
}
