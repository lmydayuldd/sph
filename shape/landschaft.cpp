#include "landschaft.h"

#include "gl/form.h"
#include "gl/matrices.h"
#include "physics/computer.h"

using namespace std;

Landschaft::Landschaft(vector<vector<float>> heightMap) {
    this->heightMap = heightMap;
    width  = heightMap.size() - 1;
    height = heightMap[0].size() - 1;

    sculpt(heightMap);
}

void Landschaft::sculpt(vector<vector<float>> heightMap) {
    //float w = 4.0f;
    //float d = 2 * w / width;
    if (form == nullptr) {
        int size = 0;
        for (int i = 1; i < width; ++i) {
            size += 2 * height * (3 + 3); // 2-wide height-lenghted string of 3-coord vertices, with 3-coord colors
        }
        vertices = vector<float>(size);
        form = new Form(
            vertices, 3, GL_TRIANGLE_STRIP,
            vector<float>(), 3,
            vector<float>(), 0, 0,
            LANDSCHAFT
        );
    }

    if (! (heightMap.size()-1 == (unsigned) this->width && heightMap[0].size()-1 == (unsigned) this->height)) {
        vertices.clear();
        sculpt(heightMap);
        return; //////////////////////////////////////////////////////////////////////////////
    }

    this->heightMap = heightMap;

//    float obstacle[6] = { 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f };
//    for (int i = 1; i < width; ++i) {
//        for (int j = 1, k = 0; j < height; j += 2) {
//            if (! Computer::currentComputer->flagIs(i-1, j-1, Wave2DComputer.CELL_BND)) {
//                forms[i-1].posCoords[k++] = -w + (i-1) * d;
//                forms[i-1].posCoords[k++] =  w - (j-1) * d;
//                forms[i-1].posCoords[k++] = heightMap[j-1][i-1];
//                forms[i-1].posCoords[k++] = (heightMap[j-1][i-1] + 1) / 2;
//                forms[i-1].posCoords[k++] = (heightMap[j-1][i-1] + 1) / 2;
//                forms[i-1].posCoords[k++] = (heightMap[j-1][i-1] + 1) / 2;
//            } else {
//                forms[i-1].posCoords[k++] = -w + (i-1) * d;
//                forms[i-1].posCoords[k++] =  w - (j-1) * d;
//                //System.arraycopy(obstacle, 2, forms[i-1].posCoords, k, 4);
//                copy(&obstacle[2], &obstacle[6], forms[i-1].posCoords.begin() + k);
//                k += 4;
//            }

//            if (! Computer::currentComputer->flagIs(i, j-1, Wave2DComputer.CELL_BND)) {
//                forms[i-1].posCoords[k++] = -w + i * d;
//                forms[i-1].posCoords[k++] =  w - (j-1) * d;
//                forms[i-1].posCoords[k++] = heightMap[j-1][i];
//                forms[i-1].posCoords[k++] = (heightMap[j-1][i] + 1) / 2;
//                forms[i-1].posCoords[k++] = (heightMap[j-1][i] + 1) / 2;
//                forms[i-1].posCoords[k++] = (heightMap[j-1][i] + 1) / 2;
//            } else {
//                forms[i-1].posCoords[k++] = -w + i * d;
//                forms[i-1].posCoords[k++] =  w - (j-1) * d;
//                //System.arraycopy(obstacle, 2, forms[i-1].posCoords, k, 4);
//                copy(&obstacle[2], &obstacle[6], forms[i-1].posCoords.begin() + k);
//                k += 4;
//            }

//            if (! Computer::currentComputer->flagIs(i-1, j, Wave2DComputer.CELL_BND)) {
//                forms[i-1].posCoords[k++] = -w + (i-1) * d;
//                forms[i-1].posCoords[k++] =  w - j * d;
//                forms[i-1].posCoords[k++] = heightMap[j][i-1];
//                forms[i-1].posCoords[k++] = (heightMap[j][i-1] + 1) / 2;
//                forms[i-1].posCoords[k++] = (heightMap[j][i-1] + 1) / 2;
//                forms[i-1].posCoords[k++] = (heightMap[j][i-1] + 1) / 2;
//            } else {
//                forms[i-1].posCoords[k++] = -w + (i-1) * d;
//                forms[i-1].posCoords[k++] =  w - j * d;
//                //System.arraycopy(obstacle, 2, forms[i-1].posCoords, k, 4);
//                copy(&obstacle[2], &obstacle[6], forms[i-1].posCoords.begin() + k);
//                k += 4;
//            }

//            if (! Computer::currentComputer->flagIs(i, j, Wave2DComputer.CELL_BND)) {
//                forms[i-1].posCoords[k++] = -w + i * d;
//                forms[i-1].posCoords[k++] =  w - j * d;
//                forms[i-1].posCoords[k++] = heightMap[j][i];
//                forms[i-1].posCoords[k++] = (heightMap[j][i] + 1) / 2;
//                forms[i-1].posCoords[k++] = (heightMap[j][i] + 1) / 2;
//                forms[i-1].posCoords[k++] = (heightMap[j][i] + 1) / 2;
//            } else {
//                forms[i-1].posCoords[k++] = -w + i * d;
//                forms[i-1].posCoords[k++] =  w - j * d;
//                //System.arraycopy(obstacle, 2, forms[i-1].posCoords, k, 4);
//                copy(&obstacle[2], &obstacle[6], forms[i-1].posCoords.begin() + k);
//                k += 4;
//            }
//        }
//        forms[i-1].move();
//    }
}
