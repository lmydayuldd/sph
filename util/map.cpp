#include <QFile>
#include <QTextStream>

#include "machine/particle.h"
#include "physics/grid.h"
#include "physics/vector.h"
#include "util/map.h"
#include "util/settings.h"

std::vector<std::vector<char>> Map::map;
unsigned Map::mapWidth = 0;
unsigned Map::mapHeight = 0;
unsigned Map::obstacleCount = 0;
unsigned Map::particleCount = 0;
unsigned Map::ghostCount = 0;

Map::Map()
{
}

void Map::reset()
{
    map.clear();
    mapWidth = 0;
    mapHeight = 0;
    obstacleCount = 0;
    particleCount = 0;
    ghostCount = 0;
}

void Map::import(unsigned char mapSetup)
{
    reset();

    QString fileName;
    if      (mapSetup == DAM_BREAK) fileName = ":/map/sim_dam_break.txt";
    else if (mapSetup == DROPLET)   fileName = ":/map/sim_droplet.txt";
    else if (mapSetup == VESSELS)   fileName = ":/map/sim_communicating_vessels.txt";

    QFile file(fileName);
    if (file.open(QIODevice::ReadOnly))
    {
        QTextStream in(&file);
        unsigned i = 0;
        while (! in.atEnd())
        {
            QString line = in.readLine();
            if ((unsigned) line.length() > mapWidth)
            {
                mapWidth = line.length();
            }
            ++i;
        }
        mapHeight = i;
        file.close();
    }

    map = std::vector<std::vector<char>>(mapHeight);
    for (unsigned i = 0; i < mapHeight; ++i)
        map.push_back(std::vector<char>(mapWidth));

    if (file.open(QIODevice::ReadOnly))
    {
        QTextStream in(&file);
        char c;
        unsigned i = 0;
        while (! in.atEnd())
        {
            QString line = in.readLine();
            for (unsigned j = 0; j < (unsigned) line.length(); ++j)
            {
                c = line.at(j).toLatin1();
                if      (c == 'O') ++particleCount;
                else if (c == '|') ++obstacleCount;
                else if (c == 'G') ++ghostCount;
                map[i].push_back(c);
            }
            ++i;
        }
        file.close();
    }
}

void Map::generate()
{
    if      (Settings::MAP_SETUP == RANDOM_NON_MAP
          || Settings::MAP_SETUP == DAM_BREAK_NON_MAP)
    {
        Particle::flows.push_back(std::vector<Particle*>(Settings::PARTICLE_COUNT));
        for (unsigned i = 0; i < Particle::flows[0].size(); ++i)
        {
            Particle::flows[0][i] = new Particle(Particle::flows.size() - 1);
            //if (i != 100) Particle::flows[0][i]->stationary = true;
        }
    }
    else if (Settings::MAP_SETUP == DAM_BREAK_3D_NON_MAP)
    {
        Settings::PARTICLE_COUNT *= 10;

        Particle::flows.push_back(std::vector<Particle*>(Settings::PARTICLE_COUNT));
        for (unsigned i = 0; i < Particle::flows[0].size(); ++i)
        {
            Particle::flows[0][i] = new Particle(Particle::flows.size() - 1);
        }

        Particle* p;
        for (unsigned i = 0; i < Particle::flows[0].size(); ++i)
        {
            p = Particle::flows[0][i];
            p->r = new Vector(
                (-Settings::ARENA_DIAMETER/2 + p->radius) + fmod(p->id-1     , 20) * Settings::PARTICLES_INIT_DIST,//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5),
                (-Settings::ARENA_DIAMETER/2 + p->radius) + fmod((p->id-1)/20, 20) * Settings::PARTICLES_INIT_DIST,//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5),
                                               p->radius  +      (p->id-1)/400     * Settings::PARTICLES_INIT_DIST//(static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.0f) * (rand()%5)
            );
        }
        Settings::ARENA_DIAMETER_Z
            = Particle::flows[0][Particle::flows[0].size() - 1]->r->z
              + Settings::PARTICLE_RADIUS;
    }
    else
    {
        import(Settings::MAP_SETUP);
        Settings::PARTICLE_COUNT = particleCount;

        Particle::flows.push_back(std::vector<Particle*>(particleCount));
        for (unsigned i = 0; i < Particle::flows[0].size(); ++i)
        {
            Particle::flows[0][i] = new Particle(Particle::flows.size() - 1);
        }

        unsigned k = 0;
        for (unsigned i = 0; i < map.size(); ++i)
        {
            for (unsigned j = 0; j < map[i].size(); ++j)
            {
                if (map[i][j] == 'O')
                {
                    *Particle::flows[0][k++]->r
                        = Vector(
                            -Settings::ARENA_DIAMETER/2
                                + Settings::PARTICLE_RADIUS
                                + j * Settings::PARTICLES_INIT_DIST,
                            Settings::ARENA_DIAMETER/2
                                - Settings::PARTICLE_RADIUS
                                - i * Settings::PARTICLES_INIT_DIST,
                            0
                          );
                }
            }
        }
    }
}
