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
    switch (mapSetup) {
        case DAM_BREAK : fileName = ":/map/sim_dam_break.txt";             break;
        case DROPLET   : fileName = ":/map/sim_droplet.txt";               break;
        case VESSELS   : fileName = ":/map/sim_communicating_vessels.txt"; break;
    }

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
    {
        map.push_back(std::vector<char>(mapWidth));
    }

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

    if (Settings::GHOST_LAYER_GAGE > 0)
    {
//#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned i = 0; i < Particle::flows[0].size(); ++i)
        {
            Particle::flows[0][i]->r->x += Settings::GHOST_LAYER_GAGE
                                         * Settings::PARTICLES_INIT_DIST;
            Particle::flows[0][i]->r->y += Settings::GHOST_LAYER_GAGE
                                         * Settings::PARTICLES_INIT_DIST;
        }
        Particle *ghostParticle;
        for (unsigned i = 0; i < Grid::cell_count; ++i)
        {
//#pragma omp parallel for if(Settings::PARALLEL_OMP)
            for (unsigned j = 0; j < Grid::cell_count; ++j)
            {
                if (i < Settings::GHOST_LAYER_GAGE
                 || j < Settings::GHOST_LAYER_GAGE
                 || i >= Grid::cell_count - Settings::GHOST_LAYER_GAGE
                 || j >= Grid::cell_count - Settings::GHOST_LAYER_GAGE)
                {
                    ghostParticle = new Particle(0);
                    ghostParticle->stationary = true;
                    ghostParticle->r->x = - Settings::ARENA_DIAMETER/2
                                          + Settings::PARTICLE_RADIUS
                                          + i * Settings::PARTICLES_INIT_DIST;
                    ghostParticle->r->y =   Settings::ARENA_DIAMETER/2
                                          - Settings::PARTICLE_RADIUS
                                          - j * Settings::PARTICLES_INIT_DIST;
//#pragma omp atomic // critical
                    Particle::flows[0].push_back(ghostParticle);
                    Settings::PARTICLE_COUNT += 1;
                }
            }
        }
    }
}
