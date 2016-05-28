

std::vector<Flow> Flow::flows;

Flow::Flow()
{
}

Flow::Flow(int particlesCount)
{
    particles = std::vector<Particle>(particlesCount);
}
