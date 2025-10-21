#include "../../include/graphs/spring.hpp"
#include "../../include/physics/physics.hpp"

graphs::Spring::Spring(graphs::SoftBody &body, graphs::Ball &ball1, graphs::Ball &ball2, float normalLength, float springCoefficient, sf::Color color)
    : body(body), 
    ball1(ball1),
      ball2(ball2),
      color(color),
      normalLength(normalLength),
      springCoefficient(springCoefficient),
      currentLength(0.f),
      springForce(0.f, 0.f)
{
    bool exists = false;

    // prevents to create same spring twice or more
    for (graphs::Spring &spring : body.edgeSprings)
    {
        if ((&this->ball1 == &spring.ball1 && &this->ball2 == &spring.ball2) ||
            (&this->ball1 == &spring.ball2 && &this->ball2 == &spring.ball1))
        {
            exists = true;
            break;
        }
    }
    if (exists)
    {
        return;
    }
}

graphs::Spring::Spring(graphs::Ball &ball1, graphs::Ball &ball2, float normalLength, float springCoefficient, sf::Color color)
    : body(std::nullopt), 
    ball1(ball1),
      ball2(ball2),
      color(color),
      normalLength(normalLength),
      springCoefficient(springCoefficient),
      currentLength(0.f),
      springForce(0.f, 0.f)
{
    bool exists = false;

    // prevents to create same spring twice or more
    for (graphs::Spring &spring : graphs::Springs)
    {
        if ((&this->ball1 == &spring.ball1 && &this->ball2 == &spring.ball2) ||
            (&this->ball1 == &spring.ball2 && &this->ball2 == &spring.ball1))
        {
            exists = true;
            break;
        }
    }
    if (exists)
    {
        return;
    }
}

void graphs::Spring::draw(sf::RenderWindow &window)
{
    std::array<sf::Vertex, 2> line =
        {
            sf::Vertex{sf::Vector2f(this->ball1.pos.x, this->ball1.pos.y), this->color},
            sf::Vertex{sf::Vector2f(this->ball2.pos.x, this->ball2.pos.y), this->color}};
    
    window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
}

void graphs::Spring::computeSpringForce()
{
    this->springForce = linalg::Vector(0.f, 0.f);

    linalg::Vector axis(this->ball2.pos - this->ball1.pos);
    linalg::Vector normalVector(axis.unit());
    this->currentLength = axis.magnitude();

    float springForceMagnitude = -this->springCoefficient * (this->normalLength - this->currentLength) * physics::PIXEL_PER_METER;
    this->springForce = normalVector * springForceMagnitude;

    if (!this->ball1.isBeingDragged)
    {
        this->ball1.springForce = this->ball1.springForce + this->springForce;
    }
    if (!this->ball2.isBeingDragged)
    {
        this->ball2.springForce = this->ball2.springForce - this->springForce;
    }
}