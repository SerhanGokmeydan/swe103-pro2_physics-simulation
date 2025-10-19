#include "../../include/graphs/soft-body.hpp"
#include "../../include/physics/physics.hpp"
#include <cmath>

graphs::SoftBody::SoftBody(linalg::Vector center, sf::Color color, int pointCount, float radius, float mass, float elasticity, float springStiffness, float pressureStiffness)
    : center(center),
      prevPos(0.f, 0.f),
      color(color),
      pointCount(pointCount),
      radius(radius),
      mass(mass),
      elasticity(elasticity),
      restArea(0.f),
      springStiffness(springStiffness),
      pressureStiffness(pressureStiffness),
      isBeingDragged(false)
{
    body.setPointCount(this->pointCount);

    float arcAngle = 360 / pointCount;
    for (int i = 0; i < pointCount; i++)
    {
        float angle = -i * arcAngle * (physics::pi / 180);
        linalg::Vector point = this->center + linalg::Vector(this->radius * std::cos(angle), this->radius * std::sin(angle));

        body.setPoint(i, {point.x, point.y});
        this->cornerBalls.emplace_back(point, this->color, 3.f, this->mass / this->pointCount, this->elasticity);

        if (this->cornerBalls.size() < this->pointCount)
        {
            continue;
        }
        else
        {
            for (int i = 0; i < this->pointCount; i++)
            {
                for (int j = i + 1; j < this->pointCount; j++)
                {
                    graphs::Ball &ball1 = this->cornerBalls.at(i);
                    graphs::Ball &ball2 = this->cornerBalls.at(j);

                    float normalLength = (ball1.pos - ball2.pos).magnitude();
                    this->edgeSprings.emplace_back(*this, ball1, ball2, normalLength, this->springStiffness, this->color);
                }
            }
        }
    }

    this->restArea = this->getCurrentArea();
}

void graphs::SoftBody::update()
{
    if (!this->isBeingDragged)
    {
        // computes the center of the body
        linalg::Vector sum(0.f, 0.f);
        for (graphs::Ball &ball : this->cornerBalls)
        {
            sum = sum + ball.pos;
        }
        this->center = sum / this->pointCount;
    }

    for (int i = 0; i < this->pointCount; i++)
    {
        body.setPoint(i, {this->cornerBalls.at(i).pos.x, this->cornerBalls.at(i).pos.y});
    }
}

void graphs::SoftBody::draw(sf::RenderWindow &window){
    window.draw(this->body);
}

float graphs::SoftBody::getCurrentArea() const
{
    float area = 0.0f;
    int j = this->pointCount - 1; // last corner

    for (int i = 0; i < this->pointCount; i++)
    {
        const linalg::Vector &ball_i = this->cornerBalls[i].pos;
        const linalg::Vector &ball_j = this->cornerBalls[j].pos;

        area += (ball_j.x + ball_i.x) * (ball_j.y - ball_i.y);
        j = i;
    }

    return std::abs(area / 2.0f);
}

void graphs::SoftBody::computePressureForce()
{
    float currentArea = this->getCurrentArea();
    if (currentArea == 0.f)
    {
        return;
    }

    float pressure = this->pressureStiffness * (this->restArea - currentArea);

    for (int i = 0; i < this->pointCount; i++)
    {
        graphs::Ball &ball1 = this->cornerBalls[i];
        graphs::Ball &ball2 = this->cornerBalls[(i + 1) % this->pointCount]; // Bir sonraki top, sonuncuda başa döner

        linalg::Vector edge = ball2.pos - ball1.pos;
        float length = edge.magnitude();
        if (length == 0.f)
            continue;
        linalg::Vector normal = linalg::Vector(-edge.y, edge.x).unit();

        linalg::Vector force = normal * pressure * length * physics::PIXEL_PER_METER;

        ball1.pressureForce = ball1.pressureForce + (force / 2.0f);
        ball2.pressureForce = ball2.pressureForce + (force / 2.0f);
    }
}

void graphs::SoftBody::projectileMotion(linalg::Vector &mouseVector, float deltaTime)
{
    float cursorDistance = (this->center - mouseVector).magnitude();

    if (cursorDistance < this->radius && !this->isBeingDragged)
    {
        this->isBeingDragged = true;
        for (graphs::Ball &ball : this->cornerBalls)
        {
            ball.isBeingDragged = true;
            ball.vel = linalg::Vector(0.f, 0.f);
        }
        this->prevPos = mouseVector;
    }

    if (this->isBeingDragged)
    {
        // computes the displacement of position of mouse
        linalg::Vector displacement(mouseVector - this->prevPos);

        // computes the instantaneous velocity
        linalg::Vector instantVel(displacement / deltaTime);

        for (graphs::Ball &ball : this->cornerBalls)
        {
            ball.vel = instantVel;
            ball.pos = ball.pos + displacement;
        }

        // while dragging, follow the mouse
        this->center = mouseVector;

        this->prevPos = mouseVector;
    }
}