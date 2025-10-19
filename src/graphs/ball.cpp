#include "../../include/graphs/ball.hpp"
#include "../../include/graphs/spring.hpp"
#include "../../include/physics/physics.hpp"
#include <cmath>
#include <iostream>

graphs::Ball::Ball(linalg::Vector pos, sf::Color color, float radius, float mass, float elasticity)
    : color(color),
      radius(radius),
      mass(mass),
      elasticity(elasticity),
      prevPos(0.f, 0.f),
      pos(pos),
      vel(0.f, 0.f),
      acc(0.f, 0.f),
      force(0.f, 0.f),
      gravity(0.f, 0.f), // set below after mass is known
      frictionForce(0.f, 0.f),
      dragForce(0.f, 0.f),
      springForce(0.f, 0.f),
      pressureForce(0.f, 0.f),
      isBeingDragged(false),
      isTouchWall(false)
{
    // compute gravity now that mass is set
    this->gravity = linalg::Vector(0.f, physics::g * this->mass * physics::PIXEL_PER_METER);

    shape.setFillColor(this->color);
    shape.setRadius(this->radius);
    shape.setPointCount(100);
    shape.setOrigin({this->radius, this->radius});
    shape.setPosition({this->pos.x, this->pos.y});
}

void graphs::Ball::update(float deltaTime)
{
    if (this->isBeingDragged)
    {
        return;
    }

    this->force = this->gravity + this->dragForce + this->frictionForce + this->springForce + this->pressureForce;
    this->acc = this->force / this->mass;
    this->vel = this->vel + this->acc * deltaTime;
    this->pos = this->pos + this->vel * deltaTime;
}

void graphs::Ball::draw(sf::RenderWindow &window)
{
    this->shape.setPosition({this->pos.x, this->pos.y});
    window.draw(this->shape);
}

void graphs::Ball::computeDragForce()
{
    // computes the drag force
    float speedInMeter = this->vel.magnitude() / physics::PIXEL_PER_METER;
    float radiusInMeter = this->radius / physics::PIXEL_PER_METER;
    float areaInMeter = physics::pi * std::pow(radiusInMeter, 2);
    float dragForceMagnitude = 0.5f * physics::dragCoefficient * physics::airDensity * areaInMeter * std::pow(speedInMeter, 2) * physics::PIXEL_PER_METER;

    this->dragForce = this->vel.unit() * -dragForceMagnitude;
}

void graphs::Ball::computeFrictionForce()
{
    float frictionForceMagnitude = physics::frictionCoefficient * this->gravity.magnitude();

    if (this->isTouchWall)
    {
        this->frictionForce = this->vel.unit() * -frictionForceMagnitude;
    }
    else
    {
        this->frictionForce = linalg::Vector({0.f, 0.f});
    }
}

void graphs::Ball::projectileMotion(linalg::Vector &mouseVector, float deltaTime)
{
    float cursorDistance = (this->pos - mouseVector).magnitude();

    if (cursorDistance < this->radius && !this->isBeingDragged)
    {
        this->isBeingDragged = true;
        this->vel = linalg::Vector(0.f, 0.f);
        this->prevPos = mouseVector;
    }

    if (this->isBeingDragged)
    {
        // computes the displacement of position of mouse
        linalg::Vector displacement(mouseVector - this->prevPos);

        // computes the instantaneous velocity
        linalg::Vector instantVel(displacement / deltaTime);
        this->vel = instantVel;

        // while dragging, follow the mouse
        this->pos = mouseVector;

        this->prevPos = mouseVector;
    }
}

void graphs::Ball::checkBallCollision(graphs::Ball &ball)
{
    linalg::Vector axis(this->pos - ball.pos);
    float distance = axis.magnitude();
    float overlap = this->radius + ball.radius - distance;

    if (overlap > 0)
    {
        linalg::Vector normalVector(axis.unit());
        linalg::Vector relativeVel(this->vel - ball.vel);
        const float elasticityCoefficient = (this->elasticity + ball.elasticity) / 2;
        const float totalMass = this->mass + ball.mass;

        this->pos = this->pos + normalVector * (overlap * (ball.mass / totalMass));
        ball.pos = ball.pos - normalVector * (overlap * (this->mass / totalMass));

        // computes the project of velocity along normal
        float velAlongNormal = relativeVel.dot(normalVector);

        // computes the magnitude of impulse
        float impulseMagnitude = (-(1 + elasticityCoefficient) * velAlongNormal) / (1 / this->mass + 1 / ball.mass);
        linalg::Vector impulse(normalVector * impulseMagnitude);

        // applies the impulse to be inversely proportional to mass
        this->vel = this->vel + impulse / this->mass;
        ball.vel = ball.vel - impulse / ball.mass;
    }
}

void graphs::Ball::checkSpringCollision(graphs::Spring &spring)
{
    if (&*this == &spring.ball1 || &*this == &spring.ball2)
    {
        return;
    }

    linalg::Vector springVector = spring.ball2.pos - spring.ball1.pos;
    linalg::Vector ballToStart = this->pos - spring.ball1.pos;
    linalg::Vector ballToEnd = this->pos - spring.ball2.pos;

    float springLength = springVector.magnitude();
    linalg::Vector closestVector;
    float projection;

    linalg::Vector springUnit = springVector / springLength;
    projection = ballToStart.dot(springUnit);

    if (projection < 0.0f)
    {
        closestVector = ballToStart;
    }
    else if (projection > springLength)
    {
        closestVector = ballToEnd;
    }
    else
    {
        linalg::Vector projectionvector = springUnit * projection;
        closestVector = ballToStart - projectionvector;
    }

    float closestDistance = closestVector.magnitude();
    float overlap = this->radius - closestDistance;

    if (overlap > 0)
    {
        linalg::Vector normal = closestVector / closestDistance;

        float fractionB = 0.0f;
        float fractionA = 1.0f;

        if (projection > 0.0f && projection < springLength)
        {
            fractionB = projection / springLength;
            fractionA = 1.0f - fractionB;
        }
        else if (projection >= springLength)
        {
            fractionB = 1.0f;
            fractionA = 0.0f;
        }

        linalg::Vector velOfContactPoint = (spring.ball1.vel * fractionA) + (spring.ball2.vel * fractionB);

        linalg::Vector relativeVel = this->vel - velOfContactPoint;
        float velNormal = relativeVel.dot(normal);

        if (velNormal < 0.0f)
        {
            float invMassThis = 1.0f / this->mass;
            float invMassB1 = (spring.ball1.mass > 1e-6f) ? 1.0f / spring.ball1.mass : 0.0f;
            float invMassB2 = (spring.ball2.mass > 1e-6f) ? 1.0f / spring.ball2.mass : 0.0f;

            float totalInvMass = invMassThis + invMassB1 + invMassB2;

            this->pos = this->pos + normal * (overlap * (invMassThis / totalInvMass));
            spring.ball1.pos = spring.ball1.pos - normal * (overlap * (invMassB1 / totalInvMass));
            spring.ball2.pos = spring.ball2.pos - normal * (overlap * (invMassB2 / totalInvMass));

            float invMassEffectiveSpring = (fractionA * fractionA * invMassB1) + (fractionB * fractionB * invMassB2);

            float elasticity = this->elasticity;
            float impulseMagnitude = -(1.0f + elasticity) * velNormal / (invMassThis + invMassEffectiveSpring);

            linalg::Vector impulseVector = normal * impulseMagnitude;

            this->vel = this->vel + impulseVector * invMassThis;

            linalg::Vector reactionImpulse = impulseVector * -1.0f;
            spring.ball1.vel = spring.ball1.vel + (reactionImpulse * fractionA) * invMassB1;
            spring.ball2.vel = spring.ball2.vel + (reactionImpulse * fractionB) * invMassB2;
        }
    }
}

// void graphs::Ball::checks
void graphs::Ball::checkWallCollision()
{

    if (this->pos.y + this->radius >= 900) // bottom wall
    {
        this->pos.y = 900 - this->radius;
        this->vel.y *= -this->elasticity;
        this->isTouchWall = true;
    }
    if (this->pos.y - this->radius <= 0) // top wall
    {
        this->pos.y = 0 + this->radius;
        this->vel.y *= -this->elasticity;
        this->isTouchWall = true;
    }
    if (this->pos.x + this->radius >= 1200) // right wall
    {
        this->pos.x = 1200 - this->radius;
        this->vel.x *= -this->elasticity;
        this->isTouchWall = true;
    }
    if (this->pos.x - this->radius <= 0) // left wall
    {
        this->pos.x = 0 + this->radius;
        this->vel.x *= -this->elasticity;
        this->isTouchWall = true;
    }
}