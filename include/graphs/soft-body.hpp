#pragma once
#include "vector.hpp"
#include "ball.hpp"
#include "spring.hpp"
#include "SFML/Graphics.hpp"
#include <vector>

namespace graphs
{
    class SoftBody;
    extern std::vector<graphs::SoftBody> SoftBodys;

    class SoftBody
    {
    public:
        sf::ConvexShape body;
        linalg::Vector center, prevPos;
        sf::Color color;
        std::vector<graphs::Ball> cornerBalls;
        std::vector<graphs::Spring> edgeSprings;
        float radius, mass, elasticity, springStiffness, pressureStiffness, restArea;
        int pointCount;
        bool isBeingDragged;

        SoftBody(linalg::Vector center, sf::Color color, int pointCount, float radius, float mass, float elasticity, float springStiffness, float pressureStiffness);

        void update();
        void draw(sf::RenderWindow &window);
        float getCurrentArea() const;
        void computePressureForce();
        void projectileMotion(linalg::Vector &mouseVector, float deltaTime);
    };

}