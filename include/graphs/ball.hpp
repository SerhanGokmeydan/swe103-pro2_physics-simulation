#pragma once
#include "vector.hpp"
#include <SFML/Graphics.hpp>

namespace graphs
{
    class Ball;
    class Spring;
    extern std::vector<Ball> Balls;

    class Ball
    {
    public:
        // properties
        sf::CircleShape shape;
        sf::Color color;
        linalg::Vector prevPos, pos, vel, acc, force, gravity, frictionForce, dragForce, springForce, pressureForce;
        float radius, mass, elasticity;
        bool isBeingDragged, isTouchWall;

        // constructer
        Ball(linalg::Vector pos, sf::Color color, float radius, float mass, float elasticity);

        // methods
        void update(float dt);
        void draw(sf::RenderWindow &window);
        void computeDragForce();
        void computeFrictionForce();
        void projectileMotion(linalg::Vector &mousePos, float elapsed);
        void checkBallCollision(Ball &ball);
        void checkSpringCollision(graphs::Spring &spring);
        void checkWallCollision();
    };
}