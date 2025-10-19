#pragma once
#include "vector.hpp"
#include "ball.hpp"
#include "soft-body.hpp"
#include <array>
#include <optional>
#include <functional>

namespace graphs
{
    class Ball;
    class SoftBody;

    class Spring;
    extern std::vector<Spring> Springs;

    class Spring
    {
    public:
        // properties
        std::array<sf::Vertex, 2> line;
        std::optional<std::reference_wrapper<graphs::SoftBody>> body;
        graphs::Ball &ball1, &ball2;
        linalg::Vector springForce;
        sf::Color color;
        float currentLength, normalLength, springCoefficient;

        // construcer
        Spring(graphs::SoftBody &body, graphs::Ball &ball1, graphs::Ball &ball2, float normalLength, float springCoefficient, sf::Color color = sf::Color::White);
        Spring(graphs::Ball &ball1, graphs::Ball &ball2, float normalLength, float springCoefficient, sf::Color color = sf::Color::White);

        // methods
        void update();
        void draw(sf::RenderWindow &window);
        void computeSpringForce();
    };
}